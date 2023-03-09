#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "anafi_uav_interfaces/msg/move_by_command.hpp"
#include "anafi_uav_interfaces/action/move_to_ned.hpp"

enum class MoveState{ HOVER, MOVE }; 


class MoveActionServer : public rclcpp::Node
{
public:
  using MoveToNED = anafi_uav_interfaces::action::MoveToNED;
  using GoalHandleMoveToNED = rclcpp_action::ServerGoalHandle<MoveToNED>;

  explicit MoveActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_action_server", options)
  {
    cmd_move_by_pub_ = this->create_publisher<anafi_uav_interfaces::msg::MoveByCommand>(
      "/anafi/cmd_moveby", rclcpp::QoS(1).reliable());

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<MoveToNED>(
      this,
      "/action_servers/move",
      std::bind(&MoveActionServer::handle_goal, this, _1, _2),
      std::bind(&MoveActionServer::handle_cancel, this, _1),
      std::bind(&MoveActionServer::handle_accepted, this, _1)
    );
  }

private:
  // State
  MoveState move_state_{ MoveState::HOVER };

  double radius_of_acceptance_{ 1.0 };
  Eigen::Quaterniond attitude_{ 1, 0, 0, 0 }; // w, x, y, z

  std::string anafi_state_;
  geometry_msgs::msg::PointStamped position_ned_;
  geometry_msgs::msg::PointStamped goal_position_ned_;

  const std::vector<std::string> allowed_anafi_states_ = { "FS_HOVERING", "FS_FLYING" };

  // Publishers
  rclcpp::Publisher<anafi_uav_interfaces::msg::MoveByCommand>::SharedPtr cmd_move_by_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::ConstSharedPtr attitude_sub_;

  // Actions
  rclcpp_action::Server<MoveToNED>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToNED::Goal> goal)
  {
    if(anafi_state_.empty())
    {
      // State uncertain - cannot move
      RCLCPP_ERROR(this->get_logger(), "Uncertain state of the Anafi! Rejecting action execution...");
      return rclcpp_action::GoalResponse::REJECT;
    }

    radius_of_acceptance_ = goal->spherical_radius_of_acceptance;
    goal_position_ned_.point = goal->ned_position;
    goal_position_ned_.header.stamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Received request to start execution");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToNED>)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void handle_accepted(const std::shared_ptr<GoalHandleMoveToNED> goal_handle)
  {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
  }


  void execute(const std::shared_ptr<GoalHandleMoveToNED> goal_handle)
  {
    // This is where the fun begins...
    // Good luck!

    RCLCPP_INFO(
      this->get_logger(),
      "Moving to position (NED): {" 
      + std::to_string(goal_position_ned_.point.x) + ", " 
      + std::to_string(goal_position_ned_.point.y) + ", " 
      + std::to_string(goal_position_ned_.point.z) 
      +"}"
    );

    rclcpp::Rate loop_rate(2);

    int start_move_counter = 0;
    const int max_start_move_counter = 10;
 
    double initial_distance = get_position_error_ned().norm();
    bool target_achieved = initial_distance <= radius_of_acceptance_;

    auto result = std::make_shared<MoveToNED::Result>();

    while(rclcpp::ok() && ! target_achieved)
    {
      if (goal_handle->is_canceling()) 
      {
        hover_();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if(move_state_ == MoveState::HOVER)
      {
        if(! check_hovering_())
        {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Trying to hover");
          hover_();
          continue;
        }

        if(! check_goal_achieved_())
        {
          // Target not achieved
          RCLCPP_WARN(this->get_logger(), "Hovering while not achieved goal position...");
          move_state_ = MoveState::MOVE;
          continue;
        }

        // Target achieved
        target_achieved = true;
        RCLCPP_INFO(this->get_logger(), "Hovering close to goal position");

        break;
      }
      else if(move_state_ == MoveState::MOVE)
      {
        bool hovering = check_hovering_();
        bool goal_achieved = check_goal_achieved_();

        Eigen::Vector3d pos_error_ned = get_position_error_ned();
        Eigen::Vector3d pos_error_body = attitude_.toRotationMatrix().transpose() * pos_error_ned; 
        double distance = pos_error_ned.norm(); 

        if(goal_achieved)
        {
          RCLCPP_INFO(this->get_logger(), "Goal achieved during move");
          move_state_ = MoveState::HOVER;
        }
        else if(hovering)
        {
          // Using a counter to ensure that commands are not sent frequenctly
          // and to ensure that the drone must be hovering for some time before 
          // a new move-command is transmitted
          
          if(start_move_counter >= max_start_move_counter)
          {
            float dx = -static_cast<float>(pos_error_body.x());
            float dy = -static_cast<float>(pos_error_body.y());
            float dz = -static_cast<float>(pos_error_body.z());

            RCLCPP_WARN(this->get_logger(), "Move ordered: x = %f, y = %f, z = %f", dx, dy, dz);

            pub_moveby_cmd(dx, dy, dz);
            start_move_counter = 0;
          }
          else 
          {
            start_move_counter++;
          }
        }

        std::shared_ptr<anafi_uav_interfaces::action::MoveToNED_Feedback> feedback = std::make_shared<MoveToNED::Feedback>();
        feedback->percentage_complete = 100.0 * std::min(1.0, (float) 100.0 * (1.0 - distance / initial_distance));

        goal_handle->publish_feedback(feedback);
      }
    }

    if (rclcpp::ok()) 
    {
      result->success = true;
      goal_handle->succeed(result);
    }
  }

  bool check_hovering_()
  {
    return anafi_state_.compare("FS_HOVERING") == 0; 
  }


  bool check_goal_achieved_()
  {
    Eigen::Vector3d pos_error_ned = get_position_error_ned();
    double distance = pos_error_ned.norm();
    return distance <= radius_of_acceptance_;
  }


  bool check_move_preconditions_()
  {
    // Currently assume that it can always move if the drone is either flying or hovering
    return (anafi_state_.compare("FS_HOVERING") == 0) || (anafi_state_.compare("FS_FLYING") == 0);
  }


  void hover_()
  {
    if(check_hovering_())
    {
      // Already hovering
      return;
    }
    pub_moveby_cmd(0.0, 0.0, 0.0);
  }


  void pub_moveby_cmd(float dx, float dy, float dz)
  {
    anafi_uav_interfaces::msg::MoveByCommand moveby_cmd = anafi_uav_interfaces::msg::MoveByCommand();
    moveby_cmd.header.stamp = this->now();
    moveby_cmd.dx = dx;
    moveby_cmd.dy = dy;
    moveby_cmd.dz = dz;
    moveby_cmd.dyaw = 0;

    cmd_move_by_pub_->publish(moveby_cmd);
  }


  /**
   * @brief Calculates the positional error in NED-frame
   */
  Eigen::Vector3d get_position_error_ned()
  {
    geometry_msgs::msg::Point pos_ned = position_ned_.point;
    geometry_msgs::msg::Point goal_pos_ned = goal_position_ned_.point;

    double x_diff = pos_ned.x - goal_pos_ned.x;
    double y_diff = pos_ned.y - goal_pos_ned.y;
    double z_diff = pos_ned.z - goal_pos_ned.z;

    Eigen::Vector3d error_ned;
    error_ned << x_diff, y_diff, z_diff;
    return error_ned;
  }


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
  {
    std::string state = state_msg->data;
    auto it = std::find_if(allowed_anafi_states_.begin(), allowed_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; });
    if(it == allowed_anafi_states_.end())
    {
      // No state found
      return;
    }
    anafi_state_ = state;
  }


  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
  {
    // Assume that the message is more recent for now... (bad assumption)
    position_ned_.header.stamp = ned_pos_msg->header.stamp;
    position_ned_.point = ned_pos_msg->point;
  }


  void attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg)
  {
    Eigen::Vector3d v(attitude_msg->quaternion.x, attitude_msg->quaternion.y, attitude_msg->quaternion.z);
    attitude_.w() = attitude_msg->quaternion.w;
    attitude_.vec() = v;
    attitude_.normalize();
  }

};  // class MoveActionServer

RCLCPP_COMPONENTS_REGISTER_NODE(MoveActionServer)
