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
#include "std_srvs/srv/set_bool.hpp"

#include "anafi_uav_interfaces/msg/move_by_command.hpp"
#include "anafi_uav_interfaces/action/move_to_ned.hpp"

using namespace std::chrono_literals;

class TrackActionServer : public rclcpp::Node
{
public:
  using MoveToNED = anafi_uav_interfaces::action::MoveToNED;
  using GoalHandleMoveToNED = rclcpp_action::ServerGoalHandle<MoveToNED>;

  explicit TrackActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("track_action_server", options)
  {
    goal_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/guidance/desired_ned_position", rclcpp::QoS(1).reliable());

    service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create subscribers
    using namespace std::placeholders;
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", rclcpp::QoS(1).best_effort(), std::bind(&TrackActionServer::ned_pos_cb_, this, _1));   

    // Assuming the velocity controller will be used throughout this thesis
    // Future improvement to allow for using the MPC
    enable_velocity_control_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/velocity_controller/service/enable_controller", rmw_qos_profile_services_default, service_callback_group_); 

    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<MoveToNED>(
      this,
      "/action_servers/track",
      std::bind(&TrackActionServer::handle_goal, this, _1, _2),
      std::bind(&TrackActionServer::handle_cancel, this, _1),
      std::bind(&TrackActionServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    );

    RCLCPP_INFO(this->get_logger(), "Action server initialized!");
  }

private:
  // State
  double radius_of_acceptance_{ 0.2 };

  geometry_msgs::msg::PointStamped position_ned_;
  geometry_msgs::msg::PointStamped goal_position_ned_;

  // Callback-group
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_position_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::ConstSharedPtr attitude_sub_;

  // Services
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_velocity_control_client_;

  // Actions
  rclcpp_action::Server<MoveToNED>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToNED::Goal> goal)
  {
    radius_of_acceptance_ = goal->spherical_radius_of_acceptance;
    goal_position_ned_.point = goal->ned_position;
    goal_position_ned_.header.stamp = this->get_clock()->now();

    pub_desired_ned_position_(goal_position_ned_.point);

    // Todo: Have a function for formatting the strings
    std::string goal_pos_str = "{" 
      + std::to_string(goal_position_ned_.point.x) + ", " 
      + std::to_string(goal_position_ned_.point.y) + ", " 
      + std::to_string(goal_position_ned_.point.z) 
      + "}";
    std::string current_pos_str = "{" 
      + std::to_string(position_ned_.point.x) + ", " 
      + std::to_string(position_ned_.point.y) + ", " 
      + std::to_string(position_ned_.point.z) 
      + "}";

    RCLCPP_INFO(this->get_logger(), "Received request to move towards position (NED) " 
      + goal_pos_str + " from current position (NED) " + current_pos_str
    );

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToNED>)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    hover_();
    set_velocity_controller_state_(false, "Failed to disable the velocity controller! Shut it down manually!");
    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void handle_accepted(const std::shared_ptr<GoalHandleMoveToNED> goal_handle)
  {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TrackActionServer::execute, this, _1), goal_handle}.detach();
  }


  void execute(const std::shared_ptr<GoalHandleMoveToNED> goal_handle)
  {
    rclcpp::Rate loop_rate(2);
    set_velocity_controller_state_(true);
 
    double initial_distance = get_position_error_ned_().norm();
    auto result = std::make_shared<MoveToNED::Result>();

    bool goal_achieved = check_goal_achieved_();

    while(rclcpp::ok() && ! goal_achieved)
    {
      if (goal_handle->is_canceling()) 
      {
        hover_();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2500, "Current position (NED): {" 
      + std::to_string(position_ned_.point.x) + ", " 
      + std::to_string(position_ned_.point.y) + ", " 
      + std::to_string(position_ned_.point.z) 
      +"}");

      pub_desired_ned_position_(goal_position_ned_.point);

      std::shared_ptr<anafi_uav_interfaces::action::MoveToNED_Feedback> feedback = std::make_shared<MoveToNED::Feedback>();
      double distance = get_position_error_ned_().norm();
      feedback->percentage_complete = 100.0 * std::min(1.0, (float) 100.0 * (1.0 - distance / initial_distance));
      goal_handle->publish_feedback(feedback);

      goal_achieved = check_goal_achieved_();
      loop_rate.sleep();
    }

    if (rclcpp::ok() && goal_achieved) 
    {
      result->success = true;
      goal_handle->succeed(result);

      set_velocity_controller_state_(false);
      
      RCLCPP_INFO(this->get_logger(), "Move success! Current position (NED): {" 
      + std::to_string(position_ned_.point.x) + ", " 
      + std::to_string(position_ned_.point.y) + ", " 
      + std::to_string(position_ned_.point.z) 
      +"}");
    }
  }


  bool check_goal_achieved_()
  {
    Eigen::Vector3d pos_error_ned = get_position_error_ned_();
    double distance = pos_error_ned.norm();
    // RCLCPP_INFO(this->get_logger(), "Current distance " + std::to_string(distance) + " and radius of acceptance " + std::to_string(radius_of_acceptance_));
    return distance <= radius_of_acceptance_;
  }


  void hover_()
  {
    pub_desired_ned_position_(position_ned_.point);
  }


  void pub_desired_ned_position_(const geometry_msgs::msg::Point& target_position)
  {
    geometry_msgs::msg::PointStamped point_msg = geometry_msgs::msg::PointStamped();
    point_msg.header.stamp = this->get_clock()->now();
    point_msg.point = target_position;

    goal_position_pub_->publish(point_msg);
  }


  Eigen::Vector3d get_position_error_ned_()
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


  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
  {
    position_ned_.header.stamp = ned_pos_msg->header.stamp;
    position_ned_.point = ned_pos_msg->point;
  }


  bool set_velocity_controller_state_(bool controller_state, const std::string& error_str="")
  {
    // Somehow the checks always fails, even though the service is called correctly
    // Any ideas why? Can it be due to different callback-groups?
    // Must be changed in the future!
    (void) error_str;
    enable_velocity_control_client_->wait_for_service(2s);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = controller_state;
    auto result = enable_velocity_control_client_->async_send_request(request);
    // std::future_status status = result.wait_for(2s);
    // (void) status;
    return true;
    
    // if(status == std::future_status::ready)//rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   if(result.get()->success)
    //   {
    //     return true;
    //   }
    // }
    // RCLCPP_ERROR(this->get_logger(), error_str);
    // return false;
    // return true;
  }


};  // class TrackActionServer

RCLCPP_COMPONENTS_REGISTER_NODE(TrackActionServer)
