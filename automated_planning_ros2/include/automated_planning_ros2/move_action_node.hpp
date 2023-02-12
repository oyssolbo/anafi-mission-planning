#pragma once

#include <memory>
#include <string>
#include <map>
#include <algorithm>
#include <math.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "anafi_uav_interfaces/msg/move_by_command.hpp"
#include "anafi_uav_interfaces/msg/move_to_command.hpp"
#include "anafi_uav_interfaces/msg/ekf_output.hpp"

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction() 
  : plansys2::ActionExecutorClient("move", 250ms)
  {
    // Initialize the mission objectives from somewhere
    init_locations();

    // May have some problems with QoS when interfacing with ROS1
    cmd_move_by_pub_ = this->create_publisher<anafi_uav_interfaces::msg::MoveByCommand>(
      "/anafi/cmd_moveby", 1);
    cmd_move_to_pub_ = this->create_publisher<anafi_uav_interfaces::msg::MoveToCommand>(
      "/anafi/cmd_moveto", 1);

    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", 1, std::bind(&MoveAction::anafi_state_cb_, this, _1));   
    ekf_output_sub_ = this->create_subscription<anafi_uav_interfaces::msg::EkfOutput>(
      "/estimate/ekf", 1, std::bind(&MoveAction::ekf_cb_, this, _1));    
    gnss_data_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/anafi/gnss_location", 1, std::bind(&MoveAction::gnss_data_cb_, this, _1));
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", 1, std::bind(&MoveAction::ned_pos_cb_, this, _1));     
  }

  void current_pos_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pos_ = msg->pose.pose;
  }

  // Lifecycle-events
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_exit(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
  // State
  std::map<std::string, geometry_msgs::msg::PointStamped> locations_;
  std::string anafi_state_;
  anafi_uav_interfaces::msg::EkfOutput ekf_output_;
  geometry_msgs::msg::PointStamped ned_position_;
  geometry_msgs::msg::PointStamped goal_position_;

  const std::vector<std::string> possible_anafi_states_ = 
    {"FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"};

  // Publishers
  rclcpp::Publisher<anafi_uav_interfaces::msg::MoveByCommand>::SharedPtr cmd_move_by_pub_;
  rclcpp::Publisher<anafi_uav_interfaces::msg::MoveToCommand>::SharedPtr cmd_move_to_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr anafi_state_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::EkfOutput>::SharedPtr ekf_output_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ned_pos_sub_;


  // Private functions

  /**
   * @brief Initializing some of the hardcoded values for now. Should be loaded in
   * from a config file eventually
   */
  void init_locations();

  /**
   * @brief Overload of function in ActionExecutorClient
   */
  void do_work() {};

  /**
   * @brief Calculcates the horizontal difference between two poses 
   */
  double get_horizontal_distance(const geometry_msgs::msg::Pose & pos1, const geometry_msgs::msg::Pose & pos2);


  // Callbacks
  void anafi_state_cb_(const std_msgs::msg::String::SharedPtr& state_msg);
  void ekf_cb_(const anafi_uav_interfaces::msg::EkfOutput::SharedPtr& ekf_msg);
  void ned_pos_cb_(const geometry_msgs::msg::PointStamped::SharedPtr& ned_pos_msg);
  void gnss_data_cb_(const sensor_msgs::msg::NavSatFix::SharedPtr& gnss_data_msg);


  using NavigationGoalHandle =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  using NavigationFeedback =
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>;

  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client_;
  std::shared_future<NavigationGoalHandle::SharedPtr> future_navigation_goal_handle_;
  NavigationGoalHandle::SharedPtr navigation_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  geometry_msgs::msg::Pose current_pos_;
  geometry_msgs::msg::PoseStamped goal_pos_;
  nav2_msgs::action::NavigateToPose::Goal navigation_goal_;

  double dist_to_move;
};


