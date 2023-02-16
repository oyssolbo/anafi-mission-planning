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
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
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

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "anafi_uav_interfaces/msg/move_by_command.hpp"
#include "anafi_uav_interfaces/msg/move_to_command.hpp"
#include "anafi_uav_interfaces/msg/ekf_output.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction() 
  : plansys2::ActionExecutorClient("move", 250ms),
    start_distance_(1),         // Initialize as non-zero to prevent div by 0
    radius_of_acceptance_(0.75)
  {
    // Initialize the mission objectives from somewhere. This could be done during configuration tbh
    init_locations();

    // May have some problems with QoS when interfacing with ROS1
    // The publishers are on mode reliable, to increase the likelihood of sending the message
    // The subscribers are on best-effort, since similar data will be transmitted several times. If
    // some packages are lost, so be it.
    cmd_move_by_pub_ = this->create_publisher<anafi_uav_interfaces::msg::MoveByCommand>(
      "/anafi/cmd_moveby", rclcpp::QoS(1).reliable());
    cmd_move_to_pub_ = this->create_publisher<anafi_uav_interfaces::msg::MoveToCommand>(
      "/anafi/cmd_moveto", rclcpp::QoS(1).reliable());  

    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&MoveAction::anafi_state_cb_, this, _1));   
    ekf_output_sub_ = this->create_subscription<anafi_uav_interfaces::msg::EkfOutput>(
      "/estimate/ekf", rclcpp::QoS(1).best_effort(), std::bind(&MoveAction::ekf_cb_, this, _1));    
    gnss_data_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/anafi/gnss_location", rclcpp::QoS(1).best_effort(), std::bind(&MoveAction::gnss_data_cb_, this, _1));
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", rclcpp::QoS(1).best_effort(), std::bind(&MoveAction::ned_pos_cb_, this, _1));    
    polled_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/anafi/polled_body_velocities", rclcpp::QoS(1).best_effort(), std::bind(&MoveAction::polled_vel_cb_, this, _1));   
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  double start_distance_;
  double radius_of_acceptance_;

  Eigen::Quaterniond attitude_{ 1, 0, 0, 0 }; // w, x, y, z

  std::string anafi_state_;
  anafi_uav_interfaces::msg::EkfOutput ekf_output_;
  geometry_msgs::msg::TwistStamped polled_vel_;
  geometry_msgs::msg::PointStamped position_ned_;
  geometry_msgs::msg::PointStamped goal_position_ned_;
  std::map<std::string, geometry_msgs::msg::PointStamped> locations_;

  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<anafi_uav_interfaces::msg::MoveByCommand>::SharedPtr cmd_move_by_pub_;
  rclcpp_lifecycle::LifecyclePublisher<anafi_uav_interfaces::msg::MoveToCommand>::SharedPtr cmd_move_to_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::EkfOutput>::ConstSharedPtr ekf_output_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gnss_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::ConstSharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr polled_vel_sub_;


  // Private functions
  /**
   * @brief Initializing some of the hardcoded values for now. Should be loaded in
   * from a config file eventually
   */
  void init_locations();

  /**
   * @brief Overload of function in ActionExecutorClient. This function does the 
   * majority of the work when the node is activated. 
   * 
   * The function is called at the same frequency as the node, and is responsible for
   *    - Checks whether the drone is moving
   *      - if yes: 
   *        - Check distance to goal and successfully terminate if reached 
   *        - Possible update the movement vector
   *      - if no: 
   *        - Calculates the desired movement, and requests the drone to move
   *        - If the drone has not moved in k number of attempts, fail the move 
   *    - The function will publish the progress to the executor, such that the overall
   *      system can monitor the progress
   */
  void do_work();


  /**
   * @brief Functions checking drone movement:
   *  - checking preconditions for movement 
   *  - checking whether it is currently moving along a vector
   * respectively.
   */
  bool check_move_preconditions();
  bool check_movement_along_vector(const Eigen::Vector3d& vec);


  /**
   * @brief Definitions to publish commands easier
   * 
   * @warning Assumes that the publishers are already activated
   * @warning Floats used in the moveby_cmd and not in moveto_cmd
   */
  void pub_moveby_cmd(float dx, float dy, float dz);
  void pub_moveto_cmd(double lat, double lon, double h);


  /**
   * @brief Calculates the positional error
   */
  Eigen::Vector3d get_position_error_ned();


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void ekf_cb_(anafi_uav_interfaces::msg::EkfOutput::ConstSharedPtr ekf_msg);
  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg);
  void gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg);
  void attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg);
  void polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);

}; // MoveAction
