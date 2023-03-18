#pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <math.h>
#include <map>
#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "anafi_uav_interfaces/msg/float32_stamped.hpp"
#include "anafi_uav_interfaces/msg/ekf_output.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;


enum class HelipadState{ GO, NO_GO };
enum LandingState{ INIT, HOVER_DIRECTLY_ABOVE, LAND };


class LandActionNode : public plansys2::ActionExecutorClient
{
public:
  LandActionNode() 
  : plansys2::ActionExecutorClient("land_action_node", 250ms)
  , battery_percentage_(-1)
  , helipad_detected_(false)
  {
    // Target positions during landing
    // This assumes that the target position in the GNC can be set
    // These positions are relative to the helipad, and therefore the output from the EKF
    geometry_msgs::msg::Point p_init, p_hover, p_helipad;
    p_init.x = 0;     p_init.y = 0;     p_init.z = -2.0;
    p_hover.x = 0;    p_hover.y = 0;    p_hover.z = -0.6;
    p_helipad.x = 0;  p_helipad.y = 0;  p_helipad.z = 0.0;

    landing_points_[LandingState::INIT] =  p_init;
    landing_points_[LandingState::HOVER_DIRECTLY_ABOVE] = p_hover;
    landing_points_[LandingState::LAND] = p_helipad;

    // Initializing time to zero, to ensure that all future detection-messages are valid
    last_apriltags_detection_time_ = rclcpp::Clock{RCL_ROS_TIME}.now();

    // May have some problems with QoS when interfacing with ROS1
    // The publishers are on mode reliable, to increase the likelihood of sending the message
    // The subscribers are on best-effort, since similar data will be transmitted several times. If
    // some packages are lost, so be it.
    cmd_land_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/anafi/cmd_land", rclcpp::QoS(1).reliable());
    desired_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>( // Would perhaps be better to use a service for this, 
      "/guidance/desired_ned_position", rclcpp::QoS(1).reliable());                   // but custom srvs not functioning properly with ros1-bridge

    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::anafi_state_cb_, this, _1));   
    battery_charge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/anafi/battery", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::battery_charge_cb_, this, _1));   
    ekf_sub_ = this->create_subscription<anafi_uav_interfaces::msg::EkfOutput>(
      "/estimate/ekf", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::ekf_cb_, this, _1));   
    polled_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/anafi/polled_body_velocities", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::polled_vel_cb_, this, _1));   
    apriltags_detected_sub_ = this->create_subscription<anafi_uav_interfaces::msg::Float32Stamped>(
      "/estimate/aprilTags/num_tags_detected", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::apriltags_detected_cb_, this, _1));  

    // Assuming the velocity controller will be used throughout this thesis
    // Future improvement to allow for using the MPC
    enable_velocity_control_client_ = this->create_client<std_srvs::srv::SetBool>("/velocity_controller/service/enable_controller"); 
    
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  std::string anafi_state_;
  double battery_percentage_;
  
  bool helipad_detected_;
  rclcpp::Time last_apriltags_detection_time_;
  const double max_last_apriltags_detection_time_s_{ 5.0 };   

  HelipadState helipad_state_{ HelipadState::GO };  // Assume in this thesis, that the helipad is ready
  LandingState landing_state_{ LandingState::INIT };  

  geometry_msgs::msg::Point desired_position_;
  geometry_msgs::msg::TwistStamped polled_vel_;

  anafi_uav_interfaces::msg::EkfOutput ekf_output_;

  std::map<LandingState, geometry_msgs::msg::Point> landing_points_;
  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };


  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr cmd_land_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_position_pub_;


  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr battery_charge_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::EkfOutput>::ConstSharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr polled_vel_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::Float32Stamped>::ConstSharedPtr apriltags_detected_sub_;


  // Services
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_velocity_control_client_;


  // Private functions
  /**
   * @brief Overload of function in ActionExecutorClient. This function does the 
   * majority of the work when the node is activated. 
   * 
   * The function is called at the same frequency as the node, and is responsible for
   *    - Check if the drone has not taken off
   *      - If it has taken too long, call takeoff again
   *      - If a maximum number of takeoff tries are reached, fail the task
   */
  void do_work();


  /**
   * @brief Checking preconditions for takeoff, including battery charge and drone state
   */
  bool check_land_preconditions();

  /**
   * @brief Check if inside a no-go zone around the helipad. 
   * The no-go zone is defined based on the radius of the helipad, and the expected
   * trajectory during landing. Far more complex version could be developed...  
   * 
   * The outcome will vary depending on the state of the landing. It will be stricter
   * the closer the drone is to the helipad.
   * 
   * @warning The function relies on the estimate from the EKF. If the latter is fucked
   * the estimated position could cause dangerous behavior. Always ensure that the 
   * postion estimate from the EKF is somewhat reliable before calling this function.
   * It does NOT currently use the estimated covariance from the EKF.
   */
  bool check_inside_no_go_zone();


  /**
   * @brief As the name suggests, this function will check whether the desired position 
   * is achieved or not. The radius of acceptance will depend slightly on the state, and
   * having stricter requirements the closer to helipad one gets 
   */
  bool check_target_position_achieved();


  /**
   * @brief Helper functions for publishing messages
   */
  void publish_desired_position_();


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg);
  void ekf_cb_(anafi_uav_interfaces::msg::EkfOutput::ConstSharedPtr ekf_msg);
  void polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);
  void apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg);

}; // LandActionNode
