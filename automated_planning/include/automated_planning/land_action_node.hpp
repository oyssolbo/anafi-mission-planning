#pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "anafi_uav_interfaces/msg/float32_stamped.hpp"
#include "anafi_uav_interfaces/msg/point_with_covariance_stamped.hpp"

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
  , is_gnc_activated_(false)
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
    last_apriltags_detection_time_ = this->get_clock()->now(); // rclcpp::Clock{RCL_ROS_TIME}.now();

    // Callback groups
    // Wait, the callback group caused a problem????
    // service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);


    // Publishers
    cmd_land_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/anafi/cmd_land", rclcpp::QoS(1).reliable());
    desired_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>( // Would perhaps be better to use a service for this, 
      "/guidance/desired_ned_position", rclcpp::QoS(1).reliable());                   // but custom srvs not functioning properly with ros1-bridge

    // Subscribers
    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::anafi_state_cb_, this, _1));   
    battery_charge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/anafi/battery", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::battery_charge_cb_, this, _1));   
    ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/estimate/ekf", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::ekf_cb_, this, _1));   
    polled_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/anafi/polled_body_velocities", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::polled_vel_cb_, this, _1));   
    apriltags_detected_sub_ = this->create_subscription<anafi_uav_interfaces::msg::Float32Stamped>(
      "/estimate/aprilTags/num_tags_detected", rclcpp::QoS(1).best_effort(), std::bind(&LandActionNode::apriltags_detected_cb_, this, _1));  

    // Services
    // Assuming the velocity controller will be used throughout this thesis
    // Future improvement to allow for using the MPC
    enable_velocity_control_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/velocity_controller/service/enable_controller");//, rmw_qos_profile_services_default, service_callback_group_); 
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  std::string anafi_state_;
  double battery_percentage_;
  
  bool is_gnc_activated_;
  bool helipad_detected_;
  rclcpp::Time last_apriltags_detection_time_;
  const double max_last_apriltags_detection_time_s_{ 5.0 };   

  HelipadState helipad_state_{ HelipadState::GO };  // Assume in this thesis, that the helipad is ready
  LandingState landing_state_{ LandingState::INIT };  

  geometry_msgs::msg::Point desired_position_;
  geometry_msgs::msg::TwistStamped polled_vel_;

  geometry_msgs::msg::PoseWithCovarianceStamped ekf_output_;

  std::map<LandingState, geometry_msgs::msg::Point> landing_points_;
  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };


  // Callback-groups
  // rclcpp::CallbackGroup::SharedPtr service_callback_group_;


  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr cmd_land_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr desired_position_pub_;


  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr battery_charge_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr ekf_sub_;
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
   * @brief Helper function for publishing messages
   * 
   * @warning This will currently publish the point @p desired_position_ to the 
   * guidance-module. The point @p desired_position_ is assumed to be in NED. It will 
   * therefore only be valid for a single location, namely the origin of mission start!
   */
  void publish_desired_position_();


  /**
   * @brief Quick setting of the GNC
   * 
   * @param enable_controller Boolean which determines the desired state of the 
   * GNC
   *  true  ->  Activates the GNC
   *  false ->  Disables the GNC
   * 
   * @warning Will not wait on the response!
   * @warning Sets the boolean @a is_gnc_activated_ to @p enable_controller
   * This should be changed to use the response from GNC (when waiting on response
   * works (AKA when someone competent takes over this monstrousity))
   */
  void set_controller_state_(bool enable_controller, const std::string& log_str);


  /**
   * @brief Calculates the lower and upper altitude bound given a LandingState @p state
   * 
   * @warning The bounds are given as altitude, and NOT in NED
   */    
  std::pair<double, double> get_altitude_bounds_(const LandingState& state, const geometry_msgs::msg::Point& point);


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg);
  void ekf_cb_(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr ekf_msg);
  void polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);
  void apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg);

}; // LandActionNode
