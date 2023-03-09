#pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <math.h>
#include <map>
#include <stdint.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "anafi_uav_interfaces/msg/detected_person.hpp"
#include "anafi_uav_interfaces/msg/float32_stamped.hpp"
#include "anafi_uav_interfaces/srv/set_equipment_numbers.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;


class TrackActionNode : public plansys2::ActionExecutorClient
{
public:
  TrackActionNode() 
  : plansys2::ActionExecutorClient("track_action_node", 250ms)
  , node_activated_(false)
  , radius_of_acceptance_(0.2)
  {
    // Publishers
    goal_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/guidance/desired_ned_position", rclcpp::QoS(1).reliable());

    // Subscribers
    using namespace std::placeholders;
    detected_person_sub_ = this->create_subscription<anafi_uav_interfaces::msg::DetectedPerson>(
      "estimate/detected_person", rclcpp::QoS(1).best_effort(), std::bind(&TrackActionNode::detected_person_cb_, this, _1));
    apriltags_detected_sub_ = this->create_subscription<anafi_uav_interfaces::msg::Float32Stamped>(
      "/estimate/aprilTags/num_tags_detected", rclcpp::QoS(1).best_effort(), std::bind(&TrackActionNode::apriltags_detected_cb_, this, _1));  

    // Services
    enable_velocity_control_client_ = this->create_client<std_srvs::srv::SetBool>("/velocity_controller/service/enable_controller"); 
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  bool node_activated_;
  double radius_of_acceptance_;

  geometry_msgs::msg::Point position_ned_;
  geometry_msgs::msg::Point goal_position_ned_;

  std::map<std::string, std::tuple<rclcpp::Time, geometry_msgs::msg::Point>> last_detections_; // <Apriltags/Person, last detected time, estimated position>   

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_position_pub_;

  // Subscribers
  rclcpp::Subscription<anafi_uav_interfaces::msg::DetectedPerson>::ConstSharedPtr detected_person_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::Float32Stamped>::ConstSharedPtr apriltags_detected_sub_;

  // Services
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_velocity_control_client_;
 

  // Private functions
  /**
   * @brief Overload of function in ActionExecutorClient. This function does the 
   * majority of the work when the node is activated
   */
  void do_work();


  /**
   * @brief Based on the current detections, it calculates the target position for tracking either
   * helipad / landing locations or detected people. 
   * 
   * If the argument @p preferred_target is non-empty, the function tries to move to its location if 
   * it has been detected within the @p time_limit_s 
   * 
   * If @p preferred_target is empty, the function sets the target as the most recent one. Beware, that
   * the behaviour will be ill-defined if there are multiple objects detected simultaneously!
   */
  bool get_target_position_(const std::string& preferred_target="", double time_limit_s=5.0);

  /**
   * @brief Functions copied into this node from the track action server. These are used to 
   * ensure that the drone can use the guidance and velocity control
   * 
   * For whomever comes after: 
   * Using the TrackActionServer does not work. Library error I haven't quite figured out to solve.
   * Originally developed to use an action server, but quickly changed to utilize the GNC instead.
   * Sorry for the mess! 
   */
  bool set_velocity_controller_state_(bool controller_state, const std::string& error_str="");
  bool check_goal_achieved_();
  void hover_();
  void pub_desired_ned_position_(const geometry_msgs::msg::Point& target_position);
  Eigen::Vector3d get_position_error_ned_();
  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg);

  // Callbacks
  void detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg);
  void apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg);

}; // TrackActionNode
