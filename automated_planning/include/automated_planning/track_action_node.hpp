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
#include "anafi_uav_interfaces/action/move_to_ned.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;


class TrackActionNode : public plansys2::ActionExecutorClient
{
public:
  TrackActionNode() 
  : plansys2::ActionExecutorClient("track_node", 250ms)
  , radius_of_acceptance_(0.2)
  {
    this->declare_parameter("track.radius_of_acceptance"); // Fail if not found in config
    radius_of_acceptance_ = this->get_parameter("track.radius_of_acceptance").as_double();

    // Subscribers
    using namespace std::placeholders;
    detected_person_sub_ = this->create_subscription<anafi_uav_interfaces::msg::DetectedPerson>(
      "estimate/detected_person", rclcpp::QoS(1).best_effort(), std::bind(&TrackActionNode::detected_person_cb_, this, _1));
    apriltags_detected_sub_ = this->create_subscription<anafi_uav_interfaces::msg::Float32Stamped>(
      "/estimate/aprilTags/num_tags_detected", rclcpp::QoS(1).best_effort(), std::bind(&TrackActionNode::apriltags_detected_cb_, this, _1));  

    // Actions
    move_action_client_ = rclcpp_action::create_client<anafi_uav_interfaces::action::MoveToNED>(this, "/action_servers/track");
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  double radius_of_acceptance_;

  geometry_msgs::msg::Point position_ned_;
  geometry_msgs::msg::Point goal_position_ned_;

  std::map<std::string, std::tuple<rclcpp::Time, geometry_msgs::msg::Point>> last_detections_; // <Apriltags/Person, last detected time, estimated position>   

  // Subscribers
  rclcpp::Subscription<anafi_uav_interfaces::msg::DetectedPerson>::ConstSharedPtr detected_person_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::Float32Stamped>::ConstSharedPtr apriltags_detected_sub_;

  // Actions
  using MoveGoalHandle = rclcpp_action::ClientGoalHandle<anafi_uav_interfaces::action::MoveToNED>;
  using MoveFeedback = const std::shared_ptr<const anafi_uav_interfaces::action::MoveToNED::Feedback>;

  rclcpp_action::Client<anafi_uav_interfaces::action::MoveToNED>::SharedPtr move_action_client_;
  std::shared_future<MoveGoalHandle::SharedPtr> future_move_goal_handle_;
  MoveGoalHandle::SharedPtr move_goal_handle_;
  anafi_uav_interfaces::action::MoveToNED::Goal move_goal_;


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

  // Callbacks
  void detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg);
  void apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg);

}; // TrackActionNode
