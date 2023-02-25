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
#include "anafi_uav_interfaces/srv/set_equipment_numbers.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;


enum class Severity{ MINOR, MODERATE, HIGH };
enum class Equipment{ MARKER, LIFEVEST };


class DropEquipmentActionNode : public plansys2::ActionExecutorClient
{
public:
  DropEquipmentActionNode() 
  : plansys2::ActionExecutorClient("drop_equipment_action_node", 500ms)
  , node_activated_(false)
  , selected_equipment_(Equipment::MARKER)
  {
    /**
     * Declare parameters
     */ 
    std::string mission_init_prefix = "mission_init.";
    this->declare_parameter(mission_init_prefix + "start_location", std::string());
    this->declare_parameter(mission_init_prefix + "locations_available", std::vector<std::string>());

    std::string payload_prefix = mission_init_prefix + "payload.";
    this->declare_parameter(payload_prefix + "num_markers", int());
    this->declare_parameter(payload_prefix + "num_lifevests", int());

    /**
     * Initialize subscriptions and services
     */
    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&DropEquipmentActionNode::anafi_state_cb_, this, _1));   
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", rclcpp::QoS(1).best_effort(), std::bind(&DropEquipmentActionNode::ned_pos_cb_, this, _1));   
    detected_person_sub_ = this->create_subscription<anafi_uav_interfaces::msg::DetectedPerson>(
      "estimate/detected_person", rclcpp::QoS(1).best_effort(), std::bind(&DropEquipmentActionNode::detected_person_cb_, this, _1));
  
    set_num_markers_client_ = this->create_client<anafi_uav_interfaces::srv::SetEquipmentNumbers>("/mission_controller/num_markers");
    set_num_lifevests_client_ = this->create_client<anafi_uav_interfaces::srv::SetEquipmentNumbers>("/mission_controller/num_lifevests");
  }

  void set_equipment(Equipment selected_equipment) { selected_equipment_ = selected_equipment; }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  bool node_activated_;
  int num_markers_;
  int num_lifevests_;

  std::string anafi_state_;

  Equipment selected_equipment_;

  std::tuple<geometry_msgs::msg::Point, Severity> detected_person_;
  std::vector<geometry_msgs::msg::Point> previously_helped_people_;
  geometry_msgs::msg::PointStamped position_ned_;

  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };


  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::DetectedPerson>::ConstSharedPtr detected_person_sub_;

  // Services
  rclcpp::Client<anafi_uav_interfaces::srv::SetEquipmentNumbers>::SharedPtr set_num_markers_client_;
  rclcpp::Client<anafi_uav_interfaces::srv::SetEquipmentNumbers>::SharedPtr set_num_lifevests_client_;  

  // Private functions
  /**
   * @brief Overload of function in ActionExecutorClient. This function does the 
   * majority of the work when the node is activated. 
   * 
   * The function is called at the same frequency as the node, and is responsible for
   *    - Dropping the marker if the detected person has not been helped before. The 
   *      marker will only be dropped to the person once, due to a limited amount of 
   *      carrying-capacity 
   */
  void do_work();


  /**
   * @brief Checking that the drone is capable of dropping:
   *  - hovering above and slightly to the left of the target position 
   *  - the severity of the incident is high enough to justify a drop of a marker
   * 
   * @warning It is assumed that the object of interest is tracked before the function 
   * being dropped
   */
  bool check_drop_preconditions();

  /**
   * @brief Drops equipment on the desired position, if possible and update the 
   * mission-controller about the number of equipment remaining
   */
  bool drop_marker_();
  bool drop_lifevest_();
  void update_controller_of_equipment_status_();


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg);
  void detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg);

}; // DropEquipmentActionNode
