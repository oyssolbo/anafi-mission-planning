#pragma once

#include <memory>
#include <tuple>
#include <vector>
#include <tuple>
#include <string>
#include <optional>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"

#include <plansys2_pddl_parser/Utils.h>
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"


enum class Severity{ MINOR, MODERATE, HIGH };
enum class ControllerState { NORMAL_OPERATION, PERSON_DETECTED, DRONE_EMERGENCY, AREA_UNAVAILABLE };


class MissionControllerNode : public rclcpp::Node
{
public:
  MissionControllerNode()
  : rclcpp::Node("mission_controller_node") 
  , controller_state_(ControllerState::NORMAL_OPERATION)
  , battery_charge_(-1.0) // Set as negative to indicate that it is not updated
  , is_replanning_necessary_(true)
  , is_emergency_(false)
  , is_low_battery_(false)
  , is_person_detected_(false)
  {
    // Create publishers
    plan_publisher_ = this->create_publisher<plansys2_msgs::msg::Plan>(
      "/mission_controller/plan", 1);

    // Create subscribers
    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::anafi_state_cb_, this, _1));   
    battery_charge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/anafi/battery", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::battery_charge_cb_, this, _1)); 
    gnss_data_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/anafi/gnss_location", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::gnss_data_cb_, this, _1));
    attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/anafi/attitude", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::attitude_cb_, this, _1));   
    polled_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/anafi/polled_body_velocities", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::polled_vel_cb_, this, _1));  
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::ned_pos_cb_, this, _1));    
  }


  /**
   * @brief Initializes the domain expert, problem expert, planner and executor 
   */
  void init();

  /**
   * @brief Waits on the preconditions to be fulfilled
   */
  void wait_for_preconditions();

  /**
   * @brief Steps through the problem, depending on the state
   */
  void step();

private:
  // System state 
  ControllerState controller_state_;

  double battery_charge_;
  std::string anafi_state_;
  geometry_msgs::msg::QuaternionStamped attitude_;
  geometry_msgs::msg::TwistStamped polled_vel_;
  geometry_msgs::msg::PointStamped position_ned_;
  std::map<std::string, geometry_msgs::msg::PointStamped> locations_;

  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };

  // plansys2::Predicate drone_position = plansys2::Predicate();

  bool is_replanning_necessary_;
  bool is_emergency_;
  bool is_low_battery_;
  bool is_person_detected_;

  // PlanSys2 variables
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  // System predicates
  plansys2::Predicate drone_position_predicate_;
  std::vector<plansys2::Predicate> normal_operation_predicates_;
  std::vector<plansys2::Predicate> person_detected_predicates_;
  std::vector<plansys2::Predicate> emergency_predicates_;
  std::vector<plansys2::Predicate> area_unavailable_predicates_;

  // Mission objectives
  std::vector<plansys2::Goal> primary_mission_objectives_;
  std::vector<plansys2::Goal> secondary_mission_objectives_;

  // Mission variables
  std::tuple<geometry_msgs::msg::Point, Severity> detected_person_;
  std::vector<geometry_msgs::msg::Point> previously_detected_people_;
  std::vector<std::string> inaccessible_areas_{ };  // Assumed empty at start 

  // Publishers
  rclcpp::Publisher<plansys2_msgs::msg::Plan>::SharedPtr plan_publisher_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr battery_charge_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gnss_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::ConstSharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr polled_vel_sub_;


  // Private functions
  /**
   * @brief Initializes the world knowledge 
   */
  void init_knowledge_();

  /**
   * @brief Implementations of different cases according to the states 
   *        _normal_operation_  : Perform normal search and rescue missions
   *        _person_detected_   : Evaluate how critical the situation is, and potentially rescue the person
   *        _emergency_         : Save the drone, making it land on a landable position
   *        _area_unavailable_  : Force the drone to replan. Currently not implemented
   */
  void case_normal_operation_();
  void case_person_detected_();
  void case_emergency_();
  void case_area_unavailable_();

  // /**
  //  * @brief Initialize and maintain the mission objectives
  //  */
  // void init_mission_objectives_();
  // void maintain_mission_objectives_();

  /** 
   * @brief Based on the current information and state, checks if a replanning
   * is necessary
   * 
   * Replanning if necessary 
   */
  const std::tuple<ControllerState, bool> recommend_replan_(); 
  bool replan_mission_(std::optional<plansys2_msgs::msg::Plan>& plan); 

  /**
   * @brief Removes all of the following plan predicates:
   *    - normal_operation_predicates_
   *    - person_detected_predicates_
   *    - emergency_predicates_
   *    - area_unavailable_predicates_
   * from the plan, such that a new state does not use predicates from other states 
   */
  bool remove_predicates_();

  /**
   * @brief Keeps an account of major objectives 
   */
  bool objectives_completed_() const { return false; }
  bool check_action_completed_(); 


  /**
   * @brief Checks preconditions for the entire mission to start. This includes:
   *  - information about the Anafi's state (! "")
   *  - information about the battery charge (>= 0)
   *  - ned position properly initialized (to roughly 0s). The Olympe bridge can 
   *    produce ned-positions of several 1000s if initialized too early 
   *       
   */
  bool check_controller_preconditions_();

  /**
   * @brief Output data to the terminal, such that the user is informed
   */
  void print_action_feedback_();
  void print_action_error_();


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg);
  void gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg);
  void attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg);
  void polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);
  void battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg);

}; // MissionControllerNode


