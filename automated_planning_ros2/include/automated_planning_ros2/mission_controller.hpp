#pragma once

#include <memory>
#include <tuple>
#include <vector>
#include <tuple>
#include <string>
#include <optional>

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

#include "geometry_msgs/msg/point.hpp"

// TODO: Add messages and subscriptions to follow on the real world 


enum class Severity{ MINOR, MODERATE, HIGH };
enum class ControllerState { NORMAL_OPERATION, PERSON_DETECTED, DRONE_EMERGENCY, AREA_UNAVAILABLE };


class MissionController : public rclcpp::Node
{
public:
  MissionController()
  : rclcpp::Node("mission_controller"), 
    state_(ControllerState::NORMAL_OPERATION),
    is_replanning_necessary_(true),
    is_emergency_(false),
    is_low_battery_(false),
    is_person_detected_(false)
  {
    plan_publisher_ = this->create_publisher<plansys2_msgs::msg::Plan>(
      "/mission_controller/plan", 10);
  }


  /**
   * @brief Initializes the domain expert, problem expert, planner and executor 
   */
  void init();

  /**
   * @brief Steps through the problem, depending on the state
   */
  void step();

private:
  // System state 
  ControllerState state_;

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
  std::vector<std::string> inaccessible_areas_;  // In case the drone cannot enter certain areas

  // Publishers
  rclcpp::Publisher<plansys2_msgs::msg::Plan>::SharedPtr plan_publisher_;



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
  const std::tuple<ControllerState, bool> recommend_replan_(); // Currently assuming that a replanning is not necessary
  bool replan_mission_(std::optional<plansys2_msgs::msg::Plan>& plan); 
  bool remove_predicates_();

  /**
   * @brief Keeps an account of major objectives 
   */
  bool objectives_completed_() const { return false; }
  bool check_action_completed_(); 
  void print_action_feedback_();
  void print_action_error_();
};


