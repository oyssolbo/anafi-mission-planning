#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <plansys2_pddl_parser/Utils.h>
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"


class MissionController : public rclcpp::Node
{
public:
  MissionController()
  : rclcpp::Node("mission_controller"), 
    state_(StateType::IDLE)  
  {   
    init_();
  }

  /**
   * @brief Steps through the problem, depending on the state
   */
  void step();

private:
  enum class StateType {IDLE, TAKEOFF, MOVE, LAND};
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  /**
   * @brief Implementations of actions 
   */
  void action_idle_();
  void action_takeoff_();
  void action_land_();
  void action_move_();


  /**
   * @brief Initializes the domain expert, problem expert, planner and executor 
   */
  void init_();

  /**
   * @brief Initializes the world knowledge 
   */
  void init_knowledge_();

  /**
   * @brief Initialize and maintain the mission objectives
   */
  void init_mission_objectives_();
  void maintain_mission_objectives_();

  /**
   * @brief Keeps an account of major objectives
   */
  bool objectives_completed_() const { return false; }

  std::vector<plansys2::Goal> primary_mission_objectives_;
  std::vector<plansys2::Goal> secondary_mission_objectives_;
};


