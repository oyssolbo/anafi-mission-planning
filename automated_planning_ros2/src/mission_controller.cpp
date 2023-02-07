#include "../include/automated_planning_ros2/mission_controller.hpp"


void MissionController::init_()
{
  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>();
  init_knowledge_();
}


void MissionController::init_knowledge_()
{
  problem_expert_->addInstance(plansys2::Instance{"d", "drone"});
  problem_expert_->addInstance(plansys2::Instance{"h1", "location"});
  problem_expert_->addInstance(plansys2::Instance{"a1", "location"});
  problem_expert_->addInstance(plansys2::Instance{"a2", "location"});
  problem_expert_->addInstance(plansys2::Instance{"a3", "location"});
  problem_expert_->addInstance(plansys2::Instance{"a4", "location"});

  problem_expert_->addPredicate(plansys2::Predicate("(drone_at d h1)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path h1 a1)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a1 h1)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a1 a2)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a2 a1)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a1 a3)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a3 a1)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a2 a4)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a4 a2)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a3 a4)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path a4 a3)"));
}


void MissionController::step()
{
  switch (state_) {
    case StateType::IDLE:
    {
      // Goal should be to generate a takeoff
      action_idle_();
      break;
    }
    case StateType::TAKEOFF:
    {
      action_takeoff_();
      break;
    }
    case StateType::LAND:
    {
      action_land_();
      break;
    }
    case StateType::MOVE:
    {
      action_move_();
      break;
    }
    default:
    {
      break;
    }
  }
}


void MissionController::action_idle_()
{
  // Check if the major objectives completed
  if(objectives_completed_())
  {
    // Determine what is to occur here if the objectives are completed
    // If all of the oibjectives are completed, the mission is a success, and 
    // the drone can shutdown
  }

  // Some objectives completed. Set the goals of the system to achieve these
  // Must have some information of relevant goals remaining somewhere

  


  problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

  // Compute the plan
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  auto plan = planner_client_->getPlan(domain, problem);

  if (!plan.has_value()) {
    std::cout << "Could not find plan to reach goal " <<
      parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    break;
  }

  // Execute the plan
  if (executor_client_->start_plan_execution(plan.value())) {
    state_ = PATROL_WP1;
  }
}



    case STARTING:
      {
        // Set the goal for next state
        problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

        // Compute the plan
        auto domain = domain_expert_->getDomain();
        auto problem = problem_expert_->getProblem();
        auto plan = planner_client_->getPlan(domain, problem);

        if (!plan.has_value()) {
          std::cout << "Could not find plan to reach goal " <<
            parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
          break;
        }

        // Execute the plan
        if (executor_client_->start_plan_execution(plan.value())) {
          state_ = PATROL_WP1;
        }
      }
      break;
    case PATROL_WP1:
      {
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;

            // Cleanning up
            problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp1)"));

            // Set the goal for next state
            problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp2))"));

            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value())) {
              state_ = PATROL_WP2;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }

            // Replan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Unsuccessful replan attempt to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            executor_client_->start_plan_execution(plan.value());
          }
        }
      }
      break;
    case PATROL_WP2:
      {
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;

            // Cleanning up
            problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp2)"));

            // Set the goal for next state
            problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp3))"));

            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value())) {
              state_ = PATROL_WP3;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }

            // Replan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Unsuccessful replan attempt to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            executor_client_->start_plan_execution(plan.value());
          }
        }
      }
      break;
    case PATROL_WP3:
      {
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;

            // Cleanning up
            problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp3)"));

            // Set the goal for next state
            problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp4))"));

            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value())) {
              state_ = PATROL_WP4;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }

            // Replan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Unsuccessful replan attempt to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            executor_client_->start_plan_execution(plan.value());
          }
        }
      }
      break;
    case PATROL_WP4:
      {
        auto feedback = executor_client_->getFeedBack();

        for (const auto & action_feedback : feedback.action_execution_status) {
          std::cout << "[" << action_feedback.action << " " <<
            action_feedback.completion * 100.0 << "%]";
        }
        std::cout << std::endl;

        if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) {
          if (executor_client_->getResult().value().success) {
            std::cout << "Successful finished " << std::endl;

            // Cleanning up
            problem_expert_->removePredicate(plansys2::Predicate("(patrolled wp4)"));

            // Set the goal for next state
            problem_expert_->setGoal(plansys2::Goal("(and(patrolled wp1))"));

            // Compute the plan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Could not find plan to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            if (executor_client_->start_plan_execution(plan.value())) {
              // Loop to WP1
              state_ = PATROL_WP1;
            }
          } else {
            for (const auto & action_feedback : feedback.action_execution_status) {
              if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) {
                std::cout << "[" << action_feedback.action << "] finished with error: " <<
                  action_feedback.message_status << std::endl;
              }
            }

            // Replan
            auto domain = domain_expert_->getDomain();
            auto problem = problem_expert_->getProblem();
            auto plan = planner_client_->getPlan(domain, problem);

            if (!plan.has_value()) {
              std::cout << "Unsuccessful replan attempt to reach goal " <<
                parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
              break;
            }

            // Execute the plan
            executor_client_->start_plan_execution(plan.value());
          }
        }
      }
      break;
    default:
      break;
  }
}






int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
