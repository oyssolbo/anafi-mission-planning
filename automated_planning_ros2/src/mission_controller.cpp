#include "automated_planning_ros2/mission_controller.hpp"


void MissionController::init()
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
  problem_expert_->addInstance(plansys2::Instance{"h2", "location"});
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
  problem_expert_->addPredicate(plansys2::Predicate("(path a4 h2)"));
  problem_expert_->addPredicate(plansys2::Predicate("(path h2 a4)"));
}


void MissionController::step()
{
  print_action_feedback_();

  // TODO: Must have a method for detecting that the action has finished,
  // and set the desired state back to ControllerState::NORMAL_OPERATION
  // As of now, there is no way for the controller to return to NORMAL_OPERATION
  // if an error occurs or people are detected...

  const std::tuple<ControllerState, bool> recommendation = recommend_replan_();
  if(std::get<1>(recommendation))
  {
    // Recommended to replan

    // Removing all predicates first, as it is intended that the case_...
    // will set their necessary predicates
    remove_predicates_();

    // Switch based on the recommended state
    switch (std::get<0>(recommendation)) 
    {
      case ControllerState::NORMAL_OPERATION:
      {
        case_normal_operation_();
        break;
      }
      case ControllerState::PERSON_DETECTED:
      {
        case_person_detected_();
        break;
      }
      case ControllerState::DRONE_EMERGENCY:
      {
        case_emergency_();
        break;
      }
      case ControllerState::AREA_UNAVAILABLE:
      {
        case_area_unavailable_();
        break;
      }
      default:
      {
        break;
      }
    }

    // Replan
    std::optional<plansys2_msgs::msg::Plan> plan;
    bool replan_sucess = replan_mission_(plan);
    if(replan_sucess)
    {
      executor_client_->start_plan_execution(plan.value());
    }
    else
    {
      // Do some error handling here
    }
    is_replanning_necessary_ = false; // Unsure whether this should be set if an error occurs during replanning
  } 
}


void MissionController::case_normal_operation_()
{
  // TODO: Use remaining information about mission goals to create a new set of 
  // mission goals / predicates

  // Currently assuming the drone is just going to move to h2
  problem_expert_->setGoal(plansys2::Goal("(and(drone_at h2))"));

  // Find a method for adding numeric fuel into the predicates
  //problem_expert_->updateFunction(plansys2::Function()) 
}


void MissionController::case_person_detected_()
{
  geometry_msgs::msg::Point position = std::get<0>(detected_person_);
  Severity severity = std::get<1>(detected_person_);

  std::string goals_str;

  switch (severity)
  {
    case Severity::MINOR:
    {
      // Assuming the victim has a nearby boat, taking care of the victim
      
      // Report about the detection and move on
      goals_str = "Report position";

      // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
      break;
    }
    case Severity::MODERATE:
    {
      // Assuming that the person has something to cling onto, such as a life vest 

      // Report about the position
      // Drop a marker to make it easier for the rescue team to detect them
      goals_str = "Report position \nDrop marker\n";

      // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
      // problem_expert_->setGoal(plansys2::Goal("(and(drop marker))"));
      break;
    }
    case Severity::HIGH:
    {
      // Assuming the victim is floating alone at sea, and has nothing to cling onto

      // Report about the position
      // Drop marker
      // Drop lifevest 
      goals_str = "Report position \nDrop marker \nDrop lifevest\n";

      // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
      // problem_expert_->setGoal(plansys2::Goal("(and(drop marker))"));
      // problem_expert_->setGoal(plansys2::Goal("(and(drop lifevest))"));
      break;
    }
    default:
    {
      break;
    }
  }

  std::cout << "Person detected. Decided action goals:\n" << goals_str << std::flush;

  // Store the position for future, such that it will not account for the same position twice
  previously_detected_people_.push_back(position);
}


void MissionController::case_emergency_()
{
  // Find a landing position with the lowest cost
  

}


void MissionController::case_area_unavailable_()
{
  // Must ensure that the drone keeps away from an area
  // Unsure how this will occur as of now
}


bool MissionController::check_action_completed_()
{
  if (! executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
  {
    if (executor_client_->getResult().value().success) 
    {
      std::cout << "Finished action" << std::endl;
      return true;
    } 
    else 
    {
      print_action_error_();
    }
  }

  return false;
}


bool MissionController::replan_mission_(std::optional<plansys2_msgs::msg::Plan>& plan)
{
  std::cout << "Cancelling plan execution" << std::endl;
  executor_client_->cancel_plan_execution();

  // Compute the plan
  std::cout << "Replanning" << std::endl;
  auto domain = domain_expert_->getDomain();
  auto problem = problem_expert_->getProblem();
  plan = planner_client_->getPlan(domain, problem); 

  if (! plan.has_value()) 
  {
    std::cout << "Could not find plan to reach goal " <<
      parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
    return false;
  }
  std::cout << "New plan found" << std::endl;
  return true;
}


bool MissionController::remove_predicates_()
{
  // The remove predicates will only remove existing predicates
  // It will return false if it does not exist.

  // TODO: Use the output from problem_expert->remobePredicate(...)
  // TODO: Find a better method instead of having 4 for-loops
  for(plansys2::Predicate& predicate : normal_operation_predicates_)
  {
    problem_expert_->removePredicate(predicate);
  }
  for(plansys2::Predicate& predicate : person_detected_predicates_)
  {
    problem_expert_->removePredicate(predicate);
  }
  for(plansys2::Predicate& predicate : emergency_predicates_)
  {
    problem_expert_->removePredicate(predicate);
  }
  for(plansys2::Predicate& predicate : area_unavailable_predicates_)
  {
    problem_expert_->removePredicate(predicate);
  }
  return true;
}


void MissionController::print_action_feedback_()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) 
  {
    std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]";
  }
  std::cout << std::endl;
}


void MissionController::print_action_error_()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) 
  {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) 
    {
      std::cout << "[" << action_feedback.action << "] finished with error: " <<
        action_feedback.message_status << std::endl;
    }
  }
}



const std::tuple<ControllerState, bool> MissionController::recommend_replan_()
{
  ControllerState desired_controller_state;

  // TODO: Look more into this later
  // Must be very careful with respect to the order of operations, as these will have 
  // different importance / severity. Important to perform the most critical action
  if(is_emergency_)
  {
    // Must be careful to only set this once

    // is_replanning_necessary_ = true;
    desired_controller_state = ControllerState::DRONE_EMERGENCY;
  }
  if(is_person_detected_)
  {
    // Person is detected
    // Must be careful with the postion

    // is_replanning_necessary_ = true;
    desired_controller_state = ControllerState::PERSON_DETECTED;
  }

  // Hardcoded to ensure that the system will keep in NORMAL_OPERATION
  desired_controller_state = ControllerState::NORMAL_OPERATION;
  return std::make_tuple(desired_controller_state, is_replanning_necessary_);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionController>();

  node->init();

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) 
  {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
