#include "automated_planning/mission_controller.hpp"


void MissionControllerNode::init()
{
  // Verify that the system is set correctly
  // check_controller_preconditions_();

  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>();

  init_knowledge_(); 
  init_mission_goals_();
}


void MissionControllerNode::init_knowledge_()
{
  problem_expert_->clearKnowledge();

  // Assuming the node is run in its own terminal, such that cout << "\n" does not fuck
  // with other data
  std::cout << "\n\n";
  const std::string drone_name = this->get_parameter("drone.name").as_string();
  const std::vector<std::string> locations = this->get_parameter("locations.names").as_string_array();

  problem_expert_->addInstance(plansys2::Instance{drone_name, "drone"});

  // Locations must be added separately from the paths
  // Not possible to combine into one for-loop
  for(std::string loc_str : locations)
  {
    problem_expert_->addInstance(plansys2::Instance{loc_str, "location"});
  }
  for(std::string loc_str : locations)
  {
    const std::vector<std::string> paths_from_loc = this->get_parameter("locations.paths." + loc_str).as_string_array();
    for(std::string next_loc : paths_from_loc)
    { 
      std::string predicate_str = "(path " + loc_str + " " + next_loc + ")";
      RCLCPP_INFO(this->get_logger(), "Adding path predicate: " + predicate_str);
      problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
    }
  }
  std::cout << "\n";

  const std::string drone_pos = this->get_parameter("mission_init.start_location").as_string();
  std::string predicate_str = "(drone_at " + drone_name + " " + drone_pos + ")";
  RCLCPP_INFO(this->get_logger(), "Adding position predicate: " + predicate_str);
  problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
  problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
  std::cout << "\n\n";
}


void MissionControllerNode::step()
{
  // May consider to update these online, or when a transition occurs
  print_action_feedback_();

  if(! check_plan_completed_())
  {
    // TODO: When a plan has finished executing
  }

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
    // remove_predicates_();

    // Removing all goals, as these are set in switch
    problem_expert_->clearGoal();

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
      // Publish planned actions
      // plan_publisher_->publish(plan.value());

      // Log plan
      std::vector<plansys2_msgs::msg::PlanItem> plans = plan.value().items;
      std::string log_str = "\nDetailed plan found: [time] [action] [duration]\n";
      for(plansys2_msgs::msg::PlanItem& plan_item : plans)
      {
        log_str += std::to_string(plan_item.time) + "\t" + plan_item.action + "\t" + std::to_string(plan_item.duration) + "\n";
      }
      RCLCPP_INFO(this->get_logger(), log_str);

      // Start execution
      executor_client_->start_plan_execution(plan.value());
    }
    else
    {
      // Do some error handling here
    }
    is_replanning_necessary_ = false; // Unsure whether this should be set if an error occurs during replanning
  } 
}


void MissionControllerNode::case_normal_operation_()
{
  // Use information about mission goals to



  // TODO: Use remaining information about mission goals to create a new set of 
  // mission goals / predicates

  // Currently assuming the drone is just going to move to h2
  // problem_expert_->setGoal(plansys2::Goal("(and(drone_at d h1))")); // The planner fails if goal-state is equal to state during init

  problem_expert_->setGoal(plansys2::Goal("(and(drone_at d1 h2))")); 

  // Find a method for adding numeric fuel into the predicates
  //problem_expert_->updateFunction(plansys2::Function()) 
}


void MissionControllerNode::case_person_detected_()
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

  RCLCPP_INFO(this->get_logger(), "Person detected. Decided action goals:\n" + goals_str);
  // std::cout << "Person detected. Decided action goals:\n" << goals_str << std::flush;

  // Store the position for future, such that it will not account for the same position twice
  previously_detected_people_.push_back(position);
}


void MissionControllerNode::case_emergency_()
{
  // Find a landing position with the lowest cost
  

}


void MissionControllerNode::case_area_unavailable_()
{
  // Must ensure that the drone keeps away from an area
  // Unsure how this will occur as of now
}


bool MissionControllerNode::check_plan_completed_()
{
  // OBS! This will only check whether the entire plan is finished!!!!
  // Not the action!
  if (! executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
  {
    if (executor_client_->getResult().value().success) 
    {
      // Must detect when a goal is completed, such that it is not done again

      RCLCPP_INFO(this->get_logger(), "Finished action");
      return true;
    } 
    else 
    {
      print_action_error_();
    }
  }

  return false;
}


bool MissionControllerNode::replan_mission_(std::optional<plansys2_msgs::msg::Plan>& plan)
{
  RCLCPP_WARN(this->get_logger(), "Cancelling plan execution");
  executor_client_->cancel_plan_execution();

  // Compute the plan
  RCLCPP_WARN(this->get_logger(), "Replanning");
  std::string domain = domain_expert_->getDomain();
  std::string problem = problem_expert_->getProblem();
  plan = planner_client_->getPlan(domain, problem); 

  if (! plan.has_value()) 
  {
    std::string error_str = "Could not find plan to reach goal: " +
      parser::pddl::toString(problem_expert_->getGoal()) + "\n";
    RCLCPP_ERROR(this->get_logger(), error_str);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "New plan found!");
  return true;
}


bool MissionControllerNode::remove_predicates_()
{
  // The remove predicates will only remove existing predicates
  // It will return false if it does not exist.

  // TODO: Use the output from problem_expert->remobePredicate(...)
  // TODO: Find a better method instead of having 4 for-loops
  // for(plansys2::Predicate& predicate : normal_operation_predicates_)
  // {
  //   problem_expert_->removePredicate(predicate);
  // }
  // for(plansys2::Predicate& predicate : person_detected_predicates_)
  // {
  //   problem_expert_->removePredicate(predicate);
  // }
  // for(plansys2::Predicate& predicate : emergency_predicates_)
  // {
  //   problem_expert_->removePredicate(predicate);
  // }
  // for(plansys2::Predicate& predicate : area_unavailable_predicates_)
  // {
  //   problem_expert_->removePredicate(predicate);
  // }
  return true;
}


void MissionControllerNode::print_action_feedback_()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) 
  {
    // std::cout faster than RCLCPP_INFO
    std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]" << std::endl;
  }
}


void MissionControllerNode::print_action_error_()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) 
  {
    if (action_feedback.status == plansys2_msgs::msg::ActionExecutionInfo::FAILED) 
    {
      std::string error_str = "[" + action_feedback.action + "] finished with error: " +
        action_feedback.message_status + "\n";
      RCLCPP_ERROR(this->get_logger(), error_str);
    }
  }
}



const std::tuple<ControllerState, bool> MissionControllerNode::recommend_replan_()
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

  // Hardcoded to ensure that the system will keep in NORMAL_OPERATION during initial development
  desired_controller_state = ControllerState::NORMAL_OPERATION;
  return std::make_tuple(desired_controller_state, is_replanning_necessary_);
}



void MissionControllerNode::check_controller_preconditions_()
{
  rclcpp::Rate rate(1);

  bool valid_anafi_state = false;
  bool valid_battery = false;
  bool valid_ned_pos = false;

  while (rclcpp::ok()) 
  {
    if(! anafi_state_.empty())
    {
      valid_anafi_state = true;
    }
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "No state update received");
    }

    if(battery_charge_ >= 0 && battery_charge_ <= 100)
    {
      valid_battery = true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid battery: %f ", battery_charge_);
    }

    double position_ned_norm = std::sqrt(
      std::pow(position_ned_.point.x, 2) + std::pow(position_ned_.point.y, 2) + std::pow(position_ned_.point.z, 2)
    ); 
    const double max_initial_ned_norm = 5;

    // Should also check that it is roughly zero in all states 
    if(position_ned_norm <= max_initial_ned_norm)
    {
      valid_ned_pos = true;
    }
    else 
    {
      RCLCPP_ERROR(this->get_logger(), "NED-position likely incorrect. Restart Olympe-bridge...");
    }

    if(valid_anafi_state && valid_battery && valid_ned_pos)
    {
      RCLCPP_INFO(this->get_logger(), "Preconditions checked!");
      break;
    }

    rate.sleep();
    rclcpp::spin_some(this->get_node_base_interface());
  }
}


void MissionControllerNode::init_mission_predicates_()
{

}


void MissionControllerNode::init_mission_goals_()
{
  // Read the goals from the config file, and set the initial goals
  std::string mission_goal_prefix = "mission_goals.";
  bool drone_landed = this->get_parameter(mission_goal_prefix + "drone_landed").as_bool();
  std::string preferred_landing_location = this->get_parameter(mission_goal_prefix + "preferred_landing_location").as_string();
  std::vector<std::string> locations_to_search = this->get_parameter(mission_goal_prefix + "locations_to_search").as_string_array();
  std::vector<std::string> possible_landing_locations = this->get_parameter(mission_goal_prefix + "possible_landing_locations").as_string_array();

  mission_goals_ = MissionGoals(drone_landed, preferred_landing_location, possible_landing_locations, locations_to_search);
}


void MissionControllerNode::update_mission_predicates_()
{
  
}


void MissionControllerNode::update_mission_goals_()
{

}


std::string MissionControllerNode::get_current_location_()
{
  std::string locations_prefix = "locations.";
  std::vector<std::string> locations = this->get_parameter(locations_prefix + "names").as_string_array();
  double location_radius = this->get_parameter(locations_prefix + "location_radius_m").as_double(); 

  std::string closest_loc = "";
  double min_distance = location_radius; // This allows for checking within radius at the same time
  for(std::string loc : locations)
  {
    std::vector<double> locations_positions = this->get_parameter(locations_prefix + "pos_ne." + loc).as_double_array();

    double x_diff = locations_positions[0] - position_ned_.point.x;
    double y_diff = locations_positions[1] - position_ned_.point.y;
    double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

    if(distance <= min_distance)
    {
      min_distance = distance;
      closest_loc = loc;
    }
  }
  return closest_loc;
}


void MissionControllerNode::anafi_state_cb_(std_msgs::msg::String::SharedPtr state_msg)
{
  std::string state = state_msg->data;
  if(std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; }) == possible_anafi_states_.end())
  {
    // No state found
    std::cout << "No state found!: " << state << std::endl; 
    return;
  }
  anafi_state_ = state;
}



void MissionControllerNode::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  position_ned_.header.stamp = ned_pos_msg->header.stamp;
  position_ned_.point = ned_pos_msg->point;
}


void MissionControllerNode::gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg)
{
  (void) gnss_data_msg;
}


void MissionControllerNode::attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg)
{
  attitude_.header.stamp = attitude_msg->header.stamp;
  attitude_.quaternion = attitude_msg->quaternion;
}


void MissionControllerNode::polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  polled_vel_.header.stamp = vel_msg->header.stamp;
  polled_vel_.twist = vel_msg->twist;
}


void MissionControllerNode::battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg)
{
  battery_charge_ = battery_msg->data;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionControllerNode>();

  node->init();

  rclcpp::Rate rate(10);
  try
  {
    while (rclcpp::ok()) 
    {
      node->step();
      rclcpp::spin_some(node->get_node_base_interface());
      rate.sleep();
    }
  }
  catch(...){}

  rclcpp::shutdown();

  return 0;
}
