#include "automated_planning/mission_controller.hpp"


void MissionControllerNode::init()
{
  // Verify that the system is set correctly
  check_controller_preconditions_();

  domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
  planner_client_ = std::make_shared<plansys2::PlannerClient>();
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  executor_client_ = std::make_shared<plansys2::ExecutorClient>();

  init_knowledge_(); 
  init_mission_goals_();
  update_plansys2_functions_();

  publish_plan_status_str_("Starting");
}


void MissionControllerNode::step()
{
  // Check if the plan is finished - current implementation will fail for first iteration
  if(check_plan_completed_()) // This is automatically true if no plan exist
  {
    // RCLCPP_INFO(this->get_logger(), "Mission plan completed. Idling...");
    // controller_state_ = ControllerState::IDLE;
  }

  // // Possible to iterate over the current goals, and check if these are completed
  // std::vector<plansys2::Goal> current_goals = get_goals_(controller_state_);
  // if(problem_expert_->isGoalSatisfied(goal)) ...

  const std::tuple<ControllerState, bool> recommendation = recommend_replan_();
  ControllerState recommended_next_state = std::get<0>(recommendation);
  bool recommended_to_replan = std::get<1>(recommendation);

  if(recommended_to_replan)
  {
    log_replanning_state_();

    // May want to turn this into a while-loop in the future, where the goals are 
    // relaxed until a solution is found
    if(! update_plansys2_goals_(recommended_next_state) || ! update_plansys2_functions_())
    {
      // Failed
      // Do something
    }

    // Note that the planner will fail if the original state is equal to the current
    // state. As such, one might require a method for identifying that this is the 
    // case in this situation
    std::optional<plansys2_msgs::msg::Plan> plan;
    bool replan_success = replan_mission_(plan);
    if(replan_success)
    {
      log_plan_(plan);

      // Start execution
      executor_client_->start_plan_execution(plan.value());
      controller_state_ = recommended_next_state;
    }
    else 
    {
      // Unable to find a suitable plan
      // May need to relax the plan somehow
    }
  }

  // Update the user about action performance
  // print_action_feedback_(); // This is spamming!
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

    if(battery_charge_ > 0 && battery_charge_ <= 100)
    {
      // Assumes the battery charge must be positive to start executing
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


void MissionControllerNode::init_knowledge_()
{
  // Clearing all data simplest for a small problem
  // For a larger problem, one might consider storing predicates and only removing the ones necessary 
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
  prev_location_ = drone_pos;
  std::string predicate_str = "(drone_at " + drone_name + " " + drone_pos + ")";
  RCLCPP_INFO(this->get_logger(), "Adding position predicate: " + predicate_str);
  problem_expert_->addPredicate(plansys2::Predicate(predicate_str));
  std::cout << "\n";

  std::string landed_str;
  if(anafi_state_.compare("FS_LANDED") == 0) // Preconditions already checked that string not empty
  {
    landed_str = "(landed " + drone_name + ")";
  }
  else 
  {
    landed_str = "(not_landed " + drone_name + ")";
  }
  RCLCPP_INFO(this->get_logger(), "Adding landed predicate: " + landed_str);
  problem_expert_->addPredicate(plansys2::Predicate(landed_str));
  std::cout << "\n\n";
}


bool MissionControllerNode::update_plansys2_functions_()
{
  // Delete previous function values
  std::vector<plansys2::Function> existing_functions = problem_expert_->getFunctions();
  for(plansys2::Function& function : existing_functions)
  {
    problem_expert_->removeFunction(function);
  }

  // Update values and insert new functions
  switch (controller_state_)
  {
    case ControllerState::INIT:
    {
      std::string payload_prefix = "mission_init.payload.";
      num_markers_ = this->get_parameter(payload_prefix + "num_markers").as_int();
      num_lifevests_ = this->get_parameter(payload_prefix + "num_lifevests").as_int();
      break; 
    }
    default:
    {
      break;
    }
  } 

  std::string drone_name = this->get_parameter("drone.name").as_string();

  plansys2::Function markers_function = plansys2::Function("(= (num_markers " + drone_name + ") " + std::to_string(num_markers_) +")");
  plansys2::Function lifevests_function = plansys2::Function("(= (num_lifevests " + drone_name + ")" + std::to_string(num_lifevests_) + ")");
  plansys2::Function battery_charge_function = plansys2::Function("(= (battery_charge " + drone_name + ")" + std::to_string(battery_charge_) + ")");

  /**
   * @todo Add functions for fuel usage and drone velocity with respect to action  
   */

  problem_expert_->addFunction(markers_function);
  problem_expert_->addFunction(lifevests_function);
  problem_expert_->addFunction(battery_charge_function);

  return true;
}


bool MissionControllerNode::update_plansys2_goals_(const ControllerState& state)
{
  std::vector<plansys2::Goal> goals;
  
  switch (state)
  {
    case ControllerState::SEARCH:
    {
      load_move_mission_goals_(goals);
      // load_search_mission_goals_(goals); // Temporally commented out for testing 
      break;
    }
    case ControllerState::RESCUE:
    {
      load_rescue_mission_goals_(goals);
      break;
    }
    case ControllerState::EMERGENCY:
    {
      load_emergency_mission_goals_(goals);
      break;
    }
    case ControllerState::AREA_UNAVAILABLE:
    {
      load_area_unavailable_mission_goals_(goals);
      break;
    }
    case ControllerState::IDLE:
    {
      break;
    }
    default:
    {
      RCLCPP_ERROR(this->get_logger(), "No goals loaded...");
      break;
    }
  }

  for(plansys2::Goal goal : goals)
  {
    problem_expert_->setGoal(goal);
  }

  return true;
}


bool MissionControllerNode::load_move_mission_goals_(std::vector<plansys2::Goal>& goals)
{
  // Assuming that the preferred landing-position could be used for this
  // Possible to iterate using the possible landing positions

  // But could also be solved using a goal that the drone must land, and with predicates allowing 
  // all available landing positions to be used. That will be a better solution!
  const std::string drone_name = this->get_parameter("drone.name").as_string();
  std::string desired_pos_str = "(and(drone_at " + drone_name + " a2))"; //"(and(drone_at " + drone_name + " " + mission_goals_.preferred_landing_location_ + "))";
  RCLCPP_INFO(this->get_logger(), "Desired position goal: " + desired_pos_str);
  goals.push_back(plansys2::Goal(desired_pos_str));
  return true;
}


bool MissionControllerNode::load_search_mission_goals_(std::vector<plansys2::Goal>& goals)
{
  const std::string drone_name = this->get_parameter("drone.name").as_string();
  std::vector<std::string> locations_to_search = mission_goals_.locations_to_search_;
  std::string search_position_predicates = "\n";
  for(std::string search_loc : locations_to_search)
  {
    std::string search_loc_str = "(searched " + search_loc + ")";
    goals.push_back(plansys2::Goal(search_loc_str));
    search_position_predicates += search_loc_str + "\n";
  }
  RCLCPP_INFO(this->get_logger(), "Search-predicates: " + search_position_predicates);

  // Currently assuming the 
  std::string desired_pos_str = "(drone_at " + drone_name + " " + mission_goals_.preferred_landing_location_ + ")";
  goals.push_back(plansys2::Goal(desired_pos_str));
  return true;
}


bool MissionControllerNode::load_rescue_mission_goals_(std::vector<plansys2::Goal>& goals)
{
  /**
   * TODO: 
   *  - take the severity into account when detecting people
   *  - have an id / counter to separate the people, allowing for multiple people in the planning domain 
   */

  // Person detected
  std::string person = "p";
  std::string loc = get_current_location_();
  if(loc.empty())
  {
    // If a person is detected between two locations, this is currently not 
    // supported by the planner
    // Must find a better approach to solve this exact problem
    RCLCPP_ERROR(this->get_logger(), "Support for rescuing people between specified locations are currently not supported by the planner...\
    RIP them... Hope they weren't close to you!");
    return false;
  }
  std::string rescue_str = std::string("(rescued ") + person + " " +  loc + std::string(")");
  goals.push_back(plansys2::Goal(rescue_str));
  return true;

  // geometry_msgs::msg::Point position = std::get<0>(detected_person_);
  // Severity severity = std::get<1>(detected_person_);

  // std::string goals_str;

  // switch (severity)
  // {
  //   case Severity::MINOR:
  //   {
  //     // Assuming the victim has a nearby boat, taking care of the victim
      
  //     // Report about the detection and move on
  //     goals_str = "Report position";

  //     // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
  //     break;
  //   }
  //   case Severity::MODERATE:
  //   {
  //     // Assuming that the person has something to cling onto, such as a life vest 

  //     // Report about the position
  //     // Drop a marker to make it easier for the rescue team to detect them
  //     goals_str = "Report position \nDrop marker\n";

  //     // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
  //     // problem_expert_->setGoal(plansys2::Goal("(and(drop marker))"));
  //     break;
  //   }
  //   case Severity::HIGH:
  //   {
  //     // Assuming the victim is floating alone at sea, and has nothing to cling onto

  //     // Report about the position
  //     // Drop marker
  //     // Drop lifevest 
  //     goals_str = "Report position \nDrop marker \nDrop lifevest\n";

  //     // problem_expert_->setGoal(plansys2::Goal("(and(report position))"));
  //     // problem_expert_->setGoal(plansys2::Goal("(and(drop marker))"));
  //     // problem_expert_->setGoal(plansys2::Goal("(and(drop lifevest))"));
  //     break;
  //   }
  //   default:
  //   {
  //     break;
  //   }
  // }

  // RCLCPP_INFO(this->get_logger(), "Person detected. Decided action goals:\n" + goals_str);
  // // std::cout << "Person detected. Decided action goals:\n" << goals_str << std::flush;

  // // Store the position for future, such that it will not account for the same position twice
  // previously_detected_people_.push_back(position);
}


bool MissionControllerNode::load_emergency_mission_goals_(std::vector<plansys2::Goal>& goals)
{
  // Find a landing position with the lowest cost

  // Currently just return the drone to the desired landing position, even though there will be 
  // other positions available
  load_move_mission_goals_(goals);
  return true;
}


bool MissionControllerNode::load_area_unavailable_mission_goals_(std::vector<plansys2::Goal>& goals)
{
  // Must ensure that the drone keeps away from an area
  // Unsure how this will occur as of now
  goals.clear();
  return true;
}


const std::tuple<ControllerState, bool> MissionControllerNode::recommend_replan_()
{
  bool recommend_replan = false;
  ControllerState desired_controller_state;

  std::string reason_to_replan;

  if(is_person_detected_)
  {
    recommend_replan = true;
    desired_controller_state = ControllerState::RESCUE;
    reason_to_replan = "Person detected!";
  }
  else if(is_emergency_)
  {
    recommend_replan = true;
    desired_controller_state = ControllerState::EMERGENCY;
    reason_to_replan = "Emergency occured!";
  }
  else
  {
    switch (controller_state_)
    {
      case ControllerState::INIT:
      case ControllerState::AREA_UNAVAILABLE:
      {
        recommend_replan = true;
        desired_controller_state = ControllerState::SEARCH;
        reason_to_replan = "INIT or area unavailable";
        break;
      }
      case ControllerState::IDLE:
      case ControllerState::SEARCH:
      default:
      {
        // Continue as before
        break;
      }
    }
  }
  if(recommend_replan)
  {
    RCLCPP_INFO(this->get_logger(), "Reason to replan: " + reason_to_replan);
  }

  return std::make_tuple(desired_controller_state, recommend_replan);
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


bool MissionControllerNode::check_plan_completed_()
{
  if (! executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
  {
    if (executor_client_->getResult().value().success) 
    {
      RCLCPP_INFO(this->get_logger(), "Finished plan");
      return true;
    } 
    else 
    {
      log_action_error_();
    }
  }

  return false;
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


void MissionControllerNode::log_replanning_state_()
{
  std::stringstream ss;
  ss << std::boolalpha;
  ss << "\n\n";
  ss << "Current system state:\n";
  ss << "Person detected: " << is_person_detected_ << "\n";
  ss << "Emergency occured: "  << is_emergency_ << "\n";
  ss << "Low battery: " << is_low_battery_ << "\n";
  
  ss << "\n";
  ss << "NED-position: {" << position_ned_.point.x << " " << position_ned_.point.y << " " << position_ned_.point.z << "}\n";
  ss << "BODY-velocity: {" << polled_vel_.twist.linear.x << " " << polled_vel_.twist.linear.y << " " << polled_vel_.twist.linear.z << "}\n";
  ss << "Anafi-state: " << anafi_state_ << "\n";
  ss << "Battery percentage: " << battery_charge_ << "\n";
  ss << "Num markers: " << num_markers_ << "\n";
  ss << "Num lifevests: " << num_lifevests_ << "\n"; 

  ss << "\n";
  ss << "Current plan: \n\n" << previous_plan_str_ << "\n\n";

  std::string log_str = ss.str();
  RCLCPP_INFO(this->get_logger(), log_str);
}


void MissionControllerNode::log_plan_(const std::optional<plansys2_msgs::msg::Plan>& plan)
{
  std::vector<plansys2_msgs::msg::PlanItem> plans = plan.value().items;
  std::string plan_str = "";
  for(plansys2_msgs::msg::PlanItem& plan_item : plans)
  {
    plan_str += std::to_string(plan_item.time) + "\t" + plan_item.action + "\t" + std::to_string(plan_item.duration) + "\n";
  }
  std::string log_str = "\nDetailed plan found: [time] [action] [duration]\n" + plan_str;
  RCLCPP_INFO(this->get_logger(), log_str);

  publish_plan_status_str_(plan_str);
  publish_plansys2_plan_(plan);
}


void MissionControllerNode::log_action_error_()
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


void MissionControllerNode::print_action_feedback_()
{
  auto feedback = executor_client_->getFeedBack();
  for (const auto & action_feedback : feedback.action_execution_status) 
  {
    // std::cout faster than RCLCPP_INFO
    std::cout << "[" << action_feedback.action << " " << action_feedback.completion * 100.0 << "%]" << std::endl;
  }
}


void MissionControllerNode::publish_plan_status_str_(const std::string& str)
{
  // anafi_uav_interfaces::msg::StampedString msg;
  // msg.header.stamp = this->get_clock()->now();
  std_msgs::msg::String msg;
  msg.data = str;
  planning_status_pub_->publish(msg);
}


void MissionControllerNode::publish_plansys2_plan_(const std::optional<plansys2_msgs::msg::Plan>& plan)
{
  plansys2_msgs::msg::Plan plan_msg = plan.value();
  plan_pub_->publish(plan_msg);
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


void MissionControllerNode::battery_charge_cb_(std_msgs::msg::UInt8::ConstSharedPtr battery_msg)
{
  battery_charge_ = battery_msg->data;
}


void MissionControllerNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg)
{
  /**
   * TODO:
   *  - find a better method to store information about detected people and the severity
   *  - determining a method for people not at a predetermined location
   *    - one will risk finding people between a pair of locations  
   */

  geometry_msgs::msg::Point position = detected_person_msg->position;
  uint8_t severity = detected_person_msg->severity;
  (void) severity;

  const double allowed_distance_to_previous_detected = 2.0; // Discuss this with Simen

  auto it = std::find_if(
    previously_detected_people_.begin(), 
    previously_detected_people_.end(), 
    [position, allowed_distance_to_previous_detected](geometry_msgs::msg::Point pt){ 
      return (std::sqrt(std::pow((pt.x - position.x), 2) + std::pow((pt.y - position.y), 2)) <= allowed_distance_to_previous_detected);  
    }
  );

  if(it != previously_detected_people_.end())
  {
    // Previously detected
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Information already stored about the detected person...");
    return;
  } 
  
  // Add a person to a list / map / tuple / etc and ensure that it is stored for help
  // controller_state_ = ControllerState::RESCUE;

  // Must remember to set the predicate that the person must be rescued, such that the 
  // planner is aware of the situation
  std::string person = "p";
  std::string loc = get_current_location_();
  if(loc.empty())
  {
    return; // For now
  }
  std::string person_predicative_str = std::string("(person_at ") + person + " " +  loc + std::string(")");
  RCLCPP_INFO(this->get_logger(), "Adding predicative: " + person_predicative_str);
  problem_expert_->addPredicate(plansys2::Predicate(person_predicative_str));
  is_person_detected_ = true;
}


void MissionControllerNode::set_num_markers_srv_cb_(
    const std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Request> request,
    std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Response> response)
{
  num_markers_ = request->num_equipment;

  RCLCPP_INFO(this->get_logger(), "Current equipment:\nMarkers: %f \nLifevests: %f", num_markers_, num_lifevests_);

  response->success = true;
} 


void MissionControllerNode::set_num_lifevests_srv_cb_(
    const std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Request> request,
    std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Response> response)
{
  num_lifevests_ = request->num_equipment;

  RCLCPP_INFO(this->get_logger(), "Current equipment:\nMarkers: %f \nLifevests: %f", num_markers_, num_lifevests_);

  response->success = true;
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
