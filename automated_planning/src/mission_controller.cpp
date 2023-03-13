#include "automated_planning/mission_controller.hpp"


void MissionControllerNode::init()
{
  // Verify that the system is set correctly
  check_controller_preconditions_();

  // Sometime these fail to initialize
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
  /**
  * If not person_detected and not emergency, check this prior to switch-case. Possible to 
  * check that all of the vital mission goals are achieved at the same time, such that it
  * can force the planner to finish said missions goals after f.ex. a person has been 
  * detected and rescued at an area. Detecting that a goal has been achieved, some goal might
  * be removed, however one must take great care in only removing the correct goals. Other 
  * removals can cause an ill-defined state
  * 
  * The kind of goals that could be nice to remove:
  *   - goals for searching an area once searched
  *   - goals for communicating, marking or rescuing once done
  * It would require some form of maintaining the current goals 
  */
  if(check_plan_completed_() && controller_state_ != ControllerState::INIT) 
  {
    // if(get_num_remaining_mission_goals_() == 0)
    // {
    //   // Entire mission completed!
    //   // Move the drone back to base, or ensure that the drone is back at base and terminate
    // }
    // else 
    // {
    //   // A sub-mission is finished
    // }
    switch (controller_state_)
    {
      case ControllerState::EMERGENCY:
        // If an emergency occurs, the drone is not allowed to switch to another state!
        // The drone must land on an
        break;
      default:
        std::string mission_completed_str = "Mission (sub)plan completed! Idling...";//. Reporting plan-information before idling: \n" + problem_expert_->getProblem();
        RCLCPP_INFO(this->get_logger(), mission_completed_str);
        controller_state_ = ControllerState::IDLE;
        break;
    }
  }

  const std::tuple<ControllerState, bool> recommendation = recommend_replan_();
  ControllerState recommended_next_state = std::get<0>(recommendation);
  bool recommended_to_replan = std::get<1>(recommendation);

  if(recommended_to_replan)
  {
    // Important to save active goals before clearing!
    // save_remaining_mission_goals_(); // Note that this does not work atm! Need to find a method for detecting goals
    problem_expert_->clearGoal(); // Clears all goals!

    // May want to turn this into a while-loop in the future, where the goals are 
    // relaxed until a solution is found
    if(! update_plansys2_goals_(recommended_next_state) || ! update_plansys2_functions_())
    {
      // Failed
      // Do something
      RCLCPP_ERROR(this->get_logger(), "Failed to update either goals or functions...");
    }

    // Log plan after new goals have been set! 
    log_planning_state_();

    // Check whether or not the state of the drone satisfies the current goals

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
      // For now just throw an error to prevent spam
      std::runtime_error("Could not find a suitable plan");
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
  std::string mission_goal_prefix = "mission_goals.";

  // Land drone or not
  const std::string drone_name = this->get_parameter("drone.name").as_string();
  bool landing_desired = this->get_parameter(mission_goal_prefix + "landing_desired").as_bool();
  std::string landing_desired_str;
  if(landing_desired)
  { 
    landing_desired_str = "(landed " + drone_name + ")"; 
  }
  else
  {
    landing_desired_str = "(not_landed " + drone_name + ")"; 
  }
  RCLCPP_INFO(this->get_logger(), "Landed goal: " + landing_desired_str);
  mission_goals_.landed_goal_str_ = landing_desired_str;
  // mission_goals_.landed_goal_ = plansys2::Goal(landing_desired_str);  


  // Locations to search
  std::vector<std::string> locations_to_search = this->get_parameter(mission_goal_prefix + "locations_to_search").as_string_array();
  std::string search_position_goals = "\n";
  for(std::string search_loc : locations_to_search)
  {
    std::string search_loc_str = "(searched " + search_loc + ")";
    mission_goals_.search_goal_strings_.push_back(search_loc_str);
    // mission_goals_.search_goals_.push_back(plansys2::Goal(search_loc_str));
    search_position_goals += search_loc_str + "\n";
  }
  RCLCPP_INFO(this->get_logger(), "Locations to search: " + search_position_goals);


  // Landing locations
  std::string preferred_landing_location = this->get_parameter(mission_goal_prefix + "preferred_landing_location").as_string();
  std::string preferred_landing_location_str = "(drone_at " + drone_name + " " + preferred_landing_location + ")";
  RCLCPP_INFO(this->get_logger(), "Preferred landing location: " + preferred_landing_location_str);
  mission_goals_.preferred_landing_goal_str_ = preferred_landing_location_str;
  // mission_goals_.preferred_landing_goal_ = plansys2::Goal(preferred_landing_location_str);

  std::vector<std::string> possible_landing_locations = this->get_parameter(mission_goal_prefix + "possible_landing_locations").as_string_array();
  std::string possible_landing_location_goals = "\n";
  for(std::string land_loc : possible_landing_locations)
  {
    std::string possible_landing_goal_str = "(drone_at " + drone_name + " " + land_loc + ")";
    mission_goals_.possible_landing_goal_strings_.push_back(possible_landing_goal_str);
    // mission_goals_.possible_landing_goals_.push_back(plansys2::Goal(possible_landing_goal_str));
    possible_landing_location_goals += possible_landing_goal_str + "\n";
  }
  RCLCPP_INFO(this->get_logger(), "Possible landing locations: " + possible_landing_location_goals);

  // mission_goals_ = MissionGoals(landing_desired, preferred_landing_location, possible_landing_locations, locations_to_search);
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
    // Set predicates for paths and distances
    const std::vector<std::string> paths_from_loc = this->get_parameter("locations.paths." + loc_str).as_string_array();
    const std::vector<double> from_location_ne_position = this->get_parameter("locations.pos_ne." + loc_str).as_double_array();

    for(std::string next_loc : paths_from_loc)
    { 
      // Initialize paths
      std::string predicate_str = "(path " + loc_str + " " + next_loc + ")";
      RCLCPP_INFO(this->get_logger(), "Adding path predicate: " + predicate_str);
      problem_expert_->addPredicate(plansys2::Predicate(predicate_str));

      // Initialize distances on said paths
      const std::vector<double> to_location_ne_position = this->get_parameter("locations.pos_ne." + next_loc).as_double_array();
      double north_diff = from_location_ne_position[0] - to_location_ne_position[0];
      double east_diff = from_location_ne_position[1] - to_location_ne_position[1]; 
      double distance = std::sqrt(std::pow(north_diff, 2) + std::pow(east_diff, 2));
      
      std::string distance_str = "(= (distance " + loc_str + " " + next_loc + ") " + std::to_string(distance) + ")";
      RCLCPP_INFO(this->get_logger(), "Adding distance function: " + distance_str);
      problem_expert_->addFunction(plansys2::Function(distance_str));
    }

    // Set search-distance for each location (currently assumed fixed...)
    double search_distance = this->get_parameter("search.distance").as_double();
    std::string search_distance_str = "(= (search_distance " + loc_str + ")" + std::to_string(search_distance) + ")";
    RCLCPP_INFO(this->get_logger(), "Adding distance function: " + search_distance_str);
    problem_expert_->addFunction(plansys2::Function(search_distance_str));

    // Set all locations as not searched, as the drone might have to search a location before landing
    std::string not_searched_loc_str = "(not_searched " + loc_str + ")";
    RCLCPP_INFO(this->get_logger(), "Adding search predicate: " + not_searched_loc_str);
    problem_expert_->addPredicate(plansys2::Predicate(not_searched_loc_str));
  }
  std::cout << "\n";

  const std::string drone_pos = this->get_parameter("mission_init.start_location").as_string(); // May consider to use this->get_location() instead
  std::string predicate_str = "(drone_at " + drone_name + " " + drone_pos + ")";
  RCLCPP_INFO(this->get_logger(), "Adding position predicate: " + predicate_str);
  problem_expert_->addPredicate(plansys2::Predicate(predicate_str));

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

  if(anafi_state_.compare("FS_FLYING") != 0) // Preconditions already checked that string not empty
  {
    // Assuminhg that movement requires the drone state to be FS_FLYING
    std::string moving_str = "(not_moving " + drone_name + ")";
    RCLCPP_INFO(this->get_logger(), "Adding moving predicate: " + moving_str);
    problem_expert_->addPredicate(plansys2::Predicate(moving_str));
  }

  std::vector<std::string> landable_locations = this->get_parameter("locations.landing_available").as_string_array();
  for(std::string land_loc : landable_locations)
  {
    std::string landable_loc_str = "(can_land " + land_loc + ")";
    RCLCPP_INFO(this->get_logger(), "Adding landable location predicate: " + landable_loc_str);
    problem_expert_->addPredicate(plansys2::Predicate(landable_loc_str));

    // std::string not_tracked_landing_location_str = "(not_tracked " + land_loc + ")";
    // RCLCPP_INFO(this->get_logger(), "Adding location tracking predicate: " + not_tracked_landing_location_str);
    // problem_expert_->addPredicate(plansys2::Predicate(not_tracked_landing_location_str));

    // // This is a wierd method attempting to circumwent a problem
    // // For whomever is unfortunate enough to read this, I am sorry
    // std::string trackable_landing_loc = "lz_" + land_loc;
    // problem_expert_->addInstance(plansys2::Instance{trackable_landing_loc, "person"}); 
    // std::string not_tracked_landing_location_str = "(not_tracked " + trackable_landing_loc + ")";
    // std::string lz_at_str = "(person_at " + trackable_landing_loc + " " + land_loc +  ")";
    // RCLCPP_INFO(this->get_logger(), "Adding predicate: " + not_tracked_landing_location_str);
    // problem_expert_->addPredicate(plansys2::Predicate(not_tracked_landing_location_str));
    // RCLCPP_INFO(this->get_logger(), "Adding predicate: " + lz_at_str);
    // problem_expert_->addPredicate(plansys2::Predicate(lz_at_str));
  }

  // The drone is assumed to not search, drop, track, rescue nor mark at the start of the mission
  // All of these are required to be false to be able to move the drone
  // See the PDDL-file
  std::string searching_str = "(not_searching " + drone_name + ")";
  RCLCPP_INFO(this->get_logger(), "Adding searching predicate: " + searching_str);
  problem_expert_->addPredicate(plansys2::Predicate(searching_str));

  std::string tracking_str = "(not_tracking " + drone_name + ")";
  RCLCPP_INFO(this->get_logger(), "Adding tracking predicate: " + tracking_str);
  problem_expert_->addPredicate(plansys2::Predicate(tracking_str));

  std::string rescuing_str = "(not_rescuing " + drone_name + ")";
  RCLCPP_INFO(this->get_logger(), "Adding rescuing predicate: " + rescuing_str);
  problem_expert_->addPredicate(plansys2::Predicate(rescuing_str));

  std::string marking_str = "(not_marking " + drone_name + ")";
  RCLCPP_INFO(this->get_logger(), "Adding marking predicate: " + marking_str);
  problem_expert_->addPredicate(plansys2::Predicate(marking_str));

  // Fixed functional values
  std::string battery_usage_prefix = "drone.battery_usage_per_time_unit.";
  double track_battery_usage = this->get_parameter(battery_usage_prefix + "track").as_double();
  double move_battery_usage = this->get_parameter(battery_usage_prefix + "move").as_double();

  std::string velocity_prefix = "drone.velocity_limits.";
  double track_velocity_limit = this->get_parameter(velocity_prefix + "track").as_double();
  double move_velocity_limit = this->get_parameter(velocity_prefix + "move").as_double();

  std::string track_battery_usage_str = "(= (track_battery_usage " + drone_name + ") " + std::to_string(track_battery_usage) + ")";
  RCLCPP_INFO(this->get_logger(), "Adding battery usage function: " + track_battery_usage_str);
  problem_expert_->addFunction(plansys2::Function(track_battery_usage_str));

  std::string move_battery_usage_str = "(= (move_battery_usage " + drone_name + ") " + std::to_string(move_battery_usage) + ")";
  RCLCPP_INFO(this->get_logger(), "Adding battery usage function: " + move_battery_usage_str);
  problem_expert_->addFunction(plansys2::Function(move_battery_usage_str));

  std::string track_velocity_str = "(= (track_velocity " + drone_name + ") " + std::to_string(track_velocity_limit) + ")";
  RCLCPP_INFO(this->get_logger(), "Adding velocity function: " + track_velocity_str);
  problem_expert_->addFunction(plansys2::Function(track_velocity_str));

  std::string move_velocity_str = "(= (move_velocity " + drone_name + ") " + std::to_string(move_velocity_limit) + ")";
  RCLCPP_INFO(this->get_logger(), "Adding velocity function: " + move_velocity_str);
  problem_expert_->addFunction(plansys2::Function(move_velocity_str));

  std::cout << "\n\n";
}


bool MissionControllerNode::update_plansys2_functions_()
{
  // Delete previous function values
  // std::vector<plansys2::Function> existing_functions = problem_expert_->getFunctions();
  // for(plansys2::Function& function : existing_functions)
  // {
  //   problem_expert_->removeFunction(function);
  // }

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

  problem_expert_->addFunction(markers_function);
  problem_expert_->addFunction(lifevests_function);
  problem_expert_->addFunction(battery_charge_function);

  return true;
}


bool MissionControllerNode::update_plansys2_goals_(const ControllerState& state)
{
  std::vector<std::string> goal_strings;
  
  switch (state)
  {
    case ControllerState::SEARCH:
    {
      load_move_mission_goals_(goal_strings);
      load_search_mission_goals_(goal_strings);
      break;
    }
    case ControllerState::RESCUE:
    {
      load_rescue_mission_goals_(goal_strings);
      break;
    }
    case ControllerState::EMERGENCY:
    {
      load_emergency_mission_goals_(goal_strings);
      break;
    }
    case ControllerState::AREA_UNAVAILABLE:
    {
      load_area_unavailable_mission_goals_(goal_strings);
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

  std::string total_goal_string = "(and";
  for(const std::string& goal_str : goal_strings)
  {
    total_goal_string += goal_str;
    // problem_expert_->setGoal(goal_str);
  }
  total_goal_string += ")";
  RCLCPP_INFO(this->get_logger(), "Setting goal-string as: " + total_goal_string);

  problem_expert_->setGoal(plansys2::Goal(total_goal_string));

  return true;
}


bool MissionControllerNode::load_move_mission_goals_(std::vector<std::string>& goals)
{
  // Assuming that the preferred landing-position could be used for this
  // Possible to iterate using the possible landing positions

  // But could also be solved using a goal that the drone must land, and with predicates allowing 
  // all available landing positions to be used. That will be a better solution!
  // const std::string drone_name = this->get_parameter("drone.name").as_string();
  // // std::string desired_pos_str = "(and(drone_at " + drone_name + " " + mission_goals_.preferred_landing_location_ + "))";
  // // std::string desired_pos_str = "(and(drone_at " + drone_name + " h1))"; //"(and(drone_at " + drone_name + " " + mission_goals_.preferred_landing_location_ + "))";
  // RCLCPP_INFO(this->get_logger(), "Desired position goal: " + desired_pos_str);
  goals.push_back(mission_goals_.landed_goal_str_);
  goals.push_back(mission_goals_.preferred_landing_goal_str_); // plansys2::Goal(mission_goals_.preferred_landing_goal_));

  // This somehow fucks with the planner - going to be interesting to plan for the drone to land then...
  // std::string landing_desired = "(and(not_landed " + drone_name + "))"; 
  // RCLCPP_INFO(this->get_logger(), "Not landed goal: " + landing_desired);
  // goals.push_back(plansys2::Goal(landing_desired));

  // Test of what occurs if multiple of the same goal are added - looks ok
  // goals.push_back(plansys2::Goal(desired_pos_str));
  return true;
}


bool MissionControllerNode::load_search_mission_goals_(std::vector<std::string>& goals)
{
  // for(plansys2::Goal goal : mission_goals_.search_goals_)
  // {
  //   goals.push_back(goal);
  // }

  for(std::string str : mission_goals_.search_goal_strings_)
  {
    goals.push_back(str);
  }
  return true;
}


bool MissionControllerNode::load_rescue_mission_goals_(std::vector<std::string>& goals)
{
  // Using a set to ensure that the rescue goals are unique before planning
  // Testing shows that the planner can handle multiple identical goals
  std::set<std::string> goal_string_set;
  std::vector<int> unhelped_people_ids_ = get_unhelped_people_();

  for(const int& person_id : unhelped_people_ids_)
  {
    std::tuple<geometry_msgs::msg::Point, Severity, bool> person_information = detected_people_[person_id];
    geometry_msgs::msg::Point position = std::get<0>(person_information);
    Severity severity = std::get<1>(person_information);
    bool helped = std::get<2>(person_information);

    // Avoid helping those basterds a second time
    // They can be left to die if they attempt to win the Darwin award! 
    if(helped)
    {
      std::string fatal_string = "Person acquired as not helped is suddenly helped???";
      RCLCPP_FATAL(this->get_logger(), fatal_string);
      throw std::runtime_error(fatal_string);
    }

    std::string person_str = "p" + std::to_string(person_id);
    std::string location_str = get_location_(position);

    if(location_str.empty())
    {
      // Person detected between two locations currently not supported by the planner
      RCLCPP_ERROR(this->get_logger(), "Only a predetermined set of locations is supported in the free rescue version! Order premium rescue support today!");
      continue;
    }

    // What happens if multiple identical goals are loaded into the planner?
    // Using a set to prevent this!

    // Note the lack of breaking, to ensure that a situation with a high priority get marked, rescued (with a life vest) and communicated,
    // while a case with minor severity only gets communicated
    switch (severity)
    {
      case Severity::HIGH:
      {
        std::string rescue_goal_str = "(rescued " + person_str + " " +  location_str + ")";
        goal_string_set.insert(rescue_goal_str);
        [[fallthrough]];
      }
      case Severity::MODERATE:
      {
        std::string marked_goal_str = "(marked " + person_str + " " +  location_str + ")";
        goal_string_set.insert(marked_goal_str);
        [[fallthrough]];
      }
      case Severity::MINOR:
      {
        std::string communicated_goal_str = "(communicated " + person_str + " " +  location_str + ")";
        goal_string_set.insert(communicated_goal_str);
        [[fallthrough]];
      }
      default: 
      {
        break;
      }
    }
  }

  for(std::string str : goal_string_set)
  {
    goals.push_back(str);
  }
  return true;
}


bool MissionControllerNode::load_emergency_mission_goals_(std::vector<std::string>& goals)
{
  // Find a landing position with the lowest cost

  // Currently just return the drone to the desired landing position, even though there will be 
  // other positions available
  load_move_mission_goals_(goals);
  return true;
}


bool MissionControllerNode::load_area_unavailable_mission_goals_(std::vector<std::string>& goals)
{
  // Must ensure that the drone keeps away from an area
  // Unsure how this will occur as of now
  goals.clear();
  return true;
}


bool MissionControllerNode::save_remaining_mission_goals_()
{
  // Could be done using lambda or similar, but 
  // prefer 4 simple for-loops to ensure readability

  // OBS! This is wrong! 

  // std::vector<plansys2::Goal> active_search_goals_;
  // for(const plansys2::Goal& goal : mission_goals_.search_goals_)
  // {
  //   if(! problem_expert_->isGoalSatisfied(goal))
  //   {
  //     active_search_goals_.push_back(goal);
  //   }
  // }
  // mission_goals_.search_goals_ = active_search_goals_;

  // std::vector<plansys2::Goal> active_communicate_goals_;
  // for(const plansys2::Goal& goal : mission_goals_.communicate_location_goals_)
  // {
  //   if(! problem_expert_->isGoalSatisfied(goal))
  //   {
  //     active_communicate_goals_.push_back(goal);
  //   }
  // }
  // mission_goals_.communicate_location_goals_ = active_communicate_goals_;

  // std::vector<plansys2::Goal> active_mark_goals_;
  // for(const plansys2::Goal& goal : mission_goals_.mark_location_goals_)
  // {
  //   if(! problem_expert_->isGoalSatisfied(goal))
  //   {
  //     active_mark_goals_.push_back(goal);
  //   }
  // }
  // mission_goals_.mark_location_goals_ = active_mark_goals_;

  // std::vector<plansys2::Goal> active_rescue_goals_;
  // for(const plansys2::Goal& goal : mission_goals_.rescue_location_goals_)
  // {
  //   if(! problem_expert_->isGoalSatisfied(goal))
  //   {
  //     active_rescue_goals_.push_back(goal);
  //   }
  // }
  // mission_goals_.rescue_location_goals_ = active_rescue_goals_;

  return true;
}


size_t MissionControllerNode::get_num_remaining_mission_goals_()
{
  return mission_goals_.search_goal_strings_.size() + mission_goals_.communicate_location_goal_strings_.size() 
    + mission_goals_.mark_location_goal_strings_.size() + mission_goals_.rescue_location_goal_strings_.size();
}


bool MissionControllerNode::check_desired_final_state_achieved_()
{
  std::string mission_goal_prefix = "mission_goals.";

  bool landing_desired = this->get_parameter(mission_goal_prefix + "landing_desired").as_bool();
  std::string preferred_landing_location = this->get_parameter(mission_goal_prefix + "preferred_landing_location").as_string();
  std::vector<std::string> possible_landing_locations = this->get_parameter(mission_goal_prefix + "possible_landing_locations").as_string_array();
  possible_landing_locations.push_back(preferred_landing_location);
  std::string location = get_location_(position_ned_.point);
  bool achieved_location = std::find(possible_landing_locations.begin(), possible_landing_locations.end(), location) != possible_landing_locations.end();

  int landed_str_comparison = anafi_state_.compare("FS_LANDED");
  int hovering_str_comparison = anafi_state_.compare("FS_HOVERING"); // The drone should hover, if not desired to land
  bool achieved_anafi_state = (landing_desired) ? landed_str_comparison == 0 : hovering_str_comparison == 0;

  return achieved_location && achieved_anafi_state;
}


const std::tuple<ControllerState, bool> MissionControllerNode::recommend_replan_()
{
  bool recommend_replan = false;

  ControllerState desired_controller_state;
  std::string reason_to_replan;

  // Obs! Race condition: A replanning will be triggered even if the state is in emergency,
  // if a new message is received before the ifs is checked. Desirable to only replan once to a certain 
  // condition
  // It assumes that the original emergency is capable of planning for whatever the next emergency 
  // is being triggered by
  // For rescue, one would like to force replanning! This is due to new information about someone detected

  if(is_person_detected_)
  {
    recommend_replan = true;
    desired_controller_state = ControllerState::RESCUE;
    reason_to_replan = "Person detected!";
    is_person_detected_ = false; // Only trigger replan once
  }
  else if(is_emergency_)
  {
    // For future work, this should take the rescue situation and the severity of the emergency into 
    // account. Currently, even if there is an ongoing rescue-operation, a minor emergency will force 
    // a replanning, causing the drone to return to base  
    recommend_replan = true;
    desired_controller_state = ControllerState::EMERGENCY;
    reason_to_replan = "Emergency occured!";
    is_emergency_ = false; // Only trigger replan once
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
      {
        // Check that there are remaining mission goals or whether there are goals
        bool remaining_mission_goals = get_num_remaining_mission_goals_() == 0;
        bool final_state_achieved = check_desired_final_state_achieved_();
        if(remaining_mission_goals || ! final_state_achieved)
        {
          desired_controller_state = ControllerState::SEARCH;
          recommend_replan = true;
          reason_to_replan = "Remaining missions to complete";
          break;
        }

        // If all missions completed, fallthrough and continue idling
        [[fallthrough]];
      }
      case ControllerState::SEARCH:
      default:
      {
        // Continue as before
        desired_controller_state = controller_state_;
        break;
      }
    }
  }

  if(recommend_replan)
  {
    // Ugly code, but hopefully prevents the race conditions triggering replanning
    if(controller_state_ == desired_controller_state)
    {
      switch (controller_state_)
      {
        case ControllerState::EMERGENCY:
          // Should only be triggered by race condition
          recommend_replan = false;
          RCLCPP_WARN(this->get_logger(), "Possible race-condition, with reason to replan: " + reason_to_replan);
          break;
        case ControllerState::RESCUE:
          RCLCPP_INFO(this->get_logger(), "Replanning for new detected person");
          break;
        default:
          // This should never occur!
          // Assumes ! recommend_replan in IDLE, SEARCH, default cases
          throw std::runtime_error("Checking for replanning with the same state as currently!"); 
          break;
      }
    }
    else 
    {
      RCLCPP_INFO(this->get_logger(), "Reason to replan: " + reason_to_replan);
    }
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
  rclcpp::Time start_time = this->get_clock()->now();
  plan = planner_client_->getPlan(domain, problem); 
  rclcpp::Time end_time = this->get_clock()->now();
  rclcpp::Duration duration = end_time - start_time;

  if (! plan.has_value()) 
  {
    std::string error_str = "Could not find plan to reach goal: " +
      parser::pddl::toString(problem_expert_->getGoal()) + "\n";
    RCLCPP_ERROR(this->get_logger(), error_str);
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "New plan found! Plan-duration: %f s", duration.seconds());
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


std::string MissionControllerNode::get_location_(const geometry_msgs::msg::Point& point)
{
  std::string locations_prefix = "locations.";
  std::vector<std::string> locations = this->get_parameter(locations_prefix + "names").as_string_array();
  double location_radius = this->get_parameter(locations_prefix + "location_radius_m").as_double(); 

  std::string closest_loc = "";
  double min_distance = location_radius; // This allows for checking within radius at the same time
  for(std::string loc : locations)
  {
    std::vector<double> locations_positions = this->get_parameter(locations_prefix + "pos_ne." + loc).as_double_array();

    double x_diff = locations_positions[0] - point.x;
    double y_diff = locations_positions[1] - point.y;
    double distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

    if(distance <= min_distance)
    {
      min_distance = distance;
      closest_loc = loc;
    }
  }
  return closest_loc;
}


std::vector<int> MissionControllerNode::get_unhelped_people_()
{
  // Somewhat inefficient to store the results in a vector, but fine for small number of values
  std::vector<int> vec;

  for (auto const& [key, val] : detected_people_)
  {
    bool helped = std::get<2>(val);
    if(! helped)
    {
      vec.push_back(key);
    }
  }
  return vec;
}



std::vector<int> MissionControllerNode::get_people_within_radius_of_(const geometry_msgs::msg::Point& point, double radius)
{
  // Somewhat inefficient to store the results in a vector, but fine for small number of values
  std::vector<int> vec;

  for (auto const& [key, val] : detected_people_)
  {
    geometry_msgs::msg::Point position = std::get<0>(val);
    double distance = std::sqrt(std::pow((position.x - point.x), 2) + std::pow((position.y - point.y), 2));
    if(distance <= radius)
    {
      vec.push_back(key);
    }
  }
  return vec;
}


void MissionControllerNode::log_planning_state_()
{
  std::stringstream ss;
  ss << std::boolalpha;

  ss << "\n\n";
  ss << "Current system state:\n";
  ss << "Person detected: " << is_person_detected_ << "\n";
  ss << "Emergency occured: "  << is_emergency_ << "\n";
  ss << "Low battery: " << is_low_battery_ << "\n";
  
  ss << "\n";
  ss << "Drone location: " << get_location_(position_ned_.point) << "\n";
  ss << "NED-position: {" << position_ned_.point.x << " " << position_ned_.point.y << " " << position_ned_.point.z << "}\n";
  ss << "BODY-velocity: {" << polled_vel_.twist.linear.x << " " << polled_vel_.twist.linear.y << " " << polled_vel_.twist.linear.z << "}\n";
  ss << "Anafi-state: " << anafi_state_ << "\n";
  ss << "Battery percentage: " << battery_charge_ << "\n";
  ss << "Num markers: " << num_markers_ << "\n";
  ss << "Num lifevests: " << num_lifevests_ << "\n"; 
  
  ss << "\n";
  ss << "Possible controller states: INIT (0), SEARCH (1), RESCUE (2), EMERGENCY (3), AREA_UNAVAILABLE (4), IDLE (5)\n";
  ss << "Current controller state: " << int(controller_state_) << "\n";

  ss << "\n";
  ss << "Remaining search goals: \n"; // << mission_goals_.search_goals_.size() << "\n";
  for(std::string str : mission_goals_.search_goal_strings_)
  {
    ss << " " << str; 
  }
  ss << "\n";
  ss << "Remaining communicate goals: \n"; // << mission_goals_.communicate_location_goals_.size() << "\n";
  for(std::string str : mission_goals_.communicate_location_goal_strings_)
  {
    ss << " " << str; 
  }
  ss << "\n";
  ss << "Remaining mark goals: \n"; // << mission_goals_.mark_location_goals_.size() << "\n";
  for(std::string str : mission_goals_.mark_location_goal_strings_)
  {
    ss << " " << str; 
  }
  ss << "\n";
  ss << "Remaining rescue goals: \n"; // << mission_goals_.rescue_location_goals_.size() << "\n";
  for(std::string str : mission_goals_.rescue_location_goal_strings_)
  {
    ss << " " << str; 
  }
  ss << "\n";

  ss << "\n";
  ss << "Planning problem: \n" << problem_expert_->getProblem() + "\n";

  // ss << "\n";
  // ss << "Previous plan: \n\n" << previous_plan_str_ << "\n\n";

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
  std::string log_str = "\n\nDetailed plan found: [time] [action] [duration]\n" + plan_str;
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


void MissionControllerNode::battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg)
{
  battery_charge_ = battery_msg->data;
}


void MissionControllerNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg)
{
  geometry_msgs::msg::Point position = detected_person_msg->position;
  Severity severity = Severity(detected_person_msg->severity);

  // Maximum distance between detections to separate them as different peoople
  // Based on the discussion with Simen, the error will be roughly 0.5 meters. 
  const double radius = 2.5;
  std::vector<int> people_vec = get_people_within_radius_of_(position, radius);

  if(! people_vec.empty())
  {
    // Person previously detected
    // WARNING: This assumes that no accident will occur at the same location!
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Person at position {%f, %f, %f} previously detected", position.x, position.y, position.z);
    return;
  }

  std::string location = get_location_(position_ned_.point);
  if(location.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to determine location! Hope the detected person is not close to you...");
    return; // For now
  }

  is_person_detected_ = true;
  detected_people_[person_idx_] = std::make_tuple(position, severity, false);

  std::string person_id = "p" + std::to_string(person_idx_);
  person_idx_++;
  
  RCLCPP_INFO(this->get_logger(), "Adding instance: " + person_id);
  problem_expert_->addInstance(plansys2::Instance{person_id, "person"});

  std::string person_predicative_str = "(person_at " + person_id + " " +  location + ")";
  RCLCPP_INFO(this->get_logger(), "Adding predicative: " + person_predicative_str);
  problem_expert_->addPredicate(plansys2::Predicate(person_predicative_str));

  // Predicatives that the person is not rescued, not marked and not communicated about
  // Using a switch to ensure that the predicates are set correctly. Notice the lack of breaks, 
  // such that a high emergency should enfore communication, marking and rescuing
  switch (severity)
  {
    case Severity::HIGH:
    {
      std::string not_rescued_predicative_str = "(not_rescued " + person_id + + " " + location + ")";
      RCLCPP_INFO(this->get_logger(), "Adding predicative: " + not_rescued_predicative_str);
      problem_expert_->addPredicate(plansys2::Predicate(not_rescued_predicative_str));
      [[fallthrough]];
    }
    case Severity::MODERATE:
    {
      std::string not_marked_predicative_str = "(not_marked " + person_id + + " " + location + ")";
      RCLCPP_INFO(this->get_logger(), "Adding predicative: " + not_marked_predicative_str);
      problem_expert_->addPredicate(plansys2::Predicate(not_marked_predicative_str));
      [[fallthrough]];
    }
    case Severity::MINOR:
    {
      std::string not_communicated_predicative_str = "(not_communicated " + person_id + + " " + location + ")";
      RCLCPP_INFO(this->get_logger(), "Adding predicative: " + not_communicated_predicative_str);
      problem_expert_->addPredicate(plansys2::Predicate(not_communicated_predicative_str));

      std::string not_tracked_predicate_str = "(not_tracked " + person_id + ")";
      RCLCPP_INFO(this->get_logger(), "Adding predicative: " + not_tracked_predicate_str);
      problem_expert_->addPredicate(plansys2::Predicate(not_tracked_predicate_str));
      break;
    }
    default: 
    {
      break;
    }
  }
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


void MissionControllerNode::set_finished_action_srv_cb_(
  const std::shared_ptr<anafi_uav_interfaces::srv::SetFinishedAction::Request> request,
  std::shared_ptr<anafi_uav_interfaces::srv::SetFinishedAction::Response>)
{
  std_msgs::msg::String action_name_msg = request->finished_action_name;
  std_msgs::msg::String location_msg = request->location;
  int num_arguments = request->num_arguments;
  auto arguments = request->arguments;

  std::string action_name = action_name_msg.data;
  auto it = std::find(key_action_names_.begin(), key_action_names_.end(), action_name);
  if(it == key_action_names_.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Current action-name not found {" + action_name + "} at location {" + location_msg.data + "} with number of arguments " + std::to_string(num_arguments));
    return;
  }

  std::string location = location_msg.data;  
  std::string remove_str;

  // Preferable with a lambda-function
  // std::vector<std::string> vec;

  // auto lambda_remove_str_from_vec = [&vec, &remove_str](auto)
  // { 
  //   auto it = std::find(vec.begin(), vec.end(), remove_str);
  //   if(it != vec.end())
  //   {
  //     vec.erase(it);
  //   }
  //   return vec;
  // };

  if(action_name.compare("search") == 0)
  {
    remove_str = "(not_searched " + location + ")";

    auto it = std::find(mission_goals_.search_goal_strings_.begin(), mission_goals_.search_goal_strings_.end(), remove_str);
    if(it != mission_goals_.search_goal_strings_.end())
    {
      mission_goals_.search_goal_strings_.erase(it);
    }
  }
  else if(action_name.compare("communicate") == 0 && num_arguments >= 1)
  {
    std::string person = arguments[0].data;
    remove_str = "(not_communicated " + person + + " " + location + ")";

    auto it = std::find(mission_goals_.communicate_location_goal_strings_.begin(), mission_goals_.communicate_location_goal_strings_.end(), remove_str);
    if(it != mission_goals_.communicate_location_goal_strings_.end())
    {
      mission_goals_.communicate_location_goal_strings_.erase(it);
    }
  }
  else if(action_name.compare("mark") == 0 && num_arguments >= 1)
  {
    std::string person = arguments[0].data;
    remove_str = "(not_marked " + person + + " " + location + ")";

    auto it = std::find(mission_goals_.mark_location_goal_strings_.begin(), mission_goals_.mark_location_goal_strings_.end(), remove_str);
    if(it != mission_goals_.mark_location_goal_strings_.end())
    {
      mission_goals_.mark_location_goal_strings_.erase(it);
    }
  }
  else if(action_name.compare("rescue") == 0 && num_arguments >= 1)
  {
    std::string person = arguments[0].data;
    remove_str = "(not_rescued " + person + + " " + location + ")";

    auto it = std::find(mission_goals_.rescue_location_goal_strings_.begin(), mission_goals_.rescue_location_goal_strings_.end(), remove_str);
    if(it != mission_goals_.rescue_location_goal_strings_.end())
    {
      mission_goals_.rescue_location_goal_strings_.erase(it);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Received string to remove: " + remove_str);

  // Empty response for SetFinishedAction
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
