#include "automated_planning/search_action_node.hpp"


LifecycleNodeInterface::CallbackReturn SearchActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Race-conditions though
  bool preconditions_satisfied = check_search_preconditions_();
  if(! preconditions_satisfied)
  {
    finish(false, 0.0, "Unable to search: Prechecks failed!");
    RCLCPP_ERROR(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Getting the location to search
  search_location_ = get_arguments()[1];
  std::map<std::string, geometry_msgs::msg::Point>::iterator it_search_pos = ned_locations_.find(search_location_);
  if(it_search_pos == ned_locations_.end())
  {
    finish(false, 0.0, "Unable to find coordinate of search location: " + search_location_);
    RCLCPP_ERROR(this->get_logger(), "Unable to find coordinate of search location: " + search_location_);
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  search_center_point_ = std::get<1>(*it_search_pos);
  if(! get_search_positions_())
  {
    finish(false, 0.0, "Unable to load search locations");
    RCLCPP_ERROR(this->get_logger(), "Unable to load search points");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  if(! set_velocity_controller_state_(true, "Unable to activate the velocity controller"))
  {
    finish(false, 0.0, "Unable to activate the velocity controller");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  send_feedback(0.0, "Prechecks finished. Cleared to search!");

  // Hacky method of preventing errors with do_work running when the node is 
  // not activated
  node_activated_ = true;
  action_running_ = false;
  search_point_idx_ = 0;
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn SearchActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  node_activated_ = false;

  // If there is a running action, cancel / abort this
  // auto response = move_action_client_->async_cancel_all_goals();
  action_running_ = false;

  if(! set_velocity_controller_state_(false, "Unable to activate the velocity controller"))
  {
    finish(false, 0.0, "Unable to activate the velocity controller");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void SearchActionNode::do_work()
{
  if(! node_activated_)
  {
    return;
  }

  /**
   * Objects to detect: people or helipad
   * Currently assuming that there will only be a single object of interest in the area, and 
   * the search terminates if an object is recently detected
   * For whomever reads this in the future, it is your job to extend this methodology to
   * - multiple objects
   * - specific objects at different locations
   */

  if(check_recent_detection())
  {
    // Detection 
    RCLCPP_INFO(this->get_logger(), "Detection at location " + search_location_);
    finish(true, 1.0, "Detection of object");

    std::string argument = std::get<1>(detections_[search_location_]);
    if(! set_search_action_finished_(argument))
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to set search-action as finished...");
    }

    search_point_idx_ = 0; // Reset for next search
    return;
  }

  if(check_goal_achieved_())
  {
    hover_();
    
    // Counter to ensure that sufficient time at each search position 
    static int counter = 0;
    const int max_count = 4;
    
    counter++;
    if(counter < max_count)
    {
      return;
    }
    counter = 0;

    // Move towards the next search position
    search_point_idx_++;
    if((size_t) search_point_idx_ >= search_points_.size())
    {
      search_point_idx_ = 0;
      RCLCPP_INFO(this->get_logger(), "All positions searched for location " + search_location_);
      finish(true, 1.0, "All positions searched");

      std::string argument = std::get<1>(detections_[search_location_]);
      if(! set_search_action_finished_(argument))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to set search-action as finished...");
      }
      return;
    } 
  }

  send_feedback(((float) search_point_idx_) / ((float) search_points_.size()));
  goal_position_ned_ = search_points_[search_point_idx_];
  pub_desired_ned_position_(goal_position_ned_);

  /** WARNING: Commented out for now! Struggled to find a good method to allow
   * both GNC and the use of internal controllers
   * 
  if(! action_running_)
  {
    // Check that all goals achieved
    if((size_t) search_point_idx_ >= search_points_.size())
    {
      search_point_idx_ = 0;
      RCLCPP_INFO(this->get_logger(), "All positions searched for location " + search_location_);
      finish(true, 1.0, "All positions searched");

      std::string argument = std::get<1>(detections_[search_location_]);
      if(! set_search_action_finished_(argument))
      {
        RCLCPP_ERROR(this->get_logger(), "Unable to set search-action as finished...");
      }
      return;
    } 

    // Counter to ensure that sufficient time at each search position 
    static int counter = 0;
    const int max_count = 4;
    
    counter++;
    if(counter < max_count)
    {
      return;
    }
    counter = 0;

    RCLCPP_INFO(this->get_logger(), "Moving to search position " + std::to_string(search_point_idx_) + " out of " + std::to_string(search_points_.size()));
    send_feedback(((float) search_point_idx_) / ((float) search_points_.size()));

    // Not all goals achieved. Start the next one
    auto send_goal_options = rclcpp_action::Client<anafi_uav_interfaces::action::MoveToNED>::SendGoalOptions();

    send_goal_options.result_callback = [this](auto) 
    {
      // /
      //  * WARNING: If a multithreaded executor is used, this could cause a race condition!
      //  * Should not be a problem for a single-threaded executor! 
      //  /
      action_running_ = false;
    };
    move_goal_.spherical_radius_of_acceptance = radius_of_acceptance_;
    move_goal_.ned_position = search_points_[search_point_idx_];
    search_point_idx_++;

    future_move_goal_handle_ = move_action_client_->async_send_goal(move_goal_, send_goal_options);
    action_running_ = true;
  }
  */
}



bool SearchActionNode::check_search_preconditions_()
{
  bool is_action_server_ready = false;
  int num_waits = 0;
  const int max_waits = 3;

  while(! is_action_server_ready && num_waits <= max_waits)
  {
    RCLCPP_INFO(this->get_logger(), "Waiting for move action server...");
    is_action_server_ready = move_action_client_->wait_for_action_server(std::chrono::seconds(2));

    num_waits++;
  } 
  return is_action_server_ready && num_waits <= max_waits;
}


void SearchActionNode::init_locations_()
{
  geometry_msgs::msg::Point loc_ned_pos;
  loc_ned_pos.z = -5.0; // Hardcoded for now. Might be wise to set in config file

  const std::vector<std::string> locations = this->get_parameter("locations.names").as_string_array();

  for(std::string loc_str : locations)
  {
    std::vector<double> loc_ne = this->get_parameter("locations.pos_ne." + loc_str).as_double_array();

    loc_ned_pos.x = loc_ne[0];
    loc_ned_pos.y = loc_ne[1];

    ned_locations_[loc_str] = loc_ned_pos;
  }
}


bool SearchActionNode::get_search_positions_()
{
  auto request = std::make_shared<anafi_uav_interfaces::srv::GetSearchPositions::Request>();
  request->preferred_search_technique = request->EXPANDING_SQUARE_SEARCH; 

  if(! search_positions_client_->wait_for_service(2s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to acquire search-position service.");
    return false;
  }

  auto result = search_positions_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call search-position service");
    return false;
  }

  if(! result.get()->success)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call search-position service");
    return false;
  }

  search_points_.clear();

  // Acquire the positions
  std_msgs::msg::Float64MultiArray multiarray = result.get()->positions;
  auto dim_0 = multiarray.layout.dim[0];
  auto dim_1 = multiarray.layout.dim[1];

  int num_positions = dim_0.size;
  int num_coordinates_per_point = dim_1.size;

  if(num_coordinates_per_point != 3)
  {
    RCLCPP_ERROR(this->get_logger(), "Wrong dimension acquired from the waypoints-generator. Expected 3, got %i", num_coordinates_per_point);
    return false;
  }

  for(int point_idx = 0; point_idx < num_positions; point_idx += num_coordinates_per_point)
  {
    try
    {
      geometry_msgs::msg::Point point;
      point.x = search_center_point_.x + multiarray.data[point_idx + 0];
      point.y = search_center_point_.y + multiarray.data[point_idx + 1];
      point.z = search_center_point_.z + multiarray.data[point_idx + 2];

      search_points_.push_back(point);
    }
    catch(const std::out_of_range& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Out of range with error-message: " + std::string(e.what()));
      break;
    }
  }

  return true;
}


bool SearchActionNode::check_recent_detection()
{
  std::map<std::string, std::tuple<rclcpp::Time, std::string>>::iterator it = detections_.find(search_location_);
  if(it == detections_.end())
  {
    // No detection at this location
    return false;
  }

  const double max_time_difference_s = 1.0;
  rclcpp::Duration duration = this->get_clock()->now() - std::get<0>(it->second);

  return duration.seconds() <= max_time_difference_s;
}


bool SearchActionNode::set_search_action_finished_(const std::string& argument)
{
  auto request = std::make_shared<anafi_uav_interfaces::srv::SetFinishedAction::Request>();
  
  std_msgs::msg::String action_name_msg = std_msgs::msg::String();
  action_name_msg.data = "search";

  std_msgs::msg::String location_msg = std_msgs::msg::String();
  location_msg.data = search_location_;

  constexpr int num_args = 1; // Hardcoded for now
  
  std::vector<std_msgs::msg::String> argument_msg; 
  
  std_msgs::msg::String arg;
  arg.data = argument;

  argument_msg.push_back(arg);

  request->finished_action_name = action_name_msg;
  request->location = location_msg;
  request->arguments = argument_msg;
  request->num_arguments = num_args;

  if (! finished_action_client_->wait_for_service(1s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Service not available");
    return false;
  }

  auto result = finished_action_client_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to set action as finished!");
    return false;
  }

  return true;
}


bool SearchActionNode::set_velocity_controller_state_(bool controller_state, const std::string& error_str)
{
  enable_velocity_control_client_->wait_for_service(1s);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = controller_state;
  auto result = enable_velocity_control_client_->async_send_request(request);
  
  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->success)
    {
      return true;
    }
  }
  RCLCPP_ERROR(this->get_logger(), error_str);
  return false;
}


bool SearchActionNode::check_goal_achieved_()
{
  Eigen::Vector3d pos_error_ned = get_position_error_ned_();
  double distance = pos_error_ned.norm();
  return distance <= radius_of_acceptance_;
}


void SearchActionNode::hover_()
{
  pub_desired_ned_position_(position_ned_);
}


void SearchActionNode::pub_desired_ned_position_(const geometry_msgs::msg::Point& target_position)
{
  geometry_msgs::msg::PointStamped point_msg = geometry_msgs::msg::PointStamped();
  point_msg.header.stamp = this->get_clock()->now();
  point_msg.point = target_position;

  goal_position_pub_->publish(point_msg);
}


Eigen::Vector3d SearchActionNode::get_position_error_ned_()
{
  double x_diff = position_ned_.x - goal_position_ned_.x;
  double y_diff = position_ned_.y - goal_position_ned_.y;
  double z_diff = position_ned_.z - goal_position_ned_.z;

  Eigen::Vector3d error_ned;
  error_ned << x_diff, y_diff, z_diff;
  return error_ned;
}


void SearchActionNode::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  position_ned_ = ned_pos_msg->point;
}


void SearchActionNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr)
{
  rclcpp::Time time = this->get_clock()->now();
  detections_[search_location_] = std::make_tuple(time, "person"); 
}


void SearchActionNode::apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr)
{
  rclcpp::Time time = this->get_clock()->now();
  detections_[search_location_] = std::make_tuple(time, "helipad"); 
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "search"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
