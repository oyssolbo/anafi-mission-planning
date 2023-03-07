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
  if(!get_search_positions_())
  {
    finish(false, 0.0, "Unable to load search locations");
    RCLCPP_ERROR(this->get_logger(), "Unable to load search points");
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
  auto response = move_action_client_->async_cancel_all_goals();
  action_running_ = false;

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void SearchActionNode::do_work()
{
  if(! node_activated_)
  {
    return;
  }

  // Check that something has been detected
  // If so, finish the entire location as searched then

  // Things to detect: people or helipad
  // Currently assuming that 




  if(! action_running_)
  {
    // Check that all goals achieved
    if((size_t) search_point_idx_ >= search_points_.size())
    {
      search_point_idx_ = 0;
      finish(true, 1.0, "All positions searched");

      /** TODO: Service-request that the area has been searched! */

      return;
    } 

    /** TODO: Might need a counter or timer, to ensure that each area is sufficiently covered, and that it is not speeding ahead! */

    // Not all goals achieved. Start the next one
    auto send_goal_options = rclcpp_action::Client<anafi_uav_interfaces::action::MoveToNED>::SendGoalOptions();

    send_goal_options.result_callback = [this](auto) 
    {
      /** 
       * WARNING: If a multithreaded executor is used, this could cause a race condition!
       * Should not be a problem for a single-threaded executor! 
       */
      action_running_ = false;
    };

    move_goal_.spherical_radius_of_acceptance = radius_of_acceptance_;
    move_goal_.ned_position = search_points_[search_point_idx_];
    search_point_idx_++;

    future_move_goal_handle_ = move_action_client_->async_send_goal(move_goal_, send_goal_options);
    action_running_ = true;
  }
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

  for(int point_idx = 0; point_idx < num_positions; point_idx + num_coordinates_per_point)
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
