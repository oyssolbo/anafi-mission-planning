#include "automated_planning/drop_lifevest_action_node.hpp"

bool DropLifevestActionNode::check_drop_preconditions()
{
  // Fail if not hovering or no person is detected or hovering far away from the desired object
  if(anafi_state_.compare("FS_HOVERING") != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot drop when the drone is not hovering. Current state: " + anafi_state_);
    return false;
  }

  // Checking for position will weakly check that a person is detected or not
  // If a person has not been detected, ROS will initialize the position as 0
  geometry_msgs::msg::Point point_of_detected_person_ = std::get<0>(detected_person_);
  double x_diff = position_ned_.point.x - point_of_detected_person_.x;
  double y_diff = position_ned_.point.y - point_of_detected_person_.y;
  
  const double max_horizontal_distance = 2;
  double horizontal_distance = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));

  if(horizontal_distance > max_horizontal_distance)
  {
    RCLCPP_ERROR(this->get_logger(), "Horizontal distance to large. Current distance: %f", horizontal_distance);
    return false;
  }

  // Check that the altitude is low enough, to prevent the equipment to drift with wind or
  // get too much kinetic energy when released 
  const double max_altitude = 4;
  if(std::abs(position_ned_.point.z) > max_altitude)
  {
    RCLCPP_ERROR(this->get_logger(), "Altitude too high to safely drop. Is object tracked?");
    return false;
  }

  return true;
}


LifecycleNodeInterface::CallbackReturn DropLifevestActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  bool preconditions_satisfied = check_drop_preconditions();
  if(! preconditions_satisfied)
  {
    finish(false, 0.0, "Unable to drop lifevest: Prechecks failed!");
    RCLCPP_ERROR(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  send_feedback(0.0, "Prechecks finished. Cleared to drop lifevest!");
  
  // Hacky method of preventing errors with do_work running when the node is not activated
  node_activated_ = true;
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn DropLifevestActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  node_activated_ = false;
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void DropLifevestActionNode::do_work()
{
  if(! node_activated_)
  {
    return;
  }

  geometry_msgs::msg::Point detected_position = std::get<0>(detected_person_);
  Severity detected_severity = std::get<1>(detected_person_);

  std::vector<geometry_msgs::msg::Point>::iterator it;
  it = std::find(previously_helped_people_.begin(), previously_helped_people_.end(), detected_position);
  if(it != previously_helped_people_.end())
  {
    RCLCPP_INFO(this->get_logger(), "Position marked as helped previously");
    finish(true, 1.0, "Position helped previously!");
    return;
  }

  bool lifevest_dropped;
  switch (detected_severity)
  {
    case Severity::MINOR:
    {
      RCLCPP_INFO(this->get_logger(), "Severity detected as minor.");
      lifevest_dropped = false;
      break;
    }
    case Severity::MODERATE:
    case Severity::HIGH:
    {
      RCLCPP_INFO(this->get_logger(), "Dropping lifevest at position!");
      lifevest_dropped = drop_lifevest_();
      break;
    }
    default:
    {
      break;
    }
  }
  if(lifevest_dropped)
  {
    RCLCPP_INFO(this->get_logger(), "Lifevest dropped on position");
    update_controller_of_lifevest_status_();
  }

  previously_helped_people_.push_back(detected_position);
  finish(true, 1.0);
}


bool DropLifevestActionNode::drop_lifevest_()
{
  if(num_lifevests_ > 0)
  {
    num_lifevests_--;
    return true;
  }
  return false;
}


void DropLifevestActionNode::update_controller_of_lifevest_status_()
{
  auto request = std::make_shared<anafi_uav_interfaces::srv::SetEquipmentNumbers::Request>();
  request->num_equipment = num_lifevests_;

  if (! set_num_markers_client_->wait_for_service(1s)) 
  {
    RCLCPP_ERROR(this->get_logger(), "Service for updating number of markers not available!");
    return;
  }

  auto result = set_num_markers_client_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service");
  } 
}




void DropLifevestActionNode::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
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


void DropLifevestActionNode::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  position_ned_.header.stamp = ned_pos_msg->header.stamp;
  position_ned_.point = ned_pos_msg->point;
}


void DropLifevestActionNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg)
{
  // Could be a bit problematic if there are multiple detections arriving at the same time,
  // for example that there are two people in the image 

  // This implementation assumes at most one person will be detected at a time

  // Using a vector for now, to allow for lambda
  std::vector<geometry_msgs::msg::Point> detected_people = { std::get<0>(detected_person_) };

  geometry_msgs::msg::Point position = detected_person_msg->position;
  Severity severity = Severity(detected_person_msg->severity);

  const double allowed_distance = 2.0; // Discuss this with Simen
  auto lambda = [&position, &allowed_distance](geometry_msgs::msg::Point pt)
  { 
    return (std::sqrt(std::pow((pt.x - position.x), 2) + std::pow((pt.y - position.y), 2)) <= allowed_distance); 
  };

  auto helped_it = std::find_if(
    previously_helped_people_.begin(), 
    previously_helped_people_.end(), 
    lambda
  );

  auto detected_it = std::find_if(
    detected_people.begin(), 
    detected_people.end(), 
    lambda
  );

  if(helped_it != previously_helped_people_.end() || detected_it != detected_people.end())
  {
    // Previously detected
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Detected person previously...");
    return;
  } 

  // TODO: This will currently cause information about previously detected people to be lost!
  // Implement this in a better way, if one allows for multiple people within the frame
  detected_person_ = std::make_tuple(position, severity);
}




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropLifevestActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "drop_lifevest"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
