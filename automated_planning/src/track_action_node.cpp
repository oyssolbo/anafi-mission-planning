#include "automated_planning/track_action_node.hpp"


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // Getting the location to search
  if(! set_velocity_controller_state_(true, "Unable to activate the velocity controller"))
  {
    finish(false, 0.0, "Unable to activate the velocity controller");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  send_feedback(0.0, "Prechecks finished. Cleared to search!");

  // Hacky method of preventing errors with do_work running when the node is 
  // not activated
  node_activated_ = true;
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  node_activated_ = false;

  // If there is a running action, cancel / abort this
  // auto response = move_action_client_->async_cancel_all_goals();

  if(! set_velocity_controller_state_(false, "Unable to activate the velocity controller"))
  {
    finish(false, 0.0, "Unable to activate the velocity controller");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void TrackActionNode::do_work()
{
  if(! node_activated_)
  {
    return;
  }


  if(check_goal_achieved_())
  {
    hover_();
    
    static int counter = 0;
    const int max_count = 4;
    
    counter++;
    if(counter < max_count)
    {
      return;
    }
    counter = 0;

    std::string location = this->get_arguments()[1];
    std::string object = this->get_arguments()[2]; 

    RCLCPP_INFO(this->get_logger(), "Location " + location + " tracked with respect to object " + object);
    finish(true, 1.0, "Position tracked");
    return;
  }

  get_target_position_();
  pub_desired_ned_position_(goal_position_ned_);
}


bool TrackActionNode::get_target_position_(const std::string& preferred_target, double time_limit_s)
{
  bool point_found = false;
  geometry_msgs::msg::Point point;

  if(preferred_target.empty())
  {
    // Find target which was last detected 
    double last_detected_duration = time_limit_s;

    for (auto const& [key, val] : last_detections_)
    {
      rclcpp::Time detection_time = std::get<0>(val);
      double duration_s = (this->get_clock()->now() - detection_time).seconds();
      if(duration_s < last_detected_duration)
      {
        point = std::get<1>(val);
        point_found = true;
      }
    }
  }

  auto it = last_detections_.find(preferred_target);
  if(it == last_detections_.end() || ! point_found)
  {
    // Unable to find preferred target
    RCLCPP_WARN(this->get_logger(), "Unable to find object " + preferred_target);
    point = position_ned_;
  }
  else 
  {
    point_found = true;
  }

  goal_position_ned_ = point;
  goal_position_ned_.z -= 2.0; // Allow for some altitudal leway during tracking
  
  return point_found;
}



bool TrackActionNode::set_velocity_controller_state_(bool controller_state, const std::string& error_str)
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


bool TrackActionNode::check_goal_achieved_()
{
  Eigen::Vector3d pos_error_ned = get_position_error_ned_();
  double distance = pos_error_ned.norm();
  return distance <= radius_of_acceptance_;
}


void TrackActionNode::hover_()
{
  pub_desired_ned_position_(position_ned_);
}


void TrackActionNode::pub_desired_ned_position_(const geometry_msgs::msg::Point& target_position)
{
  geometry_msgs::msg::PointStamped point_msg = geometry_msgs::msg::PointStamped();
  point_msg.header.stamp = this->get_clock()->now();
  point_msg.point = target_position;

  goal_position_pub_->publish(point_msg);
}


Eigen::Vector3d TrackActionNode::get_position_error_ned_()
{
  double x_diff = position_ned_.x - goal_position_ned_.x;
  double y_diff = position_ned_.y - goal_position_ned_.y;
  double z_diff = position_ned_.z - goal_position_ned_.z;

  Eigen::Vector3d error_ned;
  error_ned << x_diff, y_diff, z_diff;
  return error_ned;
}


void TrackActionNode::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  position_ned_ = ned_pos_msg->point;
}


void TrackActionNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg)
{
  rclcpp::Time time = this->get_clock()->now();
  last_detections_["person"] = std::make_tuple(time, detected_person_msg->position); 
}


void TrackActionNode::apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr)
{
  rclcpp::Time time = this->get_clock()->now();
  last_detections_["helipad"] = std::make_tuple(time, position_ned_); 
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackActionNode>();

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
