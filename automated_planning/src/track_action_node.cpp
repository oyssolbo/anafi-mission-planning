#include "automated_planning/track_action_node.hpp"


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activate requested");

  std::string preferred_target = "";
  try
  {
    // Can be out of range for some files
    preferred_target = this->get_arguments()[2]; // <drone - location - trackable> 
  }
  catch(...)
  {
    RCLCPP_WARN(this->get_logger(), "Unable to acquire trackable object. Is the PDDL-file correct?");
  }

  const double time_limit_s = 5.0;
  if(! get_target_position_(preferred_target, time_limit_s))
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to find preferred_target " + preferred_target + " within the time limit [s] " + std::to_string(time_limit_s));
    finish(false, 0.0, "Unable to detect the preferred object within time-limit");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  send_feedback(0.0, "Prechecks finished. Cleared to search!");

  // Start movement to desired position
  auto send_goal_options = rclcpp_action::Client<anafi_uav_interfaces::action::MoveToNED>::SendGoalOptions();

  send_goal_options.result_callback = [this](auto) 
  {
    RCLCPP_INFO(this->get_logger(), "Action finished");
    finish(true, 1.0, "Target achieved");
  };
  move_goal_.spherical_radius_of_acceptance = radius_of_acceptance_;
  move_goal_.ned_position = goal_position_ned_;

  future_move_goal_handle_ = move_action_client_->async_send_goal(move_goal_, send_goal_options);
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  // If there is a running action, cancel / abort this
  RCLCPP_INFO(this->get_logger(), "Deactivate requested. Cancelling any action execution");
  auto response = move_action_client_->async_cancel_all_goals();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void TrackActionNode::do_work()
{
}


bool TrackActionNode::get_target_position_(const std::string& preferred_target, double time_limit_s)
{
  geometry_msgs::msg::Point point = position_ned_;
  rclcpp::Time time_now = this->get_clock()->now();

  bool point_found = false;
  auto it = last_detections_.find(preferred_target);

  if(it != last_detections_.end())
  {
    rclcpp::Time detection_time = std::get<0>(it->second);
    if(std::abs((time_now - detection_time).seconds()) <= time_limit_s)
    {
      point = std::get<1>(it->second);
      point_found = true;
    }
  }
  else if(preferred_target.empty())
  {
    // Find target which was last detected 
    double last_detected_duration = time_limit_s;

    for (auto const& [key, val] : last_detections_)
    {
      rclcpp::Time detection_time = std::get<0>(val);
      double duration_s = (time_now - detection_time).seconds();
      if(duration_s < last_detected_duration)
      {
        last_detected_duration = duration_s;
        point = std::get<1>(val);
        point_found = true;
      }
    }
  }

  // Beware that the detected object will likely be at sea
  // The goal position should be slightly above the desired object, if said object found
  goal_position_ned_ = point;
  if(point_found)
  {
    // Track to a point above the object of interest 
    goal_position_ned_.z -= 2.0;
  }
  return point_found;
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
