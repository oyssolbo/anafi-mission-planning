#include "automated_planning/track_action_node.hpp"


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Trying to activate track");

  // Tracking (as it is currently implemented) will only be used to track a person
  // The algorithm could be extended to track a helipad or similar in the future

  std::string person = this->get_arguments()[2]; // <drone - location - person> from sar_testing.pddl 
  std::string str_id = std::regex_replace (person, std::regex(R"([^0-9])"), ""); // Removes all non-numeric variables. Beware: assumes that numeric and non-numeric variables are separate ("satans69søringa420" => "69420")
  int person_idx_ = static_cast<int>(std::stoi(str_id));
  
  RCLCPP_INFO(this->get_logger(), "Trying to track person " + person);

  // There could be a raze-condition here, if the person has not beed received as detected by this node
  // Currently just forcing the drone to hover, but this should be improved by whomever is unfortunate enough 
  // to read these few lines (future work ftw!)
  auto it = detected_people_.find(person_idx_);
  if(it == detected_people_.end())
  {
    RCLCPP_WARN(this->get_logger(), "Person " + person + " has not been detected. Possible race condition! Hovering...");
    goal_position_ned_ = position_ned_;
  }
  else 
  {
    goal_position_ned_ = it->second;
    goal_position_ned_.z -= 2.0; // Small safety margin
  }

  send_feedback(0.0, "Prechecks finished!");

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


LifecycleNodeInterface::CallbackReturn TrackActionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  // If there is a running action, cancel / abort this
  RCLCPP_INFO(this->get_logger(), "Deactivate requested. Cancelling any action execution");
  auto response = move_action_client_->async_cancel_all_goals();

  return ActionExecutorClient::on_deactivate(state);
}


void TrackActionNode::do_work()
{
}


void TrackActionNode::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  position_ned_ = ned_pos_msg->point;
}


void TrackActionNode::detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg)
{
  int id = detected_person_msg->id;
  detected_people_[id] = detected_person_msg->position; 
}


void TrackActionNode::apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr)
{
  // rclcpp::Time time = this->get_clock()->now();
  // last_detections_["helipad"] = std::make_tuple(time, position_ned_); 
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrackActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "track"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
