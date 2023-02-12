#include "automated_planning_ros2/move_action_node.hpp"
//#include "move_action_node.hpp"


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // send_feedback(0.0, "Move starting");

  // navigation_action_client_ =
  //   rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
  //   shared_from_this(),
  //   "navigate_to_pose");

  // bool is_action_server_ready = false;
  // do {
  //   RCLCPP_INFO(get_logger(), "Waiting for navigation action server...");

  //   is_action_server_ready =
  //     navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  // } while (!is_action_server_ready);

  // RCLCPP_INFO(get_logger(), "Navigation action server ready");

  // auto wp_to_navigate = get_arguments()[2];  // The goal is in the 3rd argument of the action
  // RCLCPP_INFO(get_logger(), "Start navigation to [%s]", wp_to_navigate.c_str());

  // goal_pos_ = waypoints_[wp_to_navigate];
  // navigation_goal_.pose = goal_pos_;

  // dist_to_move = getDistance(goal_pos_.pose, current_pos_);

  // auto send_goal_options =
  //   rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  // send_goal_options.feedback_callback = [this](
  //   NavigationGoalHandle::SharedPtr,
  //   NavigationFeedback feedback) {
  //     send_feedback(
  //       std::min(1.0, std::max(0.0, 1.0 - (feedback->distance_remaining / dist_to_move))),
  //       "Move running");
  //   };

  // send_goal_options.result_callback = [this](auto) {
  //     finish(true, 1.0, "Move completed");
  //   };

  // future_navigation_goal_handle_ =
  //   navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);

  return ActionExecutorClient::on_activate(previous_state);
}


double MoveAction::get_horizontal_distance(
  const geometry_msgs::msg::Pose& pose1,
  const geometry_msgs::msg::Pose& pose2)
{
  return std::sqrt(
    (pose1.position.x - pose2.position.x) * (pose1.position.x - pose2.position.x) +
    (pose1.position.y - pose2.position.y) * (pose1.position.y - pose2.position.y)
  );
}


void MoveAction::anafi_state_cb_(const std_msgs::msg::String::SharedPtr &state_msg)
{
  std::string state = state_msg->data;
  if(std::find(possible_anafi_states_.begin(), possible_anafi_states_.end(), state) == possible_anafi_states_.end())
  {
    // Not relevant information
    return;
  }
  anafi_state_ = state;
}


void MoveAction::ekf_cb_(const anafi_uav_interfaces::msg::EkfOutput::SharedPtr &ekf_msg)
{

}


void MoveAction::ned_pos_cb_(const geometry_msgs::msg::PointStamped::SharedPtr &ned_pos_msg)
{

}


void MoveAction::gnss_data_cb_(const sensor_msgs::msg::NavSatFix::SharedPtr &gnss_data_msg)
{

}



void MoveAction::init_locations()
{
  // Assumes that hardcoding some values for testing is alright

  geometry_msgs::msg::PointStamped loc_ned_pos;
  loc_ned_pos.header.frame_id = "/map";
  loc_ned_pos.header.stamp = this->now();

  // Have some safety with respect to the altitude 
  loc_ned_pos.point.z = -5.0; 

  // Home location - this can be changed afterwards by getting some data from the 
  // Revolt as it moves
  loc_ned_pos.point.x = 0.0;
  loc_ned_pos.point.y = 0.0;
  locations_["h1"] = loc_ned_pos;


  loc_ned_pos.point.x = 20.0;
  loc_ned_pos.point.y = 0.0;
  locations_["a1"] = loc_ned_pos;

  loc_ned_pos.point.x = 0.0;
  loc_ned_pos.point.y = 20.0;
  locations_["a2"] = loc_ned_pos;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}
  
  rclcpp::shutdown();

  return 0;
}
