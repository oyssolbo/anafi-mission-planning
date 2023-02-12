#include "automated_planning_ros2/move_action_node.hpp"


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  send_feedback(0.0, "Move starting");

  // The examples use action servers and clients for controlling the operations. Due to
  // the interaction through the ros1_bridge, actions are not supported according to 
  // my understanding
  // For the use with the ROS1_bridge, the method of choice should start calling the 
  // Anafi's internal functions and monitor its progress



  // Check that the state is correct, such that this is not triggered once the drone has
  // landed f.ex.
  bool can_move = true;
  if(! can_move)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }


  // Publish a message to the rest of the system for desired movement, based on the current state
  // This might be handled inside of the do_work function, assuming that it is working as intended

  return ActionExecutorClient::on_activate(previous_state); // Will create a timer and call the do_work function?
}


void MoveAction::do_work()
{
  // For testing how the rest of the system work

  // The do_work function is called at a certain time-step. This should then be used to evaluate 
  // the progress of the system, and determine in which position it should move to based on the 
  // current data available

  static int i = 0;
  RCLCPP_INFO(this->get_logger(), "do_work() called with i = %i", i);
  i++;

  ned_position_.point.x = 0;
  ned_position_.point.y = 0;
  ned_position_.point.z = 0;
  goal_position_.point.x = 0;
  goal_position_.point.y = 0.5;
  goal_position_.point.z = 0;

  geometry_msgs::msg::Point position = ned_position_.point; 
  geometry_msgs::msg::Point goal = goal_position_.point;
  double distance = std::abs(get_horizontal_distance(position, goal));

  RCLCPP_INFO(this->get_logger(), "distance: %d", distance);

  double radius_of_acceptance = 1.0;
  if(distance <= radius_of_acceptance)
  {
    // TODO: May also want to take the vertical position into account
    send_feedback(1.0, "Horizontal position reached");
  }
  else
  {
    send_feedback(0.0, "Testing something...");
  }
  finish(false, 0.99, "test");

  // finish(false) will stop the plan immideately, but not alert the executor? The executor must poll the value 
  // finish(true) starts performing the next action, until the entire plan is solved, but it does not appear to alert the executor?
}



double MoveAction::get_horizontal_distance(
  const geometry_msgs::msg::Point& point1,
  const geometry_msgs::msg::Point& point2)
{
  return std::sqrt(
    std::pow((point1.x - point2.x), 2) + std::pow((point1.y - point2.y), 2) // This returns some odd values...
  );
}


void MoveAction::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
{
  std::string state = state_msg->data;
  // if(std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; }) == possible_anafi_states_.end())
  // {
  //   // No state found
  //   return;
  // }
  anafi_state_ = state;
}


void MoveAction::ekf_cb_(anafi_uav_interfaces::msg::EkfOutput::ConstSharedPtr ekf_msg)
{
  (void) ekf_msg;
}


void MoveAction::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  (void) ned_pos_msg;
}


void MoveAction::gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg)
{
  (void) gnss_data_msg;
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
