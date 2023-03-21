#include "automated_planning/takeoff_action_node.hpp"


bool TakeoffActionNode::check_takeoff_preconditions_()
{
  const double min_battery_percentage = 25; // Hardcoded for now -> config file eventually
  return ((battery_percentage_ >= min_battery_percentage) && (anafi_state_.compare("FS_LANDED") == 0));
}


LifecycleNodeInterface::CallbackReturn TakeoffActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Trying to activate takeoff");

  bool preconditions_satisfied = check_takeoff_preconditions_();
  if(! preconditions_satisfied)
  {
    finish(false, 0.0, "Unable to start takeoff: Prechecks failed!");
    RCLCPP_WARN(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  send_feedback(0.0, "Prechecks finished. Cleared to start!");
  
  // Activate lifecycle-publisher and order a takeoff
  RCLCPP_INFO(this->get_logger(), "Activating takeoff");
  cmd_takeoff_pub_->on_activate();
  cmd_takeoff_pub_->publish(std_msgs::msg::Empty());
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn TakeoffActionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating takeoff");
  cmd_takeoff_pub_->on_deactivate();

  return ActionExecutorClient::on_deactivate(state);
}


void TakeoffActionNode::do_work()
{
  // Check that the drone is hovering
  static int num_hovering_attempts = 0;
  static int max_hovering_attempts = 10; // 2.5 seconds at 250ms rate

  static int num_takeoffs_ordered = 0;
  static int max_takeoffs_ordered = 3;

  if(anafi_state_.compare("FS_HOVERING") == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Takeoff finished: Drone hovering!");
    finish(true, 1.0, "Hovering");
    
    // Reset variables before finishing
    num_hovering_attempts = 0;
    num_takeoffs_ordered = 0;
    return;
  }

  num_hovering_attempts++;
  if(num_hovering_attempts >= max_hovering_attempts)
  {
    num_hovering_attempts = 0;
    num_takeoffs_ordered++;
    if(num_takeoffs_ordered >= max_takeoffs_ordered)
    {
      finish(false, 0.0, "Takeoff failed");
      RCLCPP_ERROR(this->get_logger(), "Takeoff failed. Maximum attempts exceeded! Check the drone!");
      
      // Reset variables before finishing
      num_hovering_attempts = 0;
      num_takeoffs_ordered = 0;
      return;
    }

    // Retry takeoff
    cmd_takeoff_pub_->publish(std_msgs::msg::Empty());
  }
}


void TakeoffActionNode::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
{
  std::string state = state_msg->data;
  if(std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; }) == possible_anafi_states_.end())
  {
    // No state found
    return;
  }
  anafi_state_ = state;
}


void TakeoffActionNode::battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg)
{
  battery_percentage_ = battery_msg->data;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "takeoff"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
