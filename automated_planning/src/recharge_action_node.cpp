#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RechargeActionNode 
  : public plansys2::ActionExecutorClient
{
public:
  RechargeActionNode()
  : plansys2::ActionExecutorClient("recharge_action_node", 500ms)
  {
    this->declare_parameter("locations.recharge_available", std::vector<std::string>());
  }

// Lifecycle-events
LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Trying to activate recharge");

  const std::string location = get_arguments()[1];
  std::vector<std::string> recharge_locations = this->get_parameter("locations.recharge_available").as_string_array();

  auto it = std::find(recharge_locations.begin(), recharge_locations.end(), location);
  if(it == recharge_locations.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to recharge at " + location);
    finish(false, 0.0, "Unable to recharge at location");
    return LifecycleNode::CallbackReturn::FAILURE;
  }

  /** TODO: Also check that the drone has landed! */

  RCLCPP_INFO(this->get_logger(), "Simulating recharge at location " + location);
  RCLCPP_WARN(this->get_logger(), "The recharge-action is not implemented, and will fail!"); 
  finish(false, 1.0, "Not properly implemented");

  return LifecycleNode::CallbackReturn::FAILURE;
}

private:  
  void do_work()
  {
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "recharge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
