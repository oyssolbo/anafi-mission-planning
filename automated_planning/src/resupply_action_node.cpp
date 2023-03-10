#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ResupplyActionNode 
  : public plansys2::ActionExecutorClient
{
public:
  ResupplyActionNode()
  : plansys2::ActionExecutorClient("resupply_action_node", 500ms)
  {
    this->declare_parameter("locations.resupply_available", std::vector<std::string>());
  }

// Lifecycle-events
LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
{
  const std::string location = get_arguments()[1];
  std::vector<std::string> resupply_locations = this->get_parameter("locations.resupply_available").as_string_array();

  auto it = std::find(resupply_locations.begin(), resupply_locations.end(), location);
  if(it == resupply_locations.end())
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to resupply at " + location);
    finish(false, 0.0, "Unable to resupply at location");
    return LifecycleNode::CallbackReturn::FAILURE;
  }

  /** TODO: Also check that the drone has landed! */

  RCLCPP_INFO(this->get_logger(), "Simulating resupply at location " + location);
  RCLCPP_WARN(this->get_logger(), "The resupply-action is not coordinated with other modules, and set to fail!"); 
  finish(false, 1.0, "Not properly implemented");

  return LifecycleNode::CallbackReturn::SUCCESS;
  //return ActionExecutorClient::on_activate(previous_state);
}

private:  
  void do_work()
  {
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ResupplyActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "resupply"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
