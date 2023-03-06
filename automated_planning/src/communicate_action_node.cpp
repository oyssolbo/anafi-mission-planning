#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CommunicateActionNode 
  : public plansys2::ActionExecutorClient
{
public:
  CommunicateActionNode()
  : plansys2::ActionExecutorClient("communicate_action_node", 500ms)
  {
  }

// Lifecycle-events
LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
{
  const std::string location = get_arguments()[1];
  const std::string person_id = get_arguments()[2];

  RCLCPP_INFO(this->get_logger(), "Simulating communication for person " + person_id + " at location " + location); 
  finish(true, 1.0, "Communicated");

  return ActionExecutorClient::on_activate(previous_state);
}

private:  
  void do_work()
  {
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CommunicateActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "communicate"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
