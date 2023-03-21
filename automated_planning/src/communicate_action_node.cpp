#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"
#include "anafi_uav_interfaces/srv/set_finished_action.hpp"

using namespace std::chrono_literals;

class CommunicateActionNode 
  : public plansys2::ActionExecutorClient
{
public:
  CommunicateActionNode()
  : plansys2::ActionExecutorClient("communicate_action_node", 500ms)
  {
    set_finished_action_client_ = this->create_client<anafi_uav_interfaces::srv::SetFinishedAction>("/mission_controller/finished_action");
  }

// Lifecycle-events
LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Trying to activate communicate");

  const std::string location = get_arguments()[1];
  const std::string person_id = get_arguments()[2];

  RCLCPP_INFO(this->get_logger(), "Simulating communication for person " + person_id + " at location " + location); 
  finish(true, 1.0, "Communicated");

  set_communicate_action_finished_();

  return ActionExecutorClient::on_activate(previous_state);
}

private:  
  rclcpp::Client<anafi_uav_interfaces::srv::SetFinishedAction>::SharedPtr set_finished_action_client_;

  void do_work()
  {
  }

  bool set_communicate_action_finished_(const std::string& argument="")
  {
    auto request = std::make_shared<anafi_uav_interfaces::srv::SetFinishedAction::Request>();
    
    std_msgs::msg::String action_name_msg = std_msgs::msg::String();
    action_name_msg.data = "communicate";

    std_msgs::msg::String location_msg = std_msgs::msg::String();
    location_msg.data = get_arguments()[1]; // get_arguments returns { drone, location, person }

    constexpr int num_args = 2; // Hardcoded for now
    
    std::vector<std_msgs::msg::String> argument_msg; 
    
    std_msgs::msg::String arg;
    arg.data = get_arguments()[2];
    argument_msg.push_back(arg);

    arg.data = argument;
    argument_msg.push_back(arg);

    request->finished_action_name = action_name_msg;
    request->location = location_msg;
    request->arguments = argument_msg;
    request->num_arguments = num_args;

    if (! set_finished_action_client_->wait_for_service(1s)) 
    {
      RCLCPP_ERROR(this->get_logger(), "Service not available");
      return false;
    }

    auto result = set_finished_action_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to set action as finished!");
      return false;
    }

    return true;
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
