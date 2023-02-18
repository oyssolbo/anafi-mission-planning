#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TestNode : 
public rclcpp::Node
//  public rclcpp_lifecycle::LifecycleNode 
//  public plansys2::ActionExecutorClient
{
public:
  TestNode()
  : rclcpp::Node("test_normal_node")
  // : rclcpp_lifecycle::LifecycleNode("test_lifecycle_node")
  // : plansys2::ActionExecutorClient("test_action_executor", 250ms)
  {
    progress_ = 0.0;

    using namespace std::placeholders;
    anafi_state_sub_const_share_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&TestNode::anafi_state_cb_const_share_, this, _1)); 
    anafi_state_sub_share_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&TestNode::anafi_state_cb_share_, this, _1)); 
    anafi_state_sub_share_10_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", 10, std::bind(&TestNode::anafi_state_cb_share_10_, this, _1));   
    anafi_state_sub_share_reliable_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).reliable(), std::bind(&TestNode::anafi_state_cb_share_realiable_, this, _1));   
  }

private:
  void do_work()
  {
  }

  float progress_;
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_const_share_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr anafi_state_sub_share_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr anafi_state_sub_share_10_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr anafi_state_sub_share_reliable_;

  void anafi_state_cb_const_share_(std_msgs::msg::String::ConstSharedPtr state_msg)
  {
    std::cout << "Constsharedptr: " << state_msg->data << std::endl;
  }
  void anafi_state_cb_share_(std_msgs::msg::String::SharedPtr state_msg)
  {
    std::cout << "Sharedptr" << state_msg->data << std::endl;
  }
  
  void anafi_state_cb_share_10_(std_msgs::msg::String::SharedPtr state_msg)
  {
    std::cout << "Sharedptr 10 " << state_msg->data << std::endl;
  }

  void anafi_state_cb_share_realiable_(std_msgs::msg::String::SharedPtr state_msg)
  {
    std::cout << "Sharedptr reliable " << state_msg->data << std::endl;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>();

  // node->set_parameter(rclcpp::Parameter("action_name", "search"));
  // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  // node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
