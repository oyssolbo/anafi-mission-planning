#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class TakeoffAction : public plansys2::ActionExecutorClient
{
public:
  TakeoffAction()
  : plansys2::ActionExecutorClient("takeoff", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) 
    {
      progress_ += 0.5;
      send_feedback(progress_, "Taking off");
    }
    else 
    {
      finish(true, 1.0, "Takeoff completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "Takeoff ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakeoffAction>();

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
