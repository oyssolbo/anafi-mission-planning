#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "anafi_uav_interfaces/msg/attitude_setpoint.hpp" // Test if it will compile. Will not be used in this action

using namespace std::chrono_literals;

class SearchAction : public plansys2::ActionExecutorClient
{
public:
  SearchAction()
  : plansys2::ActionExecutorClient("search", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    if (progress_ < 1.0) 
    {
      progress_ += 0.1;
      send_feedback(progress_, "Searching");
    }
    else 
    {
      finish(true, 1.0, "Searching completed");

      progress_ = 0.0;
      std::cout << std::endl;
    }

    std::cout << "Searching ... [" << std::min(100.0, progress_ * 100.0) << "%]  " << std::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SearchAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "search"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
