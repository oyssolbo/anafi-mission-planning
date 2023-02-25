#include "automated_planning/drop_equipment_action.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DropEquipmentActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "drop_lifevest"));
  node->set_equipment(Equipment::LIFEVEST);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}