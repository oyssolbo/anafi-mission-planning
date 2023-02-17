#pragma once

#include <memory>
#include <string>
#include <algorithm>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using LifecycleNodeInterface = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;


class TakeoffAction : public plansys2::ActionExecutorClient
{
public:
  TakeoffAction() 
  : plansys2::ActionExecutorClient("takeoff", 250ms)
  , battery_percentage_(0.0) // Initializing to fail preconditiions-check if no battery-msg received 
  {
    // May have some problems with QoS when interfacing with ROS1
    // The publishers are on mode reliable, to increase the likelihood of sending the message
    // The subscribers are on best-effort, since similar data will be transmitted several times. If
    // some packages are lost, so be it.
    cmd_takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/anafi/cmd_takeoff", rclcpp::QoS(1).reliable());

    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", rclcpp::QoS(1).best_effort(), std::bind(&TakeoffAction::anafi_state_cb_, this, _1));   
    battery_charge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/anafi/battery", rclcpp::QoS(1).best_effort(), std::bind(&TakeoffAction::battery_charge_cb_, this, _1));   

    // Assuming the velocity controller will be used throughout this thesis
    // Future improvement to allow for enabling the MPC
    enable_velocity_control_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/velocity_controller/service/enable_controller"); 
  }

  // Lifecycle-events
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);


private:
  // State
  std::string anafi_state_;
  double battery_percentage_;

  const std::vector<std::string> possible_anafi_states_ = 
    {"FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY"};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Empty>::SharedPtr cmd_takeoff_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr anafi_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr battery_charge_sub_;

  // Services
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_velocity_control_client_;


  // Private functions
  /**
   * @brief Overload of function in ActionExecutorClient. This function does the 
   * majority of the work when the node is activated. 
   * 
   * The function is called at the same frequency as the node, and is responsible for
   *    - Check if the drone has not taken off
   *      - If it has taken too long, call takeoff again
   *      - If a maximum number of takeoff tries are reached, fail the task
   */
  void do_work();


  /**
   * @brief Checking preconditions for takeoff, including battery charge and drone state
   */
  bool check_takeoff_preconditions();


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg);
  void battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg);

}; // TakeoffAction
