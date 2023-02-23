#include "automated_planning/land_action_node.hpp"

bool LandActionNode::check_land_preconditions()
{
  // Detect that the helipad is detected during the last k seconds,
  // to ensure the EKF-estimate is relatively safe
  if(! helipad_detected_)
  {
    RCLCPP_ERROR(this->get_logger(), "Helipad never detected!");
    return false;
  }

  rclcpp::Duration duration = rclcpp::Clock{RCL_ROS_TIME}.now() - last_apriltags_detection_time_; // Important to specify RCL_ROS_TIME
  if(duration.seconds() > max_last_apriltags_detection_time_s_)
  {
    RCLCPP_ERROR(this->get_logger(), "Maximum allowed duration since last detection exceeded!\n\
      Current duration: %f If new data available, check clock definitions!", duration.seconds());
    return false;
  }

  // Verify that the helipad is in GO-state
  // Currently, this is always true, but could be a future extension to consider the roll/pitch angle of the ReVolt
  if(helipad_state_ == HelipadState::NO_GO)
  {
    RCLCPP_ERROR(this->get_logger(), "Helipad not safe!");
    return false;
  }

  // Check that the velocity controller is available, and disable it
  if(! enable_velocity_control_client_->wait_for_service(2s))
  {
    RCLCPP_ERROR(this->get_logger(), "Velocity controller not found!");
    return false;
  }

  // Enable the velocity controller
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  auto result = enable_velocity_control_client_->async_send_request(request);
  
  if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if(result.get()->success)
    {
      return true;
    }
  }
  RCLCPP_ERROR(this->get_logger(), "Controller service unavailable!");
  return false;
}


LifecycleNodeInterface::CallbackReturn LandActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  bool preconditions_satisfied = check_land_preconditions();
  if(! preconditions_satisfied)
  {
    finish(false, 0.0, "Unable to land: Prechecks failed!");
    RCLCPP_WARN(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  send_feedback(0.0, "Prechecks finished. Cleared to land!");
  
  // Activate lifecycle-publishers
  cmd_land_pub_->on_activate();
  desired_position_pub_->on_activate();

  // Hacky method of preventing errors with do_work running when the node is 
  // not activated
  node_activated_ = true;
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn LandActionNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  cmd_land_pub_->on_deactivate();
  desired_position_pub_->on_deactivate();

  node_activated_ = false;

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void LandActionNode::do_work()
{
  if(! node_activated_)
  {
    return;
  }

  if(anafi_state_.compare("FS_LANDED"))
  {
    // Drone landed!
    // Disable controller
    enable_velocity_control_client_->wait_for_service(1s);

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto result = enable_velocity_control_client_->async_send_request(request);
    
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      if(! result.get()->success)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable velocity controller! Shut it down manually!");
      }
    }

    finish(true, 1.0, "Anafi has landed!");
    RCLCPP_INFO(this->get_logger(), "Landed!");
    return;
  }

  bool is_inside_no_go_zone = check_inside_no_go_zone();
  bool is_target_position_achieved = check_target_position_achieved();

  switch (landing_state_)
  {
    case LandingState::INIT:
    {
      if(is_inside_no_go_zone)
      {
        landing_state_ = LandingState::INIT;
      }
      else if(is_target_position_achieved)
      {
        landing_state_ = LandingState::HOVER_DIRECTLY_ABOVE;
        send_feedback(0.5, "Trying to hover above platform");
      }
      break;
    }
    case LandingState::HOVER_DIRECTLY_ABOVE:
    {
      if(is_inside_no_go_zone)
      {
        landing_state_ = LandingState::INIT;
      }
      else if(is_target_position_achieved)
      {
        landing_state_ = LandingState::LAND;
        send_feedback(0.75, "Trying to land");
      }
      break;
    }
    case LandingState::LAND:
    {
      if(is_inside_no_go_zone)
      {
        landing_state_ = LandingState::HOVER_DIRECTLY_ABOVE;
      }
      else
      {
        // Close enough to attempt landing
        // TODO: May want to ensure that the controller does not produce vertical commands, as
        // that will cancel the landing
        cmd_land_pub_->publish(std_msgs::msg::Empty());
        send_feedback(0.9, "Landing called");
      }
      break;
    }
    
    default:
    {
      RCLCPP_ERROR(this->get_logger(), "LandingState not implemented!");
      return finish(false, 0.0, "Not implemented");
    }
  }

  desired_position_ = landing_points_[landing_state_];
  publish_desired_position_();
}


bool LandActionNode::check_inside_no_go_zone()
{
  // The EKF-output is relative to the helipad
  // x_r > 0 => helipad in front of drone
  // y_r > 0 => helipad to the right of drone
  // z_r > 0 => drone above helipad
  double altitude_m = ekf_output_.z_r;
  const double max_no_go_altitude = 1.5; // Assumes everything OK if above 1.5 m of altitude

  double radius_m = std::sqrt(std::pow(ekf_output_.x_r, 2) + std::pow(ekf_output_.y_r, 2)); 
  const double helipad_radius_m = 0.4; 

  switch (landing_state_)
  {
    case LandingState::INIT:
    {
      if(altitude_m >= max_no_go_altitude)
      {
        return true;
      }
      // In init, the drone should achieve the initial altitude, and 
      // therefore fail if below this
      break;
    }
    case LandingState::HOVER_DIRECTLY_ABOVE:
    {
      double safety_factor = 0.8; // \leq 1.0
      double min_safe_altitude_m = std::sqrt(std::abs(radius_m - safety_factor * helipad_radius_m));

      if(altitude_m >= min_safe_altitude_m)
      {
        return true;
      }
      // One could check whether this to be above max_no_go_altitude,
      // however it would be preferable to transition to INIT-state then
      break;
    }
    case LandingState::LAND:
    {
      double safety_factor = 0.6; // < 1.0
      double min_safe_altitude_m = std::sqrt(std::abs(radius_m - safety_factor * helipad_radius_m));
      
      // This is a hard-limit!
      if(altitude_m >= min_safe_altitude_m)
      {
        return true;
      }
      break;
    }
    default:
    {
      break;
    }
  }

  return false; 
}


bool LandActionNode::check_target_position_achieved()
{
  const double vertical_radius_of_acceptance = 0.2;
  const double horizontal_radius_of_acceptance = 0.15;

  double horizontal_distance = std::sqrt(
    std::pow(ekf_output_.x_r - desired_position_.x, 2) + 
    std::pow(ekf_output_.y_r - desired_position_.y, 2)
  ); 

  // z_r inverted to transform it to NED
  double vertical_distance = std::abs((-ekf_output_.z_r) - desired_position_.z); 

  return (horizontal_distance <= horizontal_radius_of_acceptance) && (vertical_distance <= vertical_radius_of_acceptance); 
}



void LandActionNode::publish_desired_position_()
{
  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.stamp = this->now();
  point_msg.point = desired_position_;
  desired_position_pub_->publish(point_msg);
}


void LandActionNode::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
{
  std::string state = state_msg->data;
  if(std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; }) == possible_anafi_states_.end())
  {
    // No state found
    return;
  }
  anafi_state_ = state;
}


void LandActionNode::ekf_cb_(anafi_uav_interfaces::msg::EkfOutput::ConstSharedPtr ekf_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  ekf_output_.header.stamp = ekf_msg->header.stamp;
  ekf_output_.x_r = ekf_msg->x_r;
  ekf_output_.y_r = ekf_msg->y_r;
  ekf_output_.z_r = ekf_msg->z_r;
  ekf_output_.u_r = ekf_msg->u_r;
  ekf_output_.v_r = ekf_msg->v_r;
  ekf_output_.w_r = ekf_msg->w_r;
}


void LandActionNode::polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  polled_vel_.header.stamp = vel_msg->header.stamp;
  polled_vel_.twist = vel_msg->twist;
}


void LandActionNode::battery_charge_cb_(std_msgs::msg::UInt8::ConstSharedPtr battery_msg)
{
  battery_percentage_ = battery_msg->data;
}


void LandActionNode::apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg)
{
  builtin_interfaces::msg::Time detection_time = detection_msg->header.stamp;

  rclcpp::Time msg_time = rclcpp::Time(detection_time, rcl_clock_type_t::RCL_ROS_TIME);
  rclcpp::Duration time_diff = msg_time - last_apriltags_detection_time_;

  if(time_diff.nanoseconds() > 0)
  {
    last_apriltags_detection_time_ = msg_time;
    helipad_detected_ = true;
  }
  else 
  {
    RCLCPP_WARN(this->get_logger(), "Old detection data received. Check clock definitions!");
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandActionNode>();

  node->set_parameter(rclcpp::Parameter("action_name", "land"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}

  rclcpp::shutdown();

  return 0;
}
