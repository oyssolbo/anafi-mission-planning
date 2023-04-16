#include "automated_planning/land_action_node.hpp"

bool LandActionNode::check_land_preconditions()
{
  // Detect that the helipad is detected during the last k seconds,
  // to ensure the EKF-estimate is relatively safe
  if(! helipad_detected_)
  {
    RCLCPP_ERROR(this->get_logger(), "Helipad not detected!");
    return false;
  }

  // rclcpp::Duration duration = this->get_clock()->now() - last_apriltags_detection_time_; // Important to specify RCL_ROS_TIME
  // if(duration.seconds() > max_last_apriltags_detection_time_s_)
  // {
  //   RCLCPP_ERROR(this->get_logger(), "Maximum allowed duration since last detection exceeded!\n
  //     Current duration: %f seconds. If new data is available, check clock definitions!", duration.seconds());
  //   return false;
  // }

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
  set_controller_state_(true, "Enabling GNC");
  return true;
}


LifecycleNodeInterface::CallbackReturn LandActionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Trying to activate land");

  bool preconditions_satisfied = check_land_preconditions();
  if(! preconditions_satisfied)
  {
    finish(false, 0.0, "Unable to land: Prechecks failed!");
    RCLCPP_WARN(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  send_feedback(0.0, "Prechecks finished. Cleared to land!");
  RCLCPP_INFO(this->get_logger(), "Landing activated"); // This is never achieved!
  
  // Activate lifecycle-publishers
  cmd_land_pub_->on_activate();
  desired_position_pub_->on_activate();
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn LandActionNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  cmd_land_pub_->on_deactivate();
  desired_position_pub_->on_deactivate();

  return ActionExecutorClient::on_deactivate(state);
}


void LandActionNode::do_work()
{
  if(anafi_state_.compare("FS_LANDED") == 0)
  {
    // Drone landed!
    finish(true, 1.0, "Anafi has landed!");
    RCLCPP_INFO(this->get_logger(), "Landed!");
    return;
  }

  bool is_inside_no_go_zone = check_inside_no_go_zone();
  bool is_target_position_achieved = check_target_position_achieved();

  // A good method should be used for simplifying this logic!
  // As of now, there are two possible transitions of code, where the most important 
  // check occurs first. If one of these occur, the state-transition function occurs

  /**
   * @warning May have a race-condition! It assumes that the velocity-controller is shut down,
   * which is not guaranteed! If rpyt-commands are received by the Olympe bridge after a land-
   * command is received, the landing will be canceled!
   * 
   * Unsure how to best solve this, since it is occuring inside a timer-callback
   * 
   * It is not a solution to just spam n land-commands, since the Olympe remembers them. The drone 
   * may immediately land after a takeoff-command is called... 
   */
  if(landing_state_ == LandingState::LAND && ! is_inside_no_go_zone && ! is_gnc_activated_)
  {
    send_feedback(0.99, "Initiating landing");
    RCLCPP_INFO(this->get_logger(), "Initiating landing");
    cmd_land_pub_->publish(std_msgs::msg::Empty());  
    return;
  }

  // State transitions
  bool is_prev_landing_state_land = false;
  if(is_inside_no_go_zone)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), 
      *this->get_clock(), 
      2000, 
      "Drone inside NOGO-zone during landing. Current altitude: %f", std::abs(ekf_output_.pose.pose.position.z)
    );

    switch (landing_state_)
    {
      case LandingState::LAND:
        is_prev_landing_state_land = true;
        landing_state_ = LandingState::HOVER_DIRECTLY_ABOVE;
        break;
      case LandingState::HOVER_DIRECTLY_ABOVE:
      case LandingState::INIT:
      default:
        landing_state_ = LandingState::INIT;
        break;
    }
  }
  else if(is_target_position_achieved)
  {
    RCLCPP_INFO(this->get_logger(), "Position achieved! Transitioning state");

    switch (landing_state_)
    {
      case LandingState::INIT:
        landing_state_ = LandingState::HOVER_DIRECTLY_ABOVE;
        break;
      case LandingState::HOVER_DIRECTLY_ABOVE:
      case LandingState::LAND:
        landing_state_ = LandingState::LAND;
        break;
      default:
        landing_state_ = LandingState::INIT;
        break;
    }
  }

  // Action transition functions 
  if(is_inside_no_go_zone || is_target_position_achieved)
  {
    switch (landing_state_)
    {
      case LandingState::INIT:
      {
        send_feedback(0.0, "State: INIT");
        RCLCPP_INFO(this->get_logger(), "State: INIT");
        break;
      }
      case LandingState::HOVER_DIRECTLY_ABOVE:
      {
        send_feedback(0.5, "State: HOVER_DIRECTLY_ABOVE");
        RCLCPP_INFO(this->get_logger(), "State: HOVER_DIRECTLY_ABOVE");

        // Initialize the controller again
        if(is_prev_landing_state_land)
        {
          set_controller_state_(true, "Reactivating the velocity controller"); 
        }
        break;
      }
      case LandingState::LAND:
      {
        send_feedback(0.9, "State: LAND");
        RCLCPP_INFO(this->get_logger(), "State: LAND");

        set_controller_state_(false, "Requesting shutdown of the velocity controller");
        break;
      }
      
      default:
      {
        RCLCPP_ERROR(this->get_logger(), "LandingState not implemented!");
        return finish(false, 0.0, "Not implemented");
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Current state idx (INIT = 0, HOVER_DIRECTLY_ABOVE = 1, LAND = 2): " + std::to_string(int(landing_state_)));

  desired_position_ = landing_points_[landing_state_];
  publish_desired_position_();
}


bool LandActionNode::check_inside_no_go_zone()
{
  // The EKF-output is relative to the helipad
  // x > 0 => helipad in front of drone
  // y > 0 => helipad to the right of drone
  // z > 0 => drone above helipad

  geometry_msgs::msg::Point point = ekf_output_.pose.pose.position;

  std::pair<double, double> altitudal_bounds = get_altitude_bounds_(landing_state_, point);

  std::stringstream ss;
  ss << "Altitudal bounds: (" << altitudal_bounds.first << ", " << altitudal_bounds.second << ") [(m, m)]" << "m\n";
  RCLCPP_INFO(this->get_logger(), ss.str());
  return (point.z < altitudal_bounds.first || point.z > altitudal_bounds.second);
}


bool LandActionNode::check_target_position_achieved()
{
  constexpr double vertical_radius_of_acceptance = 0.2;
  constexpr double horizontal_radius_of_acceptance = 0.15;

  geometry_msgs::msg::Point point = ekf_output_.pose.pose.position;
  double horizontal_distance = std::sqrt(
    std::pow(point.x - desired_position_.x, 2) + 
    std::pow(point.y - desired_position_.y, 2)
  ); 

  // z_r inverted to transform it to NED
  double vertical_distance = std::abs((-point.z) - desired_position_.z); 

  std::stringstream ss;
  ss << "Horizontal distance = " << horizontal_distance << " m, and vertical distance = " << vertical_distance << "m\n";
  RCLCPP_INFO(this->get_logger(), ss.str());

  return (horizontal_distance <= horizontal_radius_of_acceptance) && (vertical_distance <= vertical_radius_of_acceptance); 
}


// Buggy!
void LandActionNode::publish_desired_position_()
{
  geometry_msgs::msg::PointStamped point_msg;
  point_msg.header.stamp = this->now();
  point_msg.point = desired_position_;
  desired_position_pub_->publish(point_msg);
}


std::pair<double, double> LandActionNode::get_altitude_bounds_(const LandingState& state, const geometry_msgs::msg::Point& point)
{
  double lower_bound, upper_bound;

  // Config files
  constexpr double max_no_go_altitude = 1.5; // Assumes everything OK if above 1.5 m of altitude
  constexpr double max_allowed_altitude = 100.0; // Limited with respect to the law 

  constexpr double helipad_radius_m = 0.4; 
  constexpr double safety_factor = 0.7; // < 1.0

  constexpr double altitudal_offset = 0.5;

  double radius_m = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)); 
  double signed_radicand = radius_m - safety_factor * helipad_radius_m;

  // If signed_radicand less than zero, drone inside the safe landing area on the helipad
  // Allow the lower value to be slightly negative, to prevent numeric errors
  double lower_safe_altitude = (signed_radicand < 0) ? -0.1 : std::sqrt(std::abs(signed_radicand)); 

  switch (state)
  {
    case LandingState::INIT:
    {
      lower_bound = max_no_go_altitude;
      upper_bound = max_allowed_altitude;
      break;
    }
    case LandingState::HOVER_DIRECTLY_ABOVE:
    {
      lower_bound = std::min(lower_safe_altitude, max_no_go_altitude);
      upper_bound = std::abs(landing_points_[LandingState::INIT].z) + altitudal_offset;
      break;
    }
    case LandingState::LAND:
    {
      lower_bound = std::min(lower_safe_altitude, max_no_go_altitude);
      upper_bound = std::abs(landing_points_[LandingState::HOVER_DIRECTLY_ABOVE].z) + altitudal_offset;
      break;
    }
  }  
  return std::make_pair(lower_bound, upper_bound);
}


void LandActionNode::set_controller_state_(bool enable_controller, const std::string& log_str)
{
  RCLCPP_WARN(this->get_logger(), log_str);
  enable_velocity_control_client_->wait_for_service(1s);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = enable_controller;
  auto result = enable_velocity_control_client_->async_send_request(request);
  is_gnc_activated_ = enable_controller; 
}


void LandActionNode::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
{
  std::string state = state_msg->data;
  auto it = std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; });
  if(it == possible_anafi_states_.end())
  {
    // No state found
    return;
  }
  anafi_state_ = state;
}


void LandActionNode::ekf_cb_(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr ekf_msg)
{
  ekf_output_.header = ekf_msg->header;
  ekf_output_.pose = ekf_msg->pose;
}


void LandActionNode::polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  polled_vel_.header.stamp = vel_msg->header.stamp;
  polled_vel_.twist = vel_msg->twist;
}


void LandActionNode::battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg)
{
  battery_percentage_ = battery_msg->data;
}


void LandActionNode::apriltags_detected_cb_(anafi_uav_interfaces::msg::Float32Stamped::ConstSharedPtr detection_msg)
{  
  // The apriltags-message has timestamp of 0.0 
  last_apriltags_detection_time_ = this->get_clock()->now();
  helipad_detected_ = detection_msg->data > 0;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LandActionNode>();
  // auto node = std::make_shared<DummyLand>();

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
