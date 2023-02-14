#include "automated_planning_ros2/move_action_node.hpp"

LifecycleNodeInterface::CallbackReturn
MoveAction::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  // TODO: Check that the state is correct, such that this is not triggered once 
  // the drone has landed f.ex.
  bool can_move = check_can_move();
  if(! can_move)
  {
    finish(false, 0.0, "Unable to start moving: Prechecks failed!");
    RCLCPP_WARN(this->get_logger(), "Prechecks failed!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Get the goal
  const std::string goal_location = get_arguments()[2]; 
  std::map<std::string, geometry_msgs::msg::PointStamped>::iterator it_goal_pos = locations_.find(goal_location);
  if(it_goal_pos == locations_.end())
  {
    finish(false, 0.0, "Unable to find goal location!");
    RCLCPP_WARN(this->get_logger(), "Goal location not found!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  goal_position_ned_ = std::get<1>(*it_goal_pos);
  start_distance_ = get_position_error_ned().norm();

  // Return if starting position close to the goal
  if(start_distance_ <= radius_of_acceptance_)
  {
    finish(true, 1.0, "Target achieved!");
    RCLCPP_INFO(this->get_logger(), "Target achieved!");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  send_feedback(0.0, "Prechecks finished. Cleared to start!");

  // Activating lifecyclepublishers
  cmd_move_by_pub_->on_activate();
  cmd_move_to_pub_->on_activate();
  
  return ActionExecutorClient::on_activate(previous_state);
}


LifecycleNodeInterface::CallbackReturn
MoveAction::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Deactivate publishers
  cmd_move_by_pub_->on_deactivate();
  cmd_move_to_pub_->on_deactivate();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}



void MoveAction::do_work()
{
  // Monitoring the states between function calls
  static bool hovering_ordered = false;
  static int hovering_attempts = 0;
  const int max_hovering_attempts = 10; // 2.5 seconds with 250ms rate

  static bool move_ordered = false;

  Eigen::Vector3d pos_error_ned = get_position_error_ned();
  double distance = pos_error_ned.norm();

  // Check if the drone has reached the target and hovers
  if(distance <= radius_of_acceptance_)
  {
    if(! hovering_ordered)
    {
      RCLCPP_INFO(this->get_logger(), "Inside sphere of acceptance for horizontal position");
      send_feedback(0.999, "Position reached");
      pub_moveby_cmd(0.0, 0.0, 0.0);
      hovering_ordered = true;
      return;
    }

    if(anafi_state_.compare("FS_HOVERING") != 0) // Check if the drone is hovering
    {
      // Not hovering
      hovering_attempts++;
      if(hovering_attempts >= max_hovering_attempts)
      {
        // Retry hovering
        hovering_attempts = 0;
        hovering_ordered = false;
        RCLCPP_DEBUG(this->get_logger(), "Retrying hovering. Too many attempts exceeded...");
      }
      return;
    }

    // Hovering
    finish(true, 1.0, "Hovering close to the goal location!");
    RCLCPP_INFO(this->get_logger(), "Drone hovering!");

    // Setting the variables to zero again for the next move
    hovering_ordered = false;
    hovering_attempts = 0;

    move_ordered = false;
    return;
  }

  // Not close enough to the target. Reset the variables for checking hovering
  hovering_ordered = false;
  hovering_attempts = 0;

  // Move the drone
  // attitude_.normalize();
  Eigen::Vector3d pos_error_body = attitude_.toRotationMatrix().transpose() * pos_error_ned;

  // std::cout << "Position error ned: " << pos_error_ned << std::endl;
  // std::cout << "Position error body: " << pos_error_body << std::endl;
  // std::cout << "Rotation matrix: " << attitude_.toRotationMatrix() << std::endl;

  // Important to not spam the drone with new commands, as it will cancel the move
  // https://developer.parrot.com/docs/olympe/arsdkng_ardrone3_piloting.html#olympe.messages.ardrone3.Piloting.moveBy

  bool is_drone_moving = check_movement_along_vector(pos_error_body);
  if(is_drone_moving && move_ordered)
  {
    // TODO: Should have some method for potentially cancel or restart the movement 
    // if it takes too long
    send_feedback(1.0 - distance / start_distance_, "Moving...");
  }
  else if(! move_ordered) 
  {
    float dx = static_cast<float>(pos_error_body.x());
    float dy = static_cast<float>(pos_error_body.y());
    float dz = static_cast<float>(pos_error_body.z());

    pub_moveby_cmd(dx, dy, dz);
    move_ordered = true;
  }
  else
  {
    // May want a timer or counter to ensure that the drone actually moves
    // The Anafi is quite terrible in listening at some commands...
  }
}


bool MoveAction::check_can_move()
{
  // Currently assume that it can always move if the drone is either flying or hovering
  return (anafi_state_.compare("FS_HOVERING") == 0) || (anafi_state_.compare("FS_FLYING") == 0);
}


bool MoveAction::check_movement_along_vector(const Eigen::Vector3d& move_vec)
{
  // The following conditions are checked: 
  // - the drone is flying
  // - the velocity vector is comparable to the positional error vector 

  // Check that the drone is flying
  if(anafi_state_.compare("FS_FLYING") != 0)
  {
    RCLCPP_DEBUG(this->get_logger(), "Anafi not in flying state");
    return false;
  }

  // Check that the movement-vector is sufficiently large
  double min_vec_norm = 0.5;
  if(move_vec.norm() < min_vec_norm)
  {
    RCLCPP_DEBUG(this->get_logger(), "Movement vector too small");
    return false;
  }

  // Check nonzero velocity
  Eigen::Vector3d vel_vec{ polled_vel_.twist.linear.x, polled_vel_.twist.linear.y, polled_vel_.twist.linear.z };
  double min_vel = 0.2; // Below this, it is more tracking / accurate control
  if(vel_vec.norm() < min_vel)
  {
    RCLCPP_DEBUG(this->get_logger(), "Velocity too low");
    return false;
  }

  // Check the horizontal angle to be small enough
  Eigen::Vector2d hor_move_vec{ move_vec.x(), move_vec.y() };
  Eigen::Vector2d hor_vel_vec{ vel_vec.x(), vel_vec.y() };
  double norm_hor_move_vec = hor_move_vec.norm();
  double norm_hor_vel_vec = hor_vel_vec.norm();

  double min_hor_norm = 0.01;
  if(norm_hor_move_vec < min_hor_norm || norm_hor_vel_vec < min_hor_norm)
  {
    RCLCPP_DEBUG(this->get_logger(), "Horizontal norm too low");
    return false;
  }

  const double pi = 3.14159265358979323846;
  double max_angle = 15 * pi / 180.0;

  // Not the most efficient method of using acos. atan would be better
  double angle = std::acos((hor_move_vec.dot(hor_vel_vec) / (norm_hor_move_vec * norm_hor_vel_vec)));
  return std::abs(angle) <= max_angle;
}


Eigen::Vector3d MoveAction::get_position_error_ned()
{
  geometry_msgs::msg::Point pos_ned = position_ned_.point;
  geometry_msgs::msg::Point goal_pos_ned = goal_position_ned_.point;

  double x_diff = pos_ned.x - goal_pos_ned.x;
  double y_diff = pos_ned.y - goal_pos_ned.y;
  double z_diff = pos_ned.z - goal_pos_ned.z;

  Eigen::Vector3d error_ned;
  error_ned << x_diff, y_diff, z_diff;
  return error_ned;
}


void MoveAction::pub_moveby_cmd(float dx, float dy, float dz)
{
  anafi_uav_interfaces::msg::MoveByCommand moveby_cmd = anafi_uav_interfaces::msg::MoveByCommand();
  moveby_cmd.header.stamp = this->now();
  moveby_cmd.dx = dx;
  moveby_cmd.dy = dy;
  moveby_cmd.dz = dz;
  moveby_cmd.dyaw = 0;

  cmd_move_by_pub_->publish(moveby_cmd);
}


void MoveAction::pub_moveto_cmd(double lat, double lon, double h)
{
  anafi_uav_interfaces::msg::MoveToCommand moveto_cmd = anafi_uav_interfaces::msg::MoveToCommand();
  moveto_cmd.header.stamp = this->now();
  moveto_cmd.latitude = lat;
  moveto_cmd.longitude = lon;
  moveto_cmd.altitude = h;
  moveto_cmd.heading = 0;
  moveto_cmd.orientation_mode = 0; // Drone will not orient itself during movement
  
  cmd_move_to_pub_->publish(moveto_cmd);
}


void MoveAction::anafi_state_cb_(std_msgs::msg::String::ConstSharedPtr state_msg)
{
  std::string state = state_msg->data;
  if(std::find_if(possible_anafi_states_.begin(), possible_anafi_states_.end(), [state](std::string str){ return state.compare(str) == 0; }) == possible_anafi_states_.end())
  {
    // No state found
    return;
  }
  anafi_state_ = state;
}


void MoveAction::ekf_cb_(anafi_uav_interfaces::msg::EkfOutput::ConstSharedPtr ekf_msg)
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


void MoveAction::ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  position_ned_.header.stamp = ned_pos_msg->header.stamp;
  position_ned_.point = ned_pos_msg->point;
}


void MoveAction::gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg)
{
  (void) gnss_data_msg;
}


void MoveAction::attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg)
{
  Eigen::Vector3d v(attitude_msg->quaternion.x, attitude_msg->quaternion.y, attitude_msg->quaternion.z);
  attitude_.w() = attitude_msg->quaternion.w;
  attitude_.vec() = v;
  attitude_.normalize();
}


void MoveAction::polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg)
{
  // Assume that the message is more recent for now... (bad assumption)
  polled_vel_.header.stamp = vel_msg->header.stamp;
  polled_vel_.twist = vel_msg->twist;
}



void MoveAction::init_locations()
{
  // TODO:
  // - add more locations
  // - add the locations into a config file or similar

  geometry_msgs::msg::PointStamped loc_ned_pos;
  loc_ned_pos.header.frame_id = "/map";
  loc_ned_pos.header.stamp = this->now();

  // Have some safety with respect to the altitude 
  loc_ned_pos.point.z = -5.0; 

  // Home location - this can be changed afterwards by getting some data from the 
  // Revolt as it moves
  loc_ned_pos.point.x = 0.0;
  loc_ned_pos.point.y = 0.0;
  locations_["h1"] = loc_ned_pos;


  loc_ned_pos.point.x = 20.0;
  loc_ned_pos.point.y = 0.0;
  locations_["a1"] = loc_ned_pos;

  loc_ned_pos.point.x = 0.0;
  loc_ned_pos.point.y = 20.0;
  locations_["a2"] = loc_ned_pos;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  try
  { 
    rclcpp::spin(node->get_node_base_interface());
  }
  catch(...) {}
  
  rclcpp::shutdown();

  return 0;
}
