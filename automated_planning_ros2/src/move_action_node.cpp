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
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  // Get the goal
  const std::string goal_location = get_arguments()[2]; 
  std::map<std::string, geometry_msgs::msg::PointStamped>::iterator it_goal_pos = locations_.find(goal_location);
  if(it_goal_pos == locations_.end())
  {
    finish(false, 0.0, "Unable to find goal location!");
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  goal_position_ned_ = std::get<1>(*it_goal_pos);
  start_distance_ = get_position_error_ned().norm();

  // Return if starting position close to the goal
  if(start_distance_ <= radius_of_acceptance_)
  {
    finish(true, 1.0, "Target achieved!");
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
      }
      return;
    }

    // Hovering
    finish(true, 1.0, "Hovering close to the goal location!");

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

  bool is_drone_moving = check_movement();
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


bool MoveAction::check_movement()
{
  // To check movement, one could check the conditions:
  // - the drone is flying
  // - the position is changing (velocity comparable to the movement vector)
  // Currently just checking that the drone is flying, such that the velocity vector 
  // could be a future improvement
  return anafi_state_.compare("FS_FLYING") == 0;
}


Eigen::Vector3d MoveAction::get_position_error_ned()
{
  geometry_msgs::msg::Point pos_ned = ned_position_.point;
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
  (void) ned_pos_msg;
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



void MoveAction::init_locations()
{
  // Assumes that hardcoding some values for testing is alright

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
