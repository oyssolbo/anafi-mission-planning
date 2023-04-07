#pragma once

#include <memory>
#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <tuple>
#include <string>
#include <optional>
#include <Eigen/Geometry>
#include <stdint.h>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/qos.hpp"

#include <plansys2_pddl_parser/Utils.h>
#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"
#include "plansys2_msgs/msg/tree.hpp"
#include "plansys2_msgs/msg/node.hpp"

#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "anafi_uav_interfaces/msg/stamped_string.hpp"
#include "anafi_uav_interfaces/msg/detected_person.hpp"
#include "anafi_uav_interfaces/srv/set_equipment_numbers.hpp"
#include "anafi_uav_interfaces/srv/set_finished_action.hpp"


enum class Severity{ MINOR, MODERATE, HIGH };
enum class ControllerState { INIT, SEARCH, RESCUE, EMERGENCY, AREA_UNAVAILABLE, IDLE };


struct MissionGoals
{
  std::string landed_goal_str_;
  std::string preferred_landing_goal_str_;
  std::vector<std::string> possible_landing_goal_strings_;

  std::vector<std::string> search_goal_strings_;
  std::vector<std::string> communicate_location_goal_strings_;
  std::vector<std::string> mark_location_goal_strings_;
  std::vector<std::string> rescue_location_goal_strings_;
};


class MissionControllerNode : public rclcpp::Node
{
public:
  MissionControllerNode()
  : rclcpp::Node("mission_controller_node") 
  , controller_state_(ControllerState::INIT)
  , battery_charge_(-1) // Set to -1 to indicate that it is not updated
  , previous_plan_str_("")
  , is_emergency_(false)
  , is_low_battery_(false)
  , is_person_detected_(false)
  {
    /**
     * Declare parameters for the drone
     */ 
    std::string drone_prefix = "drone.";
    this->declare_parameter(drone_prefix + "name");             // Fail if not declared in config

    std::string battery_usage_prefix = drone_prefix + "battery_usage_per_time_unit.";
    this->declare_parameter(battery_usage_prefix + "track");    // Fail if not declared in config
    this->declare_parameter(battery_usage_prefix + "move");     // Fail if not declared in config

    std::string velocity_limits_prefix = drone_prefix + "velocity_limits.";
    this->declare_parameter(velocity_limits_prefix + "track");  // Fail if not declared in config
    this->declare_parameter(velocity_limits_prefix + "move");   // Fail if not declared in config
 
    /**
     * Declare parameters for locations
     */ 
    std::string location_prefix = "locations.";
    this->declare_parameter(location_prefix + "names");         // Fail if not declared in config

    std::vector<std::string> locations_names = this->get_parameter(location_prefix + "names").as_string_array();
    std::string paths_prefix = location_prefix + "paths.";
    for(std::string loc_name : locations_names)
    {
      this->declare_parameter(paths_prefix + loc_name);
    }
    this->declare_parameter(location_prefix + "recharge_available", std::vector<std::string>());
    this->declare_parameter(location_prefix + "resupply_available", std::vector<std::string>());
    this->declare_parameter(location_prefix + "landing_available"); // Fail if not declared in config

    std::string pos_ne_prefix = location_prefix + "pos_ne.";
    for(std::string loc_name : locations_names)
    {
      this->declare_parameter(pos_ne_prefix + loc_name);      
    }
    this->declare_parameter(location_prefix + "location_radius_m"); // Fail if not declared in config

    /**
     * Declare parameters for mission init
     */ 
    std::string mission_init_prefix = "mission_init.";
    this->declare_parameter(mission_init_prefix + "start_location", std::string());
    this->declare_parameter(mission_init_prefix + "locations_available", std::vector<std::string>());

    std::string payload_prefix = mission_init_prefix + "payload.";
    this->declare_parameter(payload_prefix + "num_markers", int());
    this->declare_parameter(payload_prefix + "num_lifevests", int());

    /**
     * Declare parameters for mission goals
     */ 
    std::string mission_goal_prefix = "mission_goals.";
    this->declare_parameter(mission_goal_prefix + "locations_to_search", std::vector<std::string>());
    this->declare_parameter(mission_goal_prefix + "landing_desired", true); // Assume the drone should land by default
    this->declare_parameter(mission_goal_prefix + "preferred_landing_location", std::string());
    this->declare_parameter(mission_goal_prefix + "possible_landing_locations", std::vector<std::string>());


    /**
     * Other parameters
     */
    std::string search_prefix = "search.";
    this->declare_parameter(search_prefix + "distance"); // Fail if declared in config


    // Create publishers
    plan_pub_ = this->create_publisher<plansys2_msgs::msg::Plan>("/mission_controller/plansys2_plan", 1);
    planning_status_pub_ = this->create_publisher<std_msgs::msg::String>("/mission_controller/planning_status", 1);
    // planning_status_pub_ = this->create_publisher<anafi_uav_interfaces::msg::StampedString>("/mission_controller/planning_status", 1);

    // Create subscribers
    using namespace std::placeholders;
    anafi_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/anafi/state", 10, std::bind(&MissionControllerNode::anafi_state_cb_, this, _1));   
    battery_charge_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/anafi/battery", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::battery_charge_cb_, this, _1)); 
    gnss_data_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/anafi/gnss_location", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::gnss_data_cb_, this, _1));
    attitude_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/anafi/attitude", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::attitude_cb_, this, _1));   
    polled_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/anafi/polled_body_velocities", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::polled_vel_cb_, this, _1));  
    ned_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/anafi/ned_pos_from_gnss", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::ned_pos_cb_, this, _1));   
    detected_person_sub_ = this->create_subscription<anafi_uav_interfaces::msg::DetectedPerson>(
      "estimate/person_detected", rclcpp::QoS(1).best_effort(), std::bind(&MissionControllerNode::detected_person_cb_, this, _1));

    // Create services
    set_num_markers_srv_ = this->create_service<anafi_uav_interfaces::srv::SetEquipmentNumbers>(
      "/mission_controller/num_markers", std::bind(&MissionControllerNode::set_num_markers_srv_cb_, this, _1, _2)); 
    set_num_lifevests_srv_ = this->create_service<anafi_uav_interfaces::srv::SetEquipmentNumbers>(
      "/mission_controller/num_lifevests", std::bind(&MissionControllerNode::set_num_lifevests_srv_cb_, this, _1, _2)); 
    set_finished_action_srv_ = this->create_service<anafi_uav_interfaces::srv::SetFinishedAction>(
      "/mission_controller/finished_action", std::bind(&MissionControllerNode::set_finished_action_srv_cb_, this, _1, _2)); \
  }


  /**
   * @brief Initializes the domain expert, problem expert, planner, executor
   * and initial knowledge
   */
  void init();


  /**
   * @brief Steps through the problem, depending on the state
   */
  void step();

private:
  // System state 
  ControllerState controller_state_;

  int num_markers_;
  int num_lifevests_;

  double battery_charge_;
  std::string anafi_state_;
  
  geometry_msgs::msg::QuaternionStamped attitude_;
  geometry_msgs::msg::TwistStamped polled_vel_;
  geometry_msgs::msg::PointStamped position_ned_;
  std::map<std::string, geometry_msgs::msg::PointStamped> locations_;

  // Data for replanning
  std::string previous_plan_str_; 

  bool is_replanning_necessary_;
  bool is_emergency_;
  bool is_low_battery_;
  bool is_person_detected_;

  const std::vector<std::string> possible_anafi_states_ = 
    { "FS_LANDED", "FS_MOTOR_RAMPING", "FS_TAKINGOFF", "FS_HOVERING", "FS_FLYING", "FS_LANDING", "FS_EMERGENCY" };

  const std::vector<std::string> key_action_names_ = { "search", "rescue", "mark", "communicate" };

  // Mission variables
  MissionGoals mission_goals_;

  std::map<int, std::tuple<geometry_msgs::msg::Point, Severity, bool>> detected_people_; // Each person given an ID
  std::vector<std::string> inaccessible_areas_{ };  // Assumed empty at start 

  // PlanSys2
  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  // Publishers
  rclcpp::Publisher<plansys2_msgs::msg::Plan>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
  // rclcpp::Publisher<anafi_uav_interfaces::msg::StampedString>::SharedPtr planning_status_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr anafi_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::ConstSharedPtr battery_charge_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::ConstSharedPtr gnss_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr ned_pos_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::ConstSharedPtr attitude_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::ConstSharedPtr polled_vel_sub_;
  rclcpp::Subscription<anafi_uav_interfaces::msg::DetectedPerson>::ConstSharedPtr detected_person_sub_;

  // Services
  rclcpp::Service<anafi_uav_interfaces::srv::SetEquipmentNumbers>::SharedPtr set_num_markers_srv_;
  rclcpp::Service<anafi_uav_interfaces::srv::SetEquipmentNumbers>::SharedPtr set_num_lifevests_srv_;
  rclcpp::Service<anafi_uav_interfaces::srv::SetFinishedAction>::SharedPtr set_finished_action_srv_;


  // Private functions
  /**
   * @brief Checks preconditions for the entire mission to start. This includes:
   *  - information about the Anafi's state (! "")
   *  - information about the battery charge (>= 0)
   *  - ned position properly initialized (to roughly 0s). The Olympe bridge can 
   *    produce ned-positions of several 1000s if initialized too early 
   *       
   */
  void check_controller_preconditions_(); 


  /**
   * @brief Initializes the mission goals from the mission config file
   */
  void init_mission_goals_();


  /**
   * @brief Initializes the world knowledge 
   */
  void init_knowledge_();


  /**
   * @brief Updates all plansys2::Function with the recent values. This includes:
   *  - current battery percentage
   *  - number of markers available
   *  - number of lifevests available
   */
  bool update_plansys2_functions_();


  /**
   * @brief Get recommended goals based on the next state
   * 
   * Future improvement to support relaxing some goals, if the system is incapable of 
   * finding a solution to goals. Would require some form of determining critical vs
   * noncritical goals, and getting feedback from the planner. If the planner is not
   * able to immideately determine whether the set of goals are incompatible, one might
   * risk the planner taking too long to find a plan
   */
  bool update_plansys2_goals_(const ControllerState& state);//, bool relax_noncritical_goals = false);


  /**
   * @brief Implementations of acquiring mission goals
   *        _move_mission_      : Final position and drone landed
   *        _search_mission_    : Areas to search
   *        _rescue_mission_    : Try to rescue people in danger 
   *        _emergency_         : Save the drone, making it land on a landable position (not implemented)
   *        _area_unavailable_  : Force the drone to replan. Currently not implemented (not implemented)
   * 
   * @return Boolean indicating success or failure
   */
  bool load_move_mission_goals_(std::vector<std::string>& goal_vec_ref);
  bool load_search_mission_goals_(std::vector<std::string>& goal_vec_ref);
  bool load_rescue_mission_goals_(std::vector<std::string>& goal_vec_ref);
  bool load_emergency_mission_goals_(std::vector<std::string>& goal_vec_ref);
  bool load_area_unavailable_mission_goals_(std::vector<std::string>& goal_vec_ref);

  /**
   * @brief Get number of mission-critical goals remaining. This includes all of the following
   * goal types:
   *      _search_
   *      _communicate_
   *      _mark_
   *      _rescue_
   * 
   * It does currently not consider positional goals, however that is a possible future extension.
   */
  size_t get_num_remaining_mission_goals_();


  /**
   * @brief Checks whether one of the final states are achieved. This includes whether the landing
   * state matches the desired landing state, and whether the location matches the desired location 
   * Example: Landed on location h0
   * 
   * @warning Will not take into account whether an emergency has occured
   */
  bool check_desired_final_state_achieved_();



  /** 
   * @brief Based on the current information and state, checks if a replanning
   * is necessary
   * 
   * Replanning if necessary 
   */
  const std::tuple<ControllerState, bool> recommend_replan_(); 
  bool replan_mission_(std::optional<plansys2_msgs::msg::Plan>& plan); 


  /**
   * @brief Checks if the entire plan is completed 
   */
  bool check_plan_completed_();  

  /**
   * @brief Checks whether the current goals  
   */
  bool check_current_goals_satisfied_(const ControllerState& state);


  /**
   * @brief Based on the predetermined locations in the config file, it calculates
   * which location a point is at. 
   * 
   * @warning If multiple locations have overlapping areas, it returns the location which 
   * the drone is closest to the center of  
   * 
   * @warning This function does not take area availability into account
   */
  std::string get_location_(const geometry_msgs::msg::Point& point);


  /**
   * @brief Acquiring data from the map based on a set of criteria
   *        _unhelped_people_         : Get all unhelped people
   *        _people_within_radius_of_ : Get all people within a radius of a position, both rescued and not rescued
   */
  std::vector<int> get_unhelped_people_();
  std::vector<int> get_people_within_radius_of_(const geometry_msgs::msg::Point& point, double radius);


  /**
   * @brief Output data to either terminal or via publishers.
   * 
   * Logging-functions output information using RCLCPP_LOGLEVEL. Some may call publish-functions
   *  log_planning_():      Logs the state of the system when a replanning is triggered
   *  log_plan_():          Logs the new plan
   *  log_action_error_():  Logs error during execution of an action
   * 
   * Print-functions output information using std::cout to the terminal. Warning: spam
   * 
   * Publish-functions publishes data using ROS2-publishers 
   */
  void log_planning_state_();
  void log_plan_(const std::optional<plansys2_msgs::msg::Plan>& plan);
  void log_action_error_();

  void print_action_feedback_();

  void publish_plan_status_str_(const std::string& str);
  void publish_plansys2_plan_(const std::optional<plansys2_msgs::msg::Plan>& plan);


  // Callbacks
  void anafi_state_cb_(std_msgs::msg::String::SharedPtr state_msg);
  void ned_pos_cb_(geometry_msgs::msg::PointStamped::ConstSharedPtr ned_pos_msg);
  void gnss_data_cb_(sensor_msgs::msg::NavSatFix::ConstSharedPtr gnss_data_msg);
  void attitude_cb_(geometry_msgs::msg::QuaternionStamped::ConstSharedPtr attitude_msg);
  void polled_vel_cb_(geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);
  void battery_charge_cb_(std_msgs::msg::Float64::ConstSharedPtr battery_msg);
  void detected_person_cb_(anafi_uav_interfaces::msg::DetectedPerson::ConstSharedPtr detected_person_msg);

  void set_num_markers_srv_cb_(
    const std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Request> request,
    std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Response> response
  );
  void set_num_lifevests_srv_cb_(
    const std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Request> request,
    std::shared_ptr<anafi_uav_interfaces::srv::SetEquipmentNumbers::Response> response
  );
  void set_finished_action_srv_cb_(
    const std::shared_ptr<anafi_uav_interfaces::srv::SetFinishedAction::Request> request,
    std::shared_ptr<anafi_uav_interfaces::srv::SetFinishedAction::Response> response
  );

}; // MissionControllerNode


/**
 * @todo
 *  5. Method for emergency or normal operations, where it will search through a set
 *    of multiple landing locations until it finds one where it is safe to land
 */