#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Get the launch directory
  package_name = "automated_planning"
  directory = get_package_share_directory(package_name)
  namespace = LaunchConfiguration('namespace')

  pddl_file = "sar.pddl"

  declare_namespace_cmd = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Namespace')

  stdout_linebuf_envvar = SetEnvironmentVariable(
    'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

  plansys2_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('plansys2_bringup'),
      'launch',
      'plansys2_bringup_launch_monolithic.py')),
    launch_arguments={
      'model_file': directory + "/pddl/" + pddl_file,
      'namespace': namespace
    }.items()
  )

  config_file = os.path.join(
      get_package_share_directory(package_name),
      'config',
      'config.yaml'
    )
  mission_params_file = os.path.join(
      get_package_share_directory(package_name),
      'config',
      'mission_parameters.yaml'
    )

  # Specify the actions
  move_cmd = Node(
    package=package_name,
    executable='move_action_node',
    name='move_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])

  land_cmd = Node(
    package=package_name,
    executable='land_action_node',
    name='land_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])

  takeoff_cmd = Node(
    package=package_name,
    executable='takeoff_action_node',
    name='takeoff_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])   
  
  drop_lifevest_cmd = Node(
    package=package_name,
    executable='drop_lifevest_action_node',
    name='drop_lifevest_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])   
  
  drop_marker_cmd = Node(
    package=package_name,
    executable='drop_marker_action_node',
    name='drop_marker_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])   
  
  communicate_cmd = Node(
    package=package_name,
    executable='communicate_action_node',
    name='communicate_action_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file, mission_params_file])   
  
  # search_cmd = Node(
  #   package=package_name,
  #   executable='search_action_node',
  #   name='search_action_node',
  #   namespace=namespace,
  #   output='screen',
  #   parameters=[config_file, mission_params_file])   
  
  get_search_positions = Node(
    package="waypoints_generator",
    executable='generate_search_waypoints_node',
    name='search_waypoints_node',
    namespace=namespace,
    output='screen',
    parameters=[config_file])   

  ld = LaunchDescription()

  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(declare_namespace_cmd)

  # Declare launch options
  ld.add_action(plansys2_cmd)

  ld.add_action(move_cmd)
  ld.add_action(land_cmd)
  ld.add_action(takeoff_cmd)
  ld.add_action(drop_lifevest_cmd)
  ld.add_action(drop_marker_cmd)
  ld.add_action(communicate_cmd)
  # ld.add_action(search_cmd)

  ld.add_action(get_search_positions)

  return ld
