import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  # Get the launch directory
  package_name = "automated_planning_ros2"
  directory = get_package_share_directory(package_name)
  namespace = LaunchConfiguration('namespace')

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
      'model_file': directory + '/pddl/move.pddl',
      'namespace': namespace
      }.items())

  # Specify the actions
  move_cmd = Node(
    package=package_name,
    executable='move_action_node',
    name='move_action_node',
    namespace=namespace,
    output='screen',
    parameters=[])

  land_cmd = Node(
    package=package_name,
    executable='land_action_node',
    name='land_action_node',
    namespace=namespace,
    output='screen',
    parameters=[])

  takeoff_cmd = Node(
    package=package_name,
    executable='takeoff_action_node',
    name='takeoff_action_node',
    namespace=namespace,
    output='screen',
    parameters=[])   
  ld = LaunchDescription()

  # Set environment variables
  ld.add_action(stdout_linebuf_envvar)
  ld.add_action(declare_namespace_cmd)

  # Declare the launch options
  ld.add_action(plansys2_cmd)

  ld.add_action(move_cmd)
  ld.add_action(land_cmd)
  ld.add_action(takeoff_cmd)

  return ld
