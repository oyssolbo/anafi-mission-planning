cmake_minimum_required(VERSION 3.5)
project(automated_planning_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# >= C++17 required
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(anafi_uav_interfaces REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  plansys2_msgs
  plansys2_executor
  geometry_msgs
  nav2_msgs
  sensor_msgs
  anafi_uav_interfaces
)

add_executable(move_action_node src/move_action_node.cpp)
ament_target_dependencies(move_action_node ${dependencies})

add_executable(land_action_node src/land_action_node.cpp)
ament_target_dependencies(land_action_node ${dependencies})

add_executable(takeoff_action_node src/takeoff_action_node.cpp)
ament_target_dependencies(takeoff_action_node ${dependencies})

add_executable(mission_controller_node src/mission_controller.cpp)
ament_target_dependencies(mission_controller_node ${dependencies})

install(DIRECTORY launch pddl DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_action_node
  land_action_node
  takeoff_action_node
  mission_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_dependencies()
ament_package()
