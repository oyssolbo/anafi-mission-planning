cmake_minimum_required(VERSION 3.5)
project(action_implementations)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(anafi_uav_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  rclcpp_lifecycle
  plansys2_msgs
  plansys2_executor
  geometry_msgs
  nav2_msgs
  sensor_msgs
  anafi_uav_interfaces
  std_msgs 
  std_srvs
  Eigen3
)

add_library(move_action_server SHARED src/move_action_server.cpp)
add_library(track_action_server SHARED src/track_action_server.cpp)

target_include_directories(move_action_server PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>)
target_include_directories(track_action_server PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(move_action_server PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
target_compile_definitions(track_action_server PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")

ament_target_dependencies(move_action_server ${dependencies})
ament_target_dependencies(track_action_server ${dependencies})

rclcpp_components_register_node(move_action_server PLUGIN "MoveActionServer" EXECUTABLE move_action_server.cpp)
rclcpp_components_register_node(track_action_server PLUGIN "TrackActionServer" EXECUTABLE track_action_server.cpp)

install(TARGETS
  move_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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

ament_package()
