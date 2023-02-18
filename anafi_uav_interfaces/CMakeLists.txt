cmake_minimum_required(VERSION 3.5)
project(anafi_uav_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/AttitudeCommand.msg"
  "msg/AttitudeSetpoint.msg"
  "msg/CameraCommand.msg"
  "msg/DetectedPerson.msg"
  "msg/EkfOutput.msg"
  "msg/EulerPose.msg"
  "msg/Heading.msg"
  "msg/MoveByCommand.msg"
  "msg/MoveToCommand.msg"
  "msg/PointWithCovarianceStamped.msg"
  "msg/PositionSetpointRelative.msg"
  "msg/ReferenceStates.msg"
  "msg/SaturationLimits.msg"
  "msg/PoseStampedEuler.msg"
  "msg/Float32Stamped.msg"
  "msg/SkyControllerCommand.msg"
)
set(srv_files
  "srv/SetDesiredPosition.srv"
  "srv/SetPlannedActions.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
