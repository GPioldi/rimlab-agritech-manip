cmake_minimum_required(VERSION 3.8)
project(agritech_manip)
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# find_package(ur_ros_driver REQUIRED)
# find_package(ur_ros_driver_msgs REQUIRED)
# find_package(ur10e_planning REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(ur_ros_rtde_simple_clients REQUIRED)
find_package(ur_ros_rtde_msgs REQUIRED)
find_package(ur_ros_rtde_gripper_commands REQUIRED)
find_package(moveit_planning REQUIRED)
find_package(tomato_detection REQUIRED)
find_package(tomato_msg REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# List of packages used by most of our nodes
set(mydependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  moveit_planning
  ur_ros_rtde_msgs
  ur_ros_rtde_simple_clients
  ur_ros_rtde_gripper_commands
  tomato_detection
  tomato_msg
)

message(STATUS "ROS dependencies: " ${mydependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Nodes for
# add_executable(tomato_pick src/tomato_pick.cpp src/TomatoPickNode.cpp)
add_executable(tomato_pick
  src/tomato_pick.cpp
  src/tomato_pick_node.cpp
  src/static_object_tracker.cpp
  src/robot_dispatcher.cpp
  src/transform.cpp)
ament_target_dependencies(tomato_pick ${mydependencies})
target_link_libraries(tomato_pick ${Eigen3_LIBRARIES} moveit_planning::moveit_planning_lib) # ur10e_planning::ur10e_planning_lib

# Testing outside ROS nodes
add_executable(test_static_object_tracker src/test_static_object_tracker.cpp src/static_object_tracker.cpp)

# ament_target_dependencies(test_static_object_tracker ${mydependencies})
target_link_libraries(test_static_object_tracker ${Eigen3_LIBRARIES})

install(TARGETS
  tomato_pick
  test_static_object_tracker
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  scene_geometry
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
