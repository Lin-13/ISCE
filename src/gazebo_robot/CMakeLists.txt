cmake_minimum_required(VERSION 3.5)
project(gazebo_robot)

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
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
install(
  DIRECTORY
  config
  launch
  model
  world
  DESTINATION
  share/${PROJECT_NAME}
)

add_executable(hyper_robot4_traj src/hyper_robot4_traj.cpp)
ament_target_dependencies(hyper_robot4_traj
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)
add_executable(hyper_robot4_position src/hyper_robot4_position.cpp)
ament_target_dependencies(hyper_robot4_position
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
  hyper_robot4_traj
  hyper_robot4_position
  DESTINATION
    lib/${PROJECT_NAME}
)
ament_package()
