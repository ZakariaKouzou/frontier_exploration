cmake_minimum_required(VERSION 3.8)
project(frontier_exploration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(frontier_exploration_msgs REQUIRED)

include_directories(include)

add_executable(frontier_detector_node src/frontier_detector_node.cpp)
ament_target_dependencies(frontier_detector_node
  rclcpp nav_msgs visualization_msgs geometry_msgs tf2 tf2_ros frontier_exploration_msgs)

add_executable(frontier_goal_selector_node src/frontier_goal_selector_node.cpp)
ament_target_dependencies(frontier_goal_selector_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs tf2 tf2_ros frontier_exploration_msgs)

install(TARGETS
  frontier_detector_node
  frontier_goal_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()