cmake_minimum_required(VERSION 3.8)
project(dijkstra_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

# Include directories
include_directories(include)

# Create executable
add_executable(dijkstra_planner_node
  src/dijkstra_planner.cpp
)

# Specify dependencies
ament_target_dependencies(dijkstra_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# Install targets
install(TARGETS
  dijkstra_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()