cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# Build hardware interface library
add_library(motor_test_hardware_interface SHARED 
  src/diffdrive.cpp
)

target_include_directories(motor_test_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(motor_test_hardware_interface
  rclcpp
  hardware_interface
  pluginlib  
  rclcpp_lifecycle
  std_msgs
)

# Install hardware interface library
install(TARGETS motor_test_hardware_interface
  DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description
install(FILES hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(motor_test_hardware_interface)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()