cmake_minimum_required(VERSION 3.8)
project(motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find LibSerial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Add library
add_library(
  motor_test_hardware_interface
  SHARED
  src/diffdrive.cpp
)

target_include_directories(
  motor_test_hardware_interface
  PRIVATE
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

# Link libraries - THIS IS THE CRITICAL PART
target_link_libraries(
  motor_test_hardware_interface
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(
  motor_test_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export the plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Install
install(
  TARGETS motor_test_hardware_interface
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(motor_test_hardware_interface)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()