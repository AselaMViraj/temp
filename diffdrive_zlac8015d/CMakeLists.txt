cmake_minimum_required(VERSION 3.5)
project(diffdrive_zlac8015d)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Find libmodbus system library
find_library(MODBUS_LIBRARY modbus REQUIRED)
find_path(MODBUS_INCLUDE_DIR modbus/modbus.h)

add_library(diffdrive_zlac8015d SHARED
  src/diffdrive_zlac8015d.cpp
  src/wheel.cpp
  src/zlac_comms.cpp
)

target_include_directories(
  diffdrive_zlac8015d
  PRIVATE
  include
  ${MODBUS_INCLUDE_DIR}
)

ament_target_dependencies(
  diffdrive_zlac8015d
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

target_link_libraries(diffdrive_zlac8015d ${MODBUS_LIBRARY})

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS diffdrive_zlac8015d
  DESTINATION lib
)

ament_export_libraries(
  diffdrive_zlac8015d
)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()
