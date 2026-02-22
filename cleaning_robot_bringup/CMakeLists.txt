cmake_minimum_required(VERSION 3.8)
project(cleaning_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch scripts web_ui config
  DESTINATION share/${PROJECT_NAME}
)
# Install scripts to lib as well for ros2 run usage
install(PROGRAMS
  scripts/exploration_manager.py
  scripts/odom_tf_publisher.py
  scripts/waypoint_manager.py
  scripts/web_server.py
  scripts/auto_kickstart.py
  scripts/auto_localize.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
