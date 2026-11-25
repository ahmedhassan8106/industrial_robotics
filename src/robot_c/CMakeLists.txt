cmake_minimum_required(VERSION 3.8)
project(robot_c)

# Compiler options
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 build system
find_package(ament_cmake REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED IMPORTED_TARGET gtk+-3.0)
# C programming dependencies
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# Install robot description and simulation files
install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    worlds
  DESTINATION share/${PROJECT_NAME}
)
# C joint controller
add_executable(
  slider_to_gazebo_c
  src/slider_to_gazebo.c
)

ament_target_dependencies(
  slider_to_gazebo_c
  rcl
  rclc
  sensor_msgs
  std_msgs
  trajectory_msgs
)

install(
  TARGETS slider_to_gazebo_c
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(
    gazebo_slider_panel_c
    src/gazebo_slider_panel.c
)

ament_target_dependencies(
    gazebo_slider_panel_c
    rcl
    rclc
    sensor_msgs
)

target_link_libraries(
    gazebo_slider_panel_c
    PkgConfig::GTK3
    m
)

install(
    TARGETS gazebo_slider_panel_c
    DESTINATION lib/${PROJECT_NAME}
)
#  C Publisher Node
add_executable(
  talker_node
  src/talker_node.c
)

ament_target_dependencies(
  talker_node
  rcl
  rclc
  std_msgs
)

#  C Subscriber Node
add_executable(
  listener_node
  src/listener_node.c
)

ament_target_dependencies(
  listener_node
  rcl
  rclc
  std_msgs
)


install(
  TARGETS
    talker_node
    listener_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

