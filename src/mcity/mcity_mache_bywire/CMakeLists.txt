cmake_minimum_required(VERSION 3.14)
project(mcity_bywire)

# Find required ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mcity_msgs REQUIRED)
find_package(dbw_fca_msgs REQUIRED)
find_package(dbw_ford_msgs REQUIRED)
find_package(dataspeed_dbw_msgs REQUIRED)

# Include directories
include_directories(
    include
)

# Declare a library
add_library(mcity_bywire_component SHARED
  src/mcity_bywire.cpp
)

ament_target_dependencies(mcity_bywire_component
  rclcpp
  rclcpp_components
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  mcity_msgs
  dbw_fca_msgs
  dbw_ford_msgs
  dataspeed_dbw_msgs
)

# Register component
rclcpp_components_register_node(mcity_bywire_component
  PLUGIN "mcity_bywire::McityBywire"
  EXECUTABLE mcity_bywire
)

# Install rules for the component library
install(TARGETS
  mcity_bywire_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Must be called for ament package
ament_package()
