cmake_minimum_required(VERSION 3.8)
project(autoware_trajectory_to_map_frame_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::control::trajectory_to_map_frame_converter::TrajectoryToMapFrameConverterNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

autoware_ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
