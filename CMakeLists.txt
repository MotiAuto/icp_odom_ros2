cmake_minimum_required(VERSION 3.5)
project(icp_odom_ros2)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
ament_auto_find_build_dependencies()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(target_icp_odom_ros2 SHARED
  DIRECTORY src
)
target_include_directories(target_icp_odom_ros2
  SYSTEM PUBLIC
)

rclcpp_components_register_node(target_icp_odom_ros2
  PLUGIN "icp_odom_ros2::IcpOdomROS2"
  EXECUTABLE icp_odom_ros2_node
)

ament_auto_package(
  INSTALL_TO_SHARE
)