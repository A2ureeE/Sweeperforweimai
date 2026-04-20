cmake_minimum_required(VERSION 3.8)
project(lidar_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(pc_to_scan_node src/pc_to_scan_node.cpp)

ament_target_dependencies(pc_to_scan_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros)

target_compile_features(pc_to_scan_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS pc_to_scan_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
