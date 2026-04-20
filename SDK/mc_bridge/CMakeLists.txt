cmake_minimum_required(VERSION 3.8)
project(mc_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sweeper_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mc_bridge_node src/mc_bridge_node.cpp)

ament_target_dependencies(mc_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
  sweeper_interfaces
  sensor_msgs)

target_compile_features(mc_bridge_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS mc_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
