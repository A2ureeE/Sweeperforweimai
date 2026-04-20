cmake_minimum_required(VERSION 3.8)
project(sweeper_gazebo)
find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/moving_obstacle_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME moving_obstacle_node
)

ament_package()
