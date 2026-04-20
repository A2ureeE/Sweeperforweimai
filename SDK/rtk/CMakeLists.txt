cmake_minimum_required(VERSION 3.5)
project(rtk)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sweeper_interfaces REQUIRED)
find_package(logger REQUIRED)
#find_package(Boost 1.71.0 REQUIRED COMPONENTS thread atomic system regex)

#if(Boost_FOUND)
#include_directories(${Boost_INCLUDE_DIRS})
#endif()

include_directories(include/rtk ${catkin_INCLUDE_DIRS})

add_executable(rtk_node src/rtk_node.cpp src/serial_read.cpp)
target_link_libraries(rtk_node ${Boost_LIBRARIES})
ament_target_dependencies(
  rtk_node
  rclcpp
  std_msgs
  sweeper_interfaces
  logger)

install(TARGETS rtk_node DESTINATION lib/${PROJECT_NAME})

# 安装配置文件和启动文件
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
