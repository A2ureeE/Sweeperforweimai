cmake_minimum_required(VERSION 3.8)
project(mc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(sweeper_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(logger REQUIRED)

# 搜索 src 目录下所有 .cpp 文件
file(GLOB SRC_FILES src/*.cpp)

# 创建可执行文件
add_executable(mc_node ${SRC_FILES})

ament_target_dependencies(
  mc_node
  ament_index_cpp
  rclcpp
  std_msgs
  sweeper_interfaces
  logger)

# Set include directories for the target
target_include_directories(mc_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                          $<INSTALL_INTERFACE:include>)

target_compile_features(mc_node PUBLIC c_std_99 cxx_std_17)

# Install target
install(TARGETS mc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Handle testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
