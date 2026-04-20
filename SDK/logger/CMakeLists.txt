cmake_minimum_required(VERSION 3.8)
project(logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ======== ROS2 dependencies ========
find_package(ament_cmake REQUIRED)

# ======== Build library ========
add_library(logger SHARED
  src/logger.cpp
)

# ======== Include directories ========
target_include_directories(logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ======== Link libraries ========
target_link_libraries(logger
  pthread
)

# ======== Install library ========
install(TARGETS logger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ======== Install headers ========
install(DIRECTORY include/ DESTINATION include)

# ======== Export targets ========
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)

ament_package()
