cmake_minimum_required(VERSION 3.14)
project(CILQR_Sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- [核心修改] 强制开启 Release 模式和 O3 优化 ---
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 针对 GCC/Clang 编译器显式添加优化标志
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-O3 -DNDEBUG -march=native -ffast-math)
    # add_compile_options(-O3 -DNDEBUG -march=native -fopenmp)
    message(STATUS "Optimization flags enabled: -O3 -DNDEBUG -march=native")
endif()
# --------------------------------------------------
find_package(OpenMP REQUIRED) # [新增]

find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

# 包含 include 目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# 如果 Eigen3 没有自动包含，通常也需要这一行（视系统而定，保险起见）
include_directories(${EIGEN3_INCLUDE_DIR})

# 自动查找 src 下的所有 cpp 文件
file(GLOB_RECURSE SRC_LIST CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    Python3::Python
    Python3::NumPy
    OpenMP::OpenMP_CXX
)