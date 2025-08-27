cmake_minimum_required(VERSION 3.16)
project(PCL_Visualization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 PCL 库
find_package(PCL 1.8 REQUIRED)

# 包含 PCL 头文件目录
include_directories(${PCL_INCLUDE_DIRS})

# 链接 PCL 库目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加 PCL 编译定义
add_definitions(${PCL_DEFINITIONS})

# 创建可执行文件
add_executable(pcl_viewer main.cpp)

# 链接 PCL 库
target_link_libraries(pcl_viewer ${PCL_LIBRARIES})