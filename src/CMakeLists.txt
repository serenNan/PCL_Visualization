cmake_minimum_required(VERSION 3.16)
project(PCL_Visualization VERSION 1.0.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")

# 查找必要的库
find_package(PCL 1.8 REQUIRED)
find_package(Threads REQUIRED)

# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# 链接库目录
link_directories(${PCL_LIBRARY_DIRS})

# 添加编译定义
add_definitions(${PCL_DEFINITIONS})

# 收集源文件
set(CORE_SOURCES
    core/PointCloudLoader.cpp
    core/PointCloud.cpp
)

set(VISUALIZATION_SOURCES
    visualization/Visualizer.cpp
)

set(MAIN_SOURCES
    main.cpp
)

# 创建核心库
add_library(pcl_viz_core STATIC ${CORE_SOURCES})
target_link_libraries(pcl_viz_core ${PCL_LIBRARIES})

# 创建可视化库
add_library(pcl_viz_visualization STATIC ${VISUALIZATION_SOURCES})
target_link_libraries(pcl_viz_visualization 
    pcl_viz_core
    ${PCL_LIBRARIES}
)

# 创建主可执行文件
add_executable(pcl_viewer ${MAIN_SOURCES})

# 链接所有必要的库
target_link_libraries(pcl_viewer 
    pcl_viz_core
    pcl_viz_visualization
    ${PCL_LIBRARIES}
    Threads::Threads
)

# 设置输出目录
set_target_properties(pcl_viewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

set_target_properties(pcl_viz_core pcl_viz_visualization PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 安装规则（可选）
install(TARGETS pcl_viewer
    RUNTIME DESTINATION bin
)

install(TARGETS pcl_viz_core pcl_viz_visualization
    ARCHIVE DESTINATION lib
)

# 打印配置信息
message(STATUS "PCL_Visualization Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  PCL Version: ${PCL_VERSION}")
message(STATUS "  Install Prefix: ${CMAKE_INSTALL_PREFIX}")

# 调试信息
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    message(STATUS "  Debug Mode: Enabled")
    message(STATUS "  Debug Flags: ${CMAKE_CXX_FLAGS_DEBUG}")
else()
    message(STATUS "  Release Mode: Enabled")
    message(STATUS "  Release Flags: ${CMAKE_CXX_FLAGS_RELEASE}")
endif()