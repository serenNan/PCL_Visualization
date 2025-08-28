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

# Qt6 相关设置 - 支持QVTKOpenGLNativeWidget
find_package(Qt6 REQUIRED COMPONENTS 
    Core 
    Widgets 
    OpenGLWidgets 
    OpenGL
    Gui
)

# 设置Qt自动处理工具
set(CMAKE_AUTOMOC ON)  # 自动处理MOC
set(CMAKE_AUTORCC ON)  # 自动处理RCC
set(CMAKE_AUTOUIC ON)  # 自动处理UIC

# VTK 相关设置 - 使用PCL已经找到的VTK，支持Qt集成
find_package(VTK REQUIRED)

# 检查VTK是否包含Qt支持
if(VTK_QT_VERSION VERSION_GREATER "4")
    find_package(VTK REQUIRED COMPONENTS GUISupportQt)
    message(STATUS "VTK Qt support found: ${VTK_QT_VERSION}")
endif()

message(STATUS "VTK_VERSION: ${VTK_VERSION}")
message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")
message(STATUS "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")

# VTK 9.x 模块自动初始化 - 正确的目标名称将在后面设置
if(VTK_VERSION VERSION_GREATER_EQUAL "9.0.0")
    # VTK 9.x 不需要 include(${VTK_USE_FILE})
    message(STATUS "Using VTK ${VTK_VERSION}")
else()
    # VTK 8.x的向后兼容支持
    include(${VTK_USE_FILE})
endif()

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

set(ANALYSIS_SOURCES
    analysis/AnalysisResult.cpp
    analysis/PotholeDetector.cpp
    analysis/GeometryCalculator.cpp
)

set(UI_SOURCES
    ui/MainWindow.cpp
    ui/VisualizerWidget.cpp
    ui/Camera3D.cpp
    ui/ResultPanel.cpp
    ui/ThemeManager.cpp
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

# 创建分析库
add_library(pcl_viz_analysis STATIC ${ANALYSIS_SOURCES})
target_link_libraries(pcl_viz_analysis 
    pcl_viz_core
    ${PCL_LIBRARIES}
)

# 创建UI库
add_library(pcl_viz_ui STATIC ${UI_SOURCES})
target_link_libraries(pcl_viz_ui 
    pcl_viz_core
    pcl_viz_visualization
    pcl_viz_analysis
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    Qt6::Gui
)

# 为UI库添加VTK包含目录
target_include_directories(pcl_viz_ui PRIVATE
    ${VTK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 创建主可执行文件（GUI版本）
add_executable(pcl_viewer_gui ${MAIN_SOURCES})

# 链接所有必要的库
target_link_libraries(pcl_viewer_gui 
    pcl_viz_core
    pcl_viz_visualization
    pcl_viz_analysis
    pcl_viz_ui
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    Qt6::Gui
    Threads::Threads
)

# 为主程序添加包含目录
target_include_directories(pcl_viewer_gui PRIVATE
    ${VTK_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# VTK 9.x 模块自动初始化（必须在目标创建后）
if(VTK_VERSION VERSION_GREATER_EQUAL "9.0.0")
    vtk_module_autoinit(
        TARGETS pcl_viewer_gui pcl_viz_ui pcl_viz_analysis
        MODULES ${VTK_LIBRARIES}
    )
endif()

# 创建测试程序
add_executable(test_analysis_debug build/test_analysis.cpp)
target_link_libraries(test_analysis_debug 
    pcl_viz_core
    pcl_viz_analysis
    ${PCL_LIBRARIES}
)
target_include_directories(test_analysis_debug PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 设置输出目录
set_target_properties(pcl_viewer_gui test_analysis_debug PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

set_target_properties(pcl_viz_core pcl_viz_visualization pcl_viz_analysis pcl_viz_ui PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

# 安装规则（可选）
install(TARGETS pcl_viewer_gui
    RUNTIME DESTINATION bin
)

install(TARGETS pcl_viz_core pcl_viz_visualization pcl_viz_analysis pcl_viz_ui
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