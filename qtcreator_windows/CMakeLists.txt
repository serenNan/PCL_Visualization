cmake_minimum_required(VERSION 3.21)

project(PCL_Visualization_QtCreator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(MSVC)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
    add_compile_options(/W4 /permissive- /EHsc)
else()
    add_compile_options(-Wall -Wextra)
endif()

if(WIN32 AND NOT DEFINED Qt6_DIR)
    set(Qt6_DIR "" CACHE PATH "Path to the Qt6 CMake directory (e.g. C:/Qt/6.5.3/msvc2019_64/lib/cmake/Qt6)")
endif()
if(WIN32 AND Qt6_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${Qt6_DIR}")
endif()

if(WIN32 AND NOT DEFINED VTK_DIR)
    set(VTK_DIR "" CACHE PATH "Path to the VTK CMake configuration directory (e.g. C:/VTK/lib/cmake/vtk-9.2)")
endif()
if(WIN32 AND VTK_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${VTK_DIR}")
endif()

if(WIN32 AND NOT DEFINED PCL_DIR)
    set(PCL_DIR "" CACHE PATH "Path to the PCL CMake configuration directory (e.g. C:/PCL/lib/cmake/pcl)")
endif()
if(WIN32 AND PCL_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${PCL_DIR}")
endif()

find_package(PCL 1.8 REQUIRED)
find_package(Threads REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGLWidgets OpenGL Gui)
find_package(VTK REQUIRED)

if(VTK_QT_VERSION VERSION_GREATER "4")
    find_package(VTK REQUIRED COMPONENTS GUISupportQt)
endif()

set(SRC_ROOT ${CMAKE_CURRENT_LIST_DIR}/../src)

set(CORE_SOURCES
    ${SRC_ROOT}/core/PointCloud.cpp
    ${SRC_ROOT}/core/PointCloudLoader.cpp
)
set(CORE_HEADERS
    ${SRC_ROOT}/core/PointCloud.h
    ${SRC_ROOT}/core/PointCloudLoader.h
)

set(VISUALIZATION_SOURCES
    ${SRC_ROOT}/visualization/Visualizer.cpp
)
set(VISUALIZATION_HEADERS
    ${SRC_ROOT}/visualization/Visualizer.h
)

set(ANALYSIS_SOURCES
    ${SRC_ROOT}/analysis/AnalysisResult.cpp
    ${SRC_ROOT}/analysis/GeometryCalculator.cpp
    ${SRC_ROOT}/analysis/PotholeDetector.cpp
)
set(ANALYSIS_HEADERS
    ${SRC_ROOT}/analysis/AnalysisResult.h
    ${SRC_ROOT}/analysis/GeometryCalculator.h
    ${SRC_ROOT}/analysis/PotholeDetector.h
)

set(UI_SOURCES
    ${SRC_ROOT}/ui/Camera3D.cpp
    ${SRC_ROOT}/ui/ControlPanel.cpp
    ${SRC_ROOT}/ui/MainWindow.cpp
    ${SRC_ROOT}/ui/ResultPanel.cpp
    ${SRC_ROOT}/ui/ThemeManager.cpp
    ${SRC_ROOT}/ui/VisualizerWidget.cpp
)
set(UI_HEADERS
    ${SRC_ROOT}/ui/Camera3D.h
    ${SRC_ROOT}/ui/ControlPanel.h
    ${SRC_ROOT}/ui/MainWindow.h
    ${SRC_ROOT}/ui/ResultPanel.h
    ${SRC_ROOT}/ui/ThemeManager.h
    ${SRC_ROOT}/ui/VisualizerWidget.h
)

set(MAIN_SOURCES
    ${SRC_ROOT}/main.cpp
)

set(CLI_SOURCES
    ${SRC_ROOT}/analyze_cli.cpp
)

add_library(pcl_viz_core STATIC ${CORE_SOURCES} ${CORE_HEADERS})
target_include_directories(pcl_viz_core
    PUBLIC
        ${SRC_ROOT}
        ${SRC_ROOT}/core
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcl_viz_core PUBLIC ${PCL_LIBRARIES})

add_library(pcl_viz_visualization STATIC ${VISUALIZATION_SOURCES} ${VISUALIZATION_HEADERS})
target_include_directories(pcl_viz_visualization
    PUBLIC
        ${SRC_ROOT}
        ${SRC_ROOT}/visualization
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcl_viz_visualization
    PUBLIC
        pcl_viz_core
        ${PCL_LIBRARIES}
)

add_library(pcl_viz_analysis STATIC ${ANALYSIS_SOURCES} ${ANALYSIS_HEADERS})
target_include_directories(pcl_viz_analysis
    PUBLIC
        ${SRC_ROOT}
        ${SRC_ROOT}/analysis
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(pcl_viz_analysis
    PUBLIC
        pcl_viz_core
        ${PCL_LIBRARIES}
)

add_library(pcl_viz_ui STATIC ${UI_SOURCES} ${UI_HEADERS})
target_include_directories(pcl_viz_ui
    PUBLIC
        ${SRC_ROOT}
        ${SRC_ROOT}/ui
        ${PCL_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
)
target_link_libraries(pcl_viz_ui
    PUBLIC
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

add_executable(pcl_viewer_gui ${MAIN_SOURCES})
target_include_directories(pcl_viewer_gui PRIVATE ${SRC_ROOT} ${VTK_INCLUDE_DIRS})
target_link_libraries(pcl_viewer_gui
    PRIVATE
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

add_executable(pcl_analyze_cli ${CLI_SOURCES})
target_include_directories(pcl_analyze_cli PRIVATE ${SRC_ROOT})
target_link_libraries(pcl_analyze_cli
    PRIVATE
        pcl_viz_core
        pcl_viz_analysis
        ${PCL_LIBRARIES}
        Threads::Threads
)

if(VTK_VERSION VERSION_GREATER_EQUAL "9.0.0")
    vtk_module_autoinit(
        TARGETS pcl_viewer_gui pcl_viz_ui pcl_viz_analysis
        MODULES ${VTK_LIBRARIES}
    )
endif()

set_target_properties(pcl_viewer_gui PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(pcl_analyze_cli PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
set_target_properties(pcl_viz_core pcl_viz_visualization pcl_viz_analysis pcl_viz_ui PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
)

if(MSVC)
    foreach(target pcl_viz_core pcl_viz_visualization pcl_viz_analysis pcl_viz_ui pcl_viewer_gui pcl_analyze_cli)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    endforeach()
endif()

message(STATUS "PCL_Visualization QtCreator configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  PCL Version: ${PCL_VERSION}")
message(STATUS "  Qt Core Version: ${Qt6Core_VERSION}")
message(STATUS "  VTK Version: ${VTK_VERSION}")
