# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 PCL (Point Cloud Library) 的路面坑洞检测和分析系统。项目采用模块化架构，分为 C++ 可视化引擎和 Python 分析模块。当前已完成基础的点云加载和可视化功能，需要继续开发 GUI 界面和凹坑检测算法。

## 核心功能需求

根据 `task.md`，软件需要实现：
- 3D 点云可视化显示
- 凹坑检测（深度、尺寸计算）
- 几何测量（体积、面积计算）
- 实时时间信息显示
- GUI 界面集成（计划使用 Qt）

## 常用命令

### 构建和运行
```bash
# 创建构建目录并编译
cd src
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行可视化器
./pcl_viewer ../../data/H103v2.asc

# Python 分析模块
cd python
python analyze_pointcloud.py
```

### WSL 环境特殊处理
```bash
# 如果遇到 VTK 库问题
vtk_env  # 加载 VTK 环境

# 如果遇到显示问题
export DISPLAY=:0
```

## 当前架构

### 技术栈
- **C++17** + **PCL 1.8+** - 核心点云处理引擎
- **VTK** - 3D 渲染后端（通过 PCL）
- **Python3** + **NumPy/Matplotlib/SciPy** - 数据分析和原型验证
- **CMake 3.16+** - 构建系统
- **Qt5/6**（待集成）- GUI 框架

### 已实现模块

#### core/ - 核心数据处理
- `PointCloud` - 点云数据封装类（RAII，智能指针管理）
- `PointCloudLoader` - ASC/WRP 文件加载器
- 边界框计算和统计信息管理

#### visualization/ - 可视化
- `Visualizer` - PCL 可视化器封装
- 交互式 3D 查看（鼠标控制，键盘快捷键）
- 坐标轴和网格显示

#### python/ - 分析原型
- `analyze_pointcloud.py` - 凹坑检测算法原型
- 使用 ConvexHull 进行体积计算
- Z 轴阈值法检测凹陷区域

### 待实现模块（按优先级）

1. **analysis/** - 凹坑检测算法（高优先级）
   - `PotholeDetector` - 将 Python 原型移植到 C++
   - `GeometryCalculator` - 体积/面积/深度计算
   - RANSAC 平面拟合算法

2. **ui/** - Qt GUI 界面（中优先级）
   - `MainWindow` - 主窗口框架
   - `ResultPanel` - 测量结果显示面板
   - `ControlPanel` - 参数调节界面
   - QVTKWidget 集成用于 3D 显示

3. **io/** - 扩展文件格式支持（低优先级）
   - PCD, PLY, LAS 格式支持
   - 结果导出功能（CSV, JSON）

## 代码架构说明

### 设计模式
- **RAII** - 所有资源通过智能指针管理
- **模块化** - 功能分离到独立的静态库
- **命名空间** - `pcl_viz::core`, `pcl_viz::visualization`

### CMake 结构
- 主 CMakeLists.txt 在 `src/` 目录
- 静态库：`pcl_viz_core`, `pcl_viz_visualization`
- 可执行文件：`pcl_viewer`
- 已启用 `CMAKE_EXPORT_COMPILE_COMMANDS`

## 数据文件说明

### H103v2.asc
- 740 个点，格式：`X Y Z Normal_X Normal_Y Normal_Z`
- 坐标范围：X[-34.8, -33.6], Y[5.5, 6.6], Z[12.2, 12.5]
- 表示路面扫描数据，Z 轴变化约 0.33m（可能的凹坑深度）

### H103v2.wrp
- 二进制格式，88KB，可能包含网格或处理后数据
- 当前未实现解析（需要格式规范）

## 下一步开发计划

1. **将 Python 凹坑检测算法移植到 C++**
   - 实现 Z 轴阈值检测
   - 添加 RANSAC 平面拟合
   - 计算体积和面积

2. **集成 Qt GUI**
   - 添加 Qt5 到 CMakeLists.txt
   - 创建 MainWindow 框架
   - 集成 QVTKWidget

3. **完善测量功能**
   - 实时显示检测结果
   - 添加参数调节滑块
   - 导出测量报告

## 注意事项

- WSL 环境需要先运行 `vtk_env` 加载图形环境
- 显示问题使用 `export DISPLAY=:0` 解决
- 不要手动编译，用户会自行处理构建过程