# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于 PCL (Point Cloud Library) 的高精度坑洞检测和几何测量系统。项目采用现代化 C++17 和 Qt6 架构，专门用于分析路面扫描数据中的微小缺陷（深度 ≤0.2mm，尺寸 5mm×5mm）。系统已完成完整的 GUI 框架和分析引擎。

## 核心功能需求

根据 `task.md`，软件需要实现高精度测量：
- **3D 点云可视化** - 交互式显示和操作
- **坑洞检测算法** - 基于表面拟合的精密检测
- **几何测量** - 深度（精度0.2mm）、体积、面积、尺寸计算
- **实时结果显示** - 时间戳和测量数据面板
- **Qt6 GUI界面** - 现代化用户界面（已完成）

## 常用命令

### 构建和运行
```bash
# 标准构建流程
cd src
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# 运行 GUI 应用程序
./bin/pcl_viewer_gui                    # 启动 GUI 界面
./bin/pcl_viewer_gui ../data/H103v2.asc # 启动时加载文件

# 调试模式构建
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)

# Python 原型验证
cd ../python
python analyze_pointcloud.py
```

### WSL 图形环境
```bash
# VTK 环境初始化（必需）
vtk_env

# X11 转发配置
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
```

## 项目架构

### 技术栈
- **C++17** + **PCL 1.8+** - 点云处理核心引擎
- **Qt6** - 现代化 GUI 框架（Widgets, OpenGL, VTK 集成）
- **VTK 9.x** - 3D 渲染和可视化后端
- **CMake 3.16+** - 现代化构建系统，支持自动 MOC/UIC/RCC
- **Python3** + **NumPy/SciPy** - 算法原型验证和分析

### 模块化架构 (已完成)

#### core/ - 数据处理核心
- `PointCloud` - RAII 点云数据封装，智能指针管理
- `PointCloudLoader` - 多格式加载器（ASC, PCD, PLY, TXT）
- 边界框、统计信息和内存优化管理

#### visualization/ - 3D 渲染引擎
- `Visualizer` - PCL/VTK 可视化器高级封装
- 交互式 3D 操作（鼠标、键盘、触控）
- 坐标系、网格、标注显示

#### analysis/ - 几何分析引擎 (已完成)
- `PotholeDetector` - 高精度坑洞检测算法
- `GeometryCalculator` - 体积/面积/深度/尺寸计算
- `AnalysisResult` - 测量结果数据结构

#### ui/ - Qt6 GUI 框架 (已完成)
- `MainWindow` - 主窗口和菜单系统
- `VisualizerWidget` - QVTKOpenGLNativeWidget 3D 显示组件
- `ControlPanel` - 参数控制和工具面板
- `ResultPanel` - 实时测量结果显示
- `Camera3D` - 3D 相机控制和动画
- `ThemeManager` - 主题管理和 DPI 适配

#### python/ - 算法原型验证
- `analyze_pointcloud.py` - 几何分析算法原型
- ConvexHull 体积计算、Z 轴阈值检测
- matplotlib 可视化验证

## CMake 构建系统

### 构建目标结构
```
src/CMakeLists.txt - 主构建配置
├── pcl_viz_core (静态库) - 数据处理核心
├── pcl_viz_visualization (静态库) - 3D 渲染引擎
├── pcl_viz_analysis (静态库) - 几何分析引擎
├── pcl_viz_ui (静态库) - Qt6 GUI 组件
└── pcl_viewer_gui (可执行文件) - 主应用程序
```

### 关键 CMake 特性
- **自动化 Qt 处理** - `CMAKE_AUTOMOC/AUTORCC/AUTOUIC=ON`
- **VTK 9.x 支持** - `vtk_module_autoinit()` 自动模块初始化
- **跨平台兼容** - Qt6, PCL, VTK 依赖自动检测
- **编译命令导出** - `CMAKE_EXPORT_COMPILE_COMMANDS=ON`
- **输出目录管理** - `bin/` 和 `lib/` 分离

## 数据文件规格

### H103v2.asc (测试数据集)
- **格式**: `X Y Z Normal_X Normal_Y Normal_Z` (每行 6 个浮点数)
- **规模**: 740 个扫描点
- **坐标范围**: X[-34.8, -33.6], Y[5.5, 6.6], Z[12.2, 12.5]
- **测量精度**: Z 轴变化 ~0.33m，包含微小坑洞特征
- **用途**: 路面缺陷检测和几何测量验证

### H103v2.wrp (二进制数据)
- **格式**: 未知二进制格式 (88KB)
- **状态**: 暂未实现解析器
- **推测**: 可能包含网格或预处理后的点云数据

## 算法核心

### 高精度测量要求
- **深度检测**: 精度 ≤0.2mm，基于表面拟合算法
- **尺寸测量**: 缺陷边界识别，支持 5mm×5mm 微小区域
- **体积计算**: 基于凸包算法和表面重建
- **实时处理**: GUI 响应时间 <100ms

### Python 原型 → C++ 移植路径
1. **`analyze_pointcloud.py`** → **`PotholeDetector.cpp`**
2. **SciPy 算法** → **PCL 几何处理**
3. **matplotlib 可视化** → **VTK 3D 渲染**

## 开发指南

### 代码风格约定
- **命名空间**: `pcl_viz::core`, `pcl_viz::ui`, `pcl_viz::analysis`
- **内存管理**: RAII + 智能指针 (`std::shared_ptr`, `std::unique_ptr`)
- **错误处理**: 异常安全和资源清理
- **Qt 集成**: 信号/槽机制，事件驱动架构

### 性能优化重点
- **点云数据结构**: PCL 原生格式，避免不必要的复制
- **3D 渲染管道**: VTK 渲染优化，LOD 管理
- **并行计算**: OpenMP 和多线程几何算法
- **内存使用**: 大型点云的流式处理

## 环境配置

### WSL 开发环境
- **必须运行**: `vtk_env` (加载 VTK 图形库)
- **X11 转发**: `export DISPLAY=:0`
- **Qt 平台**: `export QT_QPA_PLATFORM=xcb`
- **OpenGL 支持**: 确保 WSL2 GPU 加速可用

### 依赖版本要求
- **PCL**: ≥1.8 (点云处理)
- **Qt**: ≥6.0 (现代 GUI 框架)
- **VTK**: ≥9.0 (3D 渲染，Qt 集成支持)
- **CMake**: ≥3.16 (现代 CMake 特性)

## 专业代理调用建议

- **主要使用**: `cpp-pro` - C++17 现代特性和 PCL 优化
- **GUI 开发**: `frontend-developer` - Qt6 界面和用户体验
- **算法优化**: `performance-engineer` - 几何计算性能调优
- **架构审查**: `architect-reviewer` - 模块化设计验证
- **代码质量**: `code-reviewer` - 代码审查和最佳实践