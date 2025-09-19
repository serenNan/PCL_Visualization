# Windows 平台 QtCreator 集成说明

本目录提供在 **Windows + QtCreator** 环境下构建和运行 PCL 点云可视化项目所需的独立 CMake 工程。项目源代码仍位于 `../src` 目录，功能与界面保持不变，仅增加了适配 Windows/MSVC 的构建脚本与配置指引。

## 1. 环境准备

1. 安装 Visual Studio（建议 2019 或更新版本），并勾选 **Desktop development with C++** 组件。
2. 安装 Qt 6（建议 6.5 及以上，MSVC 64-bit 版本），记下 Qt 安装目录中 `lib/cmake/Qt6` 的路径。
3. 安装 VTK（与 PCL 版本兼容，建议使用 9.x 版本，构建时开启 Qt 支持）。
4. 安装 PCL（建议 1.12 或更新版本，并确保包含 `pcl_visualization`、`pcl_io` 等模块）。
5. 将 Qt、VTK、PCL、Boost 等依赖的 `bin` 目录加入系统 `PATH`，以便运行时能够找到对应的动态库。

> 📌 **推荐做法**：将上述依赖统一安装到如 `C:/dev` 目录下，便于管理和配置。

## 2. 目录结构

```
qtcreator_windows/
├── CMakeLists.txt   # Windows/MSVC 友好的独立 CMake 工程
└── README.md        # 本说明文件
```

- `CMakeLists.txt` 会复用 `../src` 目录中的全部源代码与资源文件，保持与原有工程一致的模块划分（核心库、可视化库、分析库、UI 库、GUI 与 CLI 可执行程序）。
- 新的 CMake 工程启用了 Qt 的 `AUTOMOC/AUTORCC/AUTOUIC` 自动化处理，并针对 MSVC 编译器添加了 `NOMINMAX`、`_CRT_SECURE_NO_WARNINGS` 等常用定义。

## 3. 在 QtCreator 中打开工程

1. 启动 QtCreator，选择 **File → Open File or Project...**。
2. 指向本目录中的 `CMakeLists.txt` 文件并打开。
3. 在弹出的 Kit 选择界面中，选择与已安装 Qt 版本匹配的 MSVC Kit（例如 *Desktop Qt 6.5.3 MSVC2019 64bit*）。
4. 在 CMake 变量面板中设置以下路径（根据本地安装位置调整）：
   - `PCL_DIR`：例如 `C:/dev/PCL/lib/cmake/pcl`
   - `VTK_DIR`：例如 `C:/dev/VTK/lib/cmake/vtk-9.2`
   - `Qt6_DIR`：例如 `C:/Qt/6.5.3/msvc2019_64/lib/cmake/Qt6`
5. 点击 **Configure Project**，等待 CMake 配置完成后即可编译运行。

> ✅ 如果依赖库位于非标准位置，可在 **Projects → Build → CMake** 选项卡中新增或修改上述变量。QtCreator 会将这些路径写入 `CMakeCache.txt`，无需每次重复设置。

## 4. 构建与运行

- QtCreator 将生成两个可执行程序：
  - `pcl_viewer_gui`：图形界面版，与原项目 GUI 功能完全一致。
  - `pcl_analyze_cli`：命令行分析工具，用于快速计算点云指标。
- 构建输出位于 `build/<kit>/bin`，静态库位于 `build/<kit>/lib`。
- 运行 GUI 程序前，请确保 `data/` 目录中的示例点云文件与可执行文件处于可访问路径（可在 QtCreator 的 **Projects → Run → Working Directory** 中指定 `PCL_Visualization` 根目录）。

## 5. 运行时注意事项

- Windows 下需要确保以下目录已加入系统 `PATH`：
  - `Qt\6.x.x\msvc20xx_64\bin`
  - `VTK\bin`
  - `PCL\bin`
  - `Boost\lib`（若 PCL 依赖 Boost 动态库）
- 若使用自定义的 GPU 或 OpenGL 环境，请确保显卡驱动和 OpenGL 运行时可用。
- 若需要使用其他版本的依赖库，可在 CMake 配置阶段更新 `CMAKE_PREFIX_PATH` 或相关变量。

## 6. 与原工程的对应关系

- 核心/可视化/分析/UI 模块的源代码与头文件均来自 `../src`，未做任何修改。
- 构建产物命名保持一致，确保与原有说明文档和脚本兼容。
- 该目录不会影响原先的 Linux/WSL 构建流程，可并行维护。

## 7. 常见问题排查

| 问题 | 解决方案 |
| ---- | -------- |
| 找不到 `Qt6::Widgets` 等目标 | 检查是否选择了正确的 Qt Kit，并确认 `Qt6_DIR` 设置指向 `lib/cmake/Qt6` |
| `PCLConfig.cmake` 未找到 | 设置 `PCL_DIR` 为 `PCL/lib/cmake/pcl`，或将该路径添加到 `CMAKE_PREFIX_PATH` |
| 运行时报错缺少 DLL | 将 Qt、VTK、PCL、Boost 的 `bin` 目录加入 `PATH`，或与可执行文件放在同一目录 |
| VTK 与 Qt 的 OpenGL 版本不匹配 | 确保 VTK 构建时启用了 `VTK_GROUP_ENABLE_Qt=YES` 且使用相同的 Qt 版本 |

---

通过上述配置，即可在 Windows + QtCreator 环境下构建并运行与原项目完全一致的点云可视化工具。若后续需要进一步自动化或添加新的 Kit，可在此目录下扩展额外的 CMake Presets 或工具链文件。
