# 点云可视化软件

## 进度
- [x] 实现数据加载和可视化
- [x] ui界面
## 注意

### wsl环境出现vtk动态库链接丢失问题

可能当时库没安装好，需要进行vtk环境加载
```bash
➤  ./pcl_viewer                                20:46:08
./pcl_viewer: error while loading shared libraries: libvtkglew-8.2.so.1: cannot open shared object file: No such file or directory
…n at serendipity in ~/work/PCL_Visualization/src/build
➤  vtk_env                                     20:46:13
✨ WSL Graphics Environment Ready!

🎯 Configuration:
   Rendering: Software (llvmpipe)
   OpenGL Version: 3.3
   Display: localhost:0 ✅

🎮 Ready to run graphics programs:
   vtk programs, paraview, blender, freecad, etc.

💡 To see all environment variables, run: env | grep -E '(MESA|LIBGL|LD_LIBRARY)'
```

### wsl显示问题

```bash
  ./pcl_viewer                                20:48:51
正在加载点云文件: ../../data/H103v2.asc
成功加载 740 个点
=== 点云信息 ===
点数量: 740
X 范围: [-34.8424, -33.6308] (跨度: 1.21156)
Y 范围: [5.48034, 6.62647] (跨度: 1.14613)
Z 范围: [12.2084, 12.5439] (跨度: 0.335485)
===============
ERROR: In /home/serenNan/Tools/VTK-8.2.0/Rendering/OpenGL2/vtkXOpenGLRenderWindow.cxx, line 1268
vtkXOpenGLRenderWindow (0x5718620fca70): bad X server connection. DISPLAY=localhost:0. Aborting.


fish: Job 1, './pcl_viewer' terminated by signal SIGABRT (Abort)
…n at serendipity in ~/work/PCL_Visualization/src/build
➤  export DISPLAY=:0 && ./pcl_viewer           20:48:57
正在加载点云文件: ../../data/H103v2.asc
成功加载 740 个点
=== 点云信息 ===
点数量: 740
X 范围: [-34.8424, -33.6308] (跨度: 1.21156)
Y 范围: [5.48034, 6.62647] (跨度: 1.14613)
Z 范围: [12.2084, 12.5439] (跨度: 0.335485)
===============
点云可视化器已启动。按 'q' 退出，'h' 显示帮助信息。
```

### 动态库丢失问题

```cpp
export LD_LIBRARY_PATH=/usr/local/Qt/Qt6.5.3/6.5.3/gcc_64/lib:/usr/local/vtk/lib:$LD_LIBRARY_PATH
```