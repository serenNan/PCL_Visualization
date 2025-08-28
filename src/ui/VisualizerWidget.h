#pragma once

#include <memory>
#include <map>
#include <QtWidgets/QWidget>
// ! 修复方案：使用QOpenGLWidget与VTK手动集成
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtCore/QTimer>
#include <QtGui/QResizeEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "../core/PointCloud.h"
#include "../visualization/Visualizer.h"
#include "Camera3D.h"

namespace pcl_viz {
namespace ui {

/**
 * @brief PCL可视化器Qt封装组件
 * 
 * 将PCL点云数据直接集成到VTK渲染管道中，提供3D点云可视化功能
 * 使用QOpenGLWidget与VTK手动集成，确保点云在Qt窗口内正确显示
 */
class VisualizerWidget : public QOpenGLWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit VisualizerWidget(QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~VisualizerWidget() override;

    /**
     * @brief 设置要显示的点云
     * @param pointCloud 点云对象
     * @return 是否设置成功
     */
    bool setPointCloud(std::shared_ptr<core::PointCloud> pointCloud);

    /**
     * @brief 移除当前点云
     */
    void removePointCloud();

    /**
     * @brief 重置相机视角
     */
    void resetCamera();

    /**
     * @brief 更新可视化配置
     * @param config 新的配置
     */
    void updateVisualizationConfig(const visualization::VisualizationConfig& config);

    /**
     * @brief 获取当前可视化配置
     * @return 当前配置
     */
    const visualization::VisualizationConfig& getVisualizationConfig() const;

    /**
     * @brief 保存当前视图截图
     * @param filename 保存的文件名
     * @return 是否保存成功
     */
    bool saveScreenshot(const QString& filename);

    /**
     * @brief 添加3D文本显示
     * @param text 文本内容
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param textId 文本标识符
     * @param fontSize 字体大小
     */
    void add3DText(const QString& text, double x, double y, double z, 
                   const QString& textId = "text", int fontSize = 12);

    /**
     * @brief 移除3D文本
     * @param textId 文本标识符
     */
    void remove3DText(const QString& textId = "text");

    /**
     * @brief 添加坐标系显示
     * @param scale 坐标系缩放比例
     * @param origin_x X原点坐标
     * @param origin_y Y原点坐标
     * @param origin_z Z原点坐标
     */
    void addCoordinateSystem(double scale = 1.0, 
                            double origin_x = 0.0, double origin_y = 0.0, double origin_z = 0.0);

    /**
     * @brief 移除坐标系
     */
    void removeCoordinateSystem();

    /**
     * @brief 设置背景颜色
     * @param r 红色分量 [0-1]
     * @param g 绿色分量 [0-1]
     * @param b 蓝色分量 [0-1]
     */
    void setBackgroundColor(double r, double g, double b);

    /**
     * @brief 设置点大小
     * @param pointSize 点大小
     */
    void setPointSize(float pointSize);

    /**
     * @brief 获取VTK渲染器指针（高级用法）
     * @return VTK渲染器指针
     */
    vtkRenderer* getVTKRenderer() { return m_renderer.Get(); }

public slots:
    /**
     * @brief 开始渲染循环
     */
    void startRenderLoop();

    /**
     * @brief 停止渲染循环
     */
    void stopRenderLoop();

    /**
     * @brief 执行一次渲染更新
     */
    void updateRender();

signals:
    /**
     * @brief 点云设置完成信号
     * @param success 是否成功
     */
    void pointCloudSet(bool success);

    /**
     * @brief 鼠标点击信号
     * @param x 屏幕X坐标
     * @param y 屏幕Y坐标
     * @param worldX 世界坐标X
     * @param worldY 世界坐标Y
     * @param worldZ 世界坐标Z
     */
    void mouseClicked(int x, int y, double worldX, double worldY, double worldZ);

    /**
     * @brief 相机视角改变信号
     */
    void cameraChanged();

protected:
    /**
     * @brief 初始化OpenGL上下文
     */
    void initializeGL() override;
    
    /**
     * @brief 绘制场景
     */
    void paintGL() override;

    /**
     * @brief 窗口大小改变事件
     * @param w 宽度
     * @param h 高度
     */
    void resizeGL(int w, int h) override;

    /**
     * @brief 鼠标按下事件
     * @param event 鼠标事件
     */
    void mousePressEvent(QMouseEvent *event) override;

    /**
     * @brief 鼠标移动事件
     * @param event 鼠标事件
     */
    void mouseMoveEvent(QMouseEvent *event) override;

    /**
     * @brief 鼠标释放事件
     * @param event 鼠标事件
     */
    void mouseReleaseEvent(QMouseEvent *event) override;

    /**
     * @brief 滚轮事件
     * @param event 滚轮事件
     */
    void wheelEvent(QWheelEvent *event) override;

private slots:
    /**
     * @brief 定时器渲染更新
     */
    void onRenderTimerTimeout();

private:
    /**
     * @brief 初始化VTK渲染管道
     */
    void initializeVTKPipeline();

    /**
     * @brief 将PCL点云转换为VTK数据
     * @param pointCloud PCL点云数据
     * @return 是否转换成功
     */
    bool convertPCLToVTK(std::shared_ptr<core::PointCloud> pointCloud);

    /**
     * @brief 设置默认视角和属性
     */
    void setupDefaultView();

    /**
     * @brief 设置颜色查找表
     */
    void setupColorLUT();

    /**
     * @brief 更新点云颜色映射
     * @param pointCloud 点云数据
     */
    void updateColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud);
    
    /**
     * @brief 设置相机位置和视角
     * @param pointCloud 点云数据用于计算适合的相机位置
     */
    void setupCamera(const std::shared_ptr<core::PointCloud>& pointCloud);
    
    /**
     * @brief 应用颜色映射
     * @param pointCloud 点云数据
     * @param cloudId 点云ID
     */
    void applyColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud, const std::string& cloudId);

    /**
     * @brief 创建坐标系显示
     * @param scale 缩放比例
     * @param origin 原点坐标
     */
    void createCoordinateSystemActor(double scale, const double origin[3]);

    /**
     * @brief 添加3D文本到VTK场景
     * @param text 文本内容
     * @param position 位置坐标
     * @param textId 文本ID
     * @param fontSize 字体大小
     */
    void addVTK3DText(const QString& text, const double position[3], 
                      const QString& textId, int fontSize);

    /**
     * @brief 使用完整VTK管道进行渲染
     */
    void renderWithVTK();

    /**
     * @brief 安全后备渲染模式
     */
    void renderFallback();

    /**
     * @brief 测试VTK渲染是否安全可用
     * @return VTK渲染是否可以安全使用
     */
    bool testVTKRenderingSafety();

    /**
     * @brief 基础OpenGL点云渲染
     * @param pclCloud PCL点云数据
     */
    void renderPointCloudBasic(core::PCLPointCloud::Ptr pclCloud);

    /**
     * @brief 渲染状态文本信息
     */
    void renderStatusText();

    /**
     * @brief 渲染测试几何体验证OpenGL基础功能
     */
    void renderTestGeometry();

    // VTK 组件
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderWindow;  ///< VTK渲染窗口
    vtkSmartPointer<vtkRenderer> m_renderer;                      ///< VTK渲染器
    
    // * 核心修复：直接使用VTK组件而非PCL Visualizer
    vtkSmartPointer<vtkPolyData> m_pointCloudPolyData;           ///< 点云VTK数据
    vtkSmartPointer<vtkPolyDataMapper> m_pointCloudMapper;       ///< 点云映射器
    vtkSmartPointer<vtkActor> m_pointCloudActor;                 ///< 点云演员
    vtkSmartPointer<vtkLookupTable> m_colorLUT;                  ///< 颜色查找表
    
    visualization::VisualizationConfig m_config;                  ///< 可视化配置

    // 数据
    std::shared_ptr<core::PointCloud> m_currentPointCloud;       ///< 当前显示的点云
    QString m_currentCloudId;                                     ///< 当前点云ID

    // 渲染控制
    QTimer* m_renderTimer;                                        ///< 渲染定时器
    bool m_renderLoopActive;                                      ///< 渲染循环活动状态

    // 交互状态
    bool m_mousePressed;                                          ///< 鼠标按下状态
    QPoint m_lastMousePosition;                                   ///< 上次鼠标位置
    Qt::MouseButton m_activeMouseButton;                          ///< 当前活动的鼠标按键
    
    // 3D相机系统
    std::unique_ptr<Camera3D> m_camera;                           ///< 3D相机控制器

    // 初始化状态
    bool m_initialized;                                           ///< 是否已初始化
    bool m_coordinateSystemAdded;                                 ///< 坐标系是否已添加
    bool m_vtkRenderingSafe;                                      ///< VTK渲染是否安全可用
    
    // VTK坐标系和文本演员 - 暂时禁用以解决编译问题
    // vtkSmartPointer<vtkAxesActor> m_coordinateSystemActor;       ///< 坐标系演员
    std::map<QString, vtkSmartPointer<vtkTextActor>> m_textActors;   ///< 文本演员映射
};

} // namespace ui
} // namespace pcl_viz