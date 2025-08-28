#include "VisualizerWidget.h"

#include <QtCore/QTimer>
#include <QtGui/QResizeEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>
#include <QtCore/QDebug>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyle.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkLookupTable.h>
#include <vtkScalarBarActor.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkTextActor.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkTextProperty.h>

#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include <QOpenGLFunctions>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pcl_viz {
namespace ui {

VisualizerWidget::VisualizerWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , m_renderWindow(nullptr)
    , m_renderer(nullptr)
    , m_pointCloudPolyData(nullptr)
    , m_pointCloudMapper(nullptr)
    , m_pointCloudActor(nullptr)
    , m_colorLUT(nullptr)
    , m_currentPointCloud(nullptr)
    , m_currentCloudId("main_cloud")
    , m_renderTimer(nullptr)
    , m_renderLoopActive(false)
    , m_mousePressed(false)
    , m_activeMouseButton(Qt::NoButton)
    , m_camera(std::make_unique<Camera3D>())
    , m_initialized(false)
    , m_coordinateSystemAdded(false)
    , m_vtkRenderingSafe(false)
{
    // 设置默认配置
    m_config.windowTitle = "PCL 点云可视化器";
    m_config.windowWidth = 800;
    m_config.windowHeight = 600;
    m_config.pointSize = 2.0f;
    m_config.colorMode = visualization::VisualizationConfig::ColorMode::HEIGHT_Z;
    m_config.showCoordinateSystem = true;
    m_config.coordinateSystemScale = 1.0f;
    m_config.backgroundColor[0] = 0.1;
    m_config.backgroundColor[1] = 0.1;
    m_config.backgroundColor[2] = 0.2;
    m_config.autoSetCamera = true;
    
    // ! 修复：禁用自动渲染定时器，改为手动触发渲染
    m_renderTimer = new QTimer(this);
    connect(m_renderTimer, &QTimer::timeout, this, &VisualizerWidget::onRenderTimerTimeout);
    // 默认不启动定时器，避免渲染死循环
    
    // ! 修复：移除构造函数中的VTK初始化，改为在initializeGL中进行
    // VTK管道将在OpenGL上下文创建后初始化
    
    qDebug() << "VisualizerWidget构造完成，使用VTK渲染管道";
}

VisualizerWidget::~VisualizerWidget() {
    stopRenderLoop();
    
    // 清理VTK资源
    if (m_pointCloudActor && m_renderer) {
        m_renderer->RemoveActor(m_pointCloudActor);
    }
    // 坐标系暂时禁用
    /*
    if (m_coordinateSystemActor && m_renderer) {
        m_renderer->RemoveActor(m_coordinateSystemActor);
    }
    */
    for (auto& textActor : m_textActors) {
        if (textActor.second && m_renderer) {
            m_renderer->RemoveActor(textActor.second);
        }
    }
    
    qDebug() << "VisualizerWidget析构完成";
}

bool VisualizerWidget::setPointCloud(std::shared_ptr<core::PointCloud> pointCloud) {
    if (!pointCloud || !m_renderer) {
        emit pointCloudSet(false);
        return false;
    }
    
    try {
        // 移除之前的点云
        if (m_pointCloudActor && m_currentPointCloud) {
            m_renderer->RemoveActor(m_pointCloudActor);
        }
        
        // 转换PCL点云为VTK数据
        if (!convertPCLToVTK(pointCloud)) {
            qDebug() << "PCL到VTK转换失败";
            emit pointCloudSet(false);
            return false;
        }
        
        // 创建或更新VTK渲染管道
        if (!m_pointCloudMapper) {
            m_pointCloudMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        }
        m_pointCloudMapper->SetInputData(m_pointCloudPolyData);
        
        if (!m_pointCloudActor) {
            m_pointCloudActor = vtkSmartPointer<vtkActor>::New();
        }
        m_pointCloudActor->SetMapper(m_pointCloudMapper);
        
        // 设置点云属性
        m_pointCloudActor->GetProperty()->SetPointSize(m_config.pointSize);
        
        // 添加到渲染器
        m_renderer->AddActor(m_pointCloudActor);
        
        // 应用颜色映射
        updateColorMapping(pointCloud);
        
        // 自动设置相机
        if (m_config.autoSetCamera) {
            setupCamera(pointCloud);
        }
        
        // 添加坐标系
        if (m_config.showCoordinateSystem && !m_coordinateSystemAdded) {
            double origin[3] = {0.0, 0.0, 0.0};
            createCoordinateSystemActor(m_config.coordinateSystemScale, origin);
        }
        
        // 更新当前点云
        m_currentPointCloud = pointCloud;
        
        // 强制渲染更新
        updateRender();
        
        emit pointCloudSet(true);
        return true;
        
    } catch (const std::exception& e) {
        qDebug() << "设置点云时发生异常:" << e.what();
        emit pointCloudSet(false);
        return false;
    }
}

void VisualizerWidget::removePointCloud() {
    if (m_pointCloudActor && m_currentPointCloud && m_renderer) {
        m_renderer->RemoveActor(m_pointCloudActor);
        m_currentPointCloud.reset();
        updateRender();
    }
}

void VisualizerWidget::resetCamera() {
    if (m_camera) {
        m_camera->reset();
        updateRender();
        emit cameraChanged();
    }
}

void VisualizerWidget::updateVisualizationConfig(const visualization::VisualizationConfig& config) {
    m_config = config;
    
    if (!m_renderer) {
        return;
    }
    
    // 更新背景颜色
    setBackgroundColor(m_config.backgroundColor[0], 
                      m_config.backgroundColor[1], 
                      m_config.backgroundColor[2]);
    
    // 更新点大小
    if (m_currentPointCloud && m_pointCloudActor) {
        setPointSize(m_config.pointSize);
        updateColorMapping(m_currentPointCloud);
    }
    
    // 更新坐标系显示
    if (m_config.showCoordinateSystem && !m_coordinateSystemAdded) {
        double origin[3] = {0.0, 0.0, 0.0};
        createCoordinateSystemActor(m_config.coordinateSystemScale, origin);
    } else if (!m_config.showCoordinateSystem && m_coordinateSystemAdded) {
        removeCoordinateSystem();
    }
    
    updateRender();
}

const visualization::VisualizationConfig& VisualizerWidget::getVisualizationConfig() const {
    return m_config;
}

bool VisualizerWidget::saveScreenshot(const QString& filename) {
    if (!m_renderWindow) {
        return false;
    }
    
    try {
        // VTK 截图功能
        vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = 
            vtkSmartPointer<vtkWindowToImageFilter>::New();
        windowToImageFilter->SetInput(m_renderWindow);
        windowToImageFilter->Update();
        
        vtkSmartPointer<vtkPNGWriter> writer = 
            vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(filename.toStdString().c_str());
        writer->SetInputConnection(windowToImageFilter->GetOutputPort());
        writer->Write();
        
        return true;
    } catch (...) {
        return false;
    }
}

void VisualizerWidget::add3DText(const QString& text, double x, double y, double z, 
                                const QString& textId, int fontSize) {
    if (m_renderer) {
        double position[3] = {x, y, z};
        addVTK3DText(text, position, textId, fontSize);
        updateRender();
    }
}

void VisualizerWidget::remove3DText(const QString& textId) {
    auto it = m_textActors.find(textId);
    if (it != m_textActors.end() && m_renderer) {
        m_renderer->RemoveActor(it->second);
        m_textActors.erase(it);
        updateRender();
    }
}

void VisualizerWidget::addCoordinateSystem(double scale, 
                                         double origin_x, double origin_y, double origin_z) {
    if (m_renderer && !m_coordinateSystemAdded) {
        double origin[3] = {origin_x, origin_y, origin_z};
        createCoordinateSystemActor(scale, origin);
        updateRender();
    }
}

void VisualizerWidget::removeCoordinateSystem() {
    // 坐标系暂时禁用
    /*
    if (m_coordinateSystemActor && m_coordinateSystemAdded && m_renderer) {
        m_renderer->RemoveActor(m_coordinateSystemActor);
        m_coordinateSystemActor = nullptr;
        m_coordinateSystemAdded = false;
        updateRender();
    }
    */
}

void VisualizerWidget::setBackgroundColor(double r, double g, double b) {
    if (m_renderer) {
        m_renderer->SetBackground(r, g, b);
        m_config.backgroundColor[0] = r;
        m_config.backgroundColor[1] = g;
        m_config.backgroundColor[2] = b;
        updateRender();
    }
}

void VisualizerWidget::setPointSize(float pointSize) {
    if (m_pointCloudActor && m_currentPointCloud) {
        m_pointCloudActor->GetProperty()->SetPointSize(pointSize);
        m_config.pointSize = pointSize;
        updateRender();
    }
}

void VisualizerWidget::startRenderLoop() {
    // ! 紧急修复：暂时禁用连续渲染循环，防止CPU过度占用
    qDebug() << "渲染循环请求被忽略 - 使用手动渲染模式";
    // 改为单次渲染
    if (m_initialized) {
        update();
    }
    /*
    if (!m_renderLoopActive) {
        m_renderLoopActive = true;
        m_renderTimer->start(33); // ~30 FPS
    }
    */
}

void VisualizerWidget::stopRenderLoop() {
    if (m_renderLoopActive) {
        m_renderLoopActive = false;
        m_renderTimer->stop();
    }
}

void VisualizerWidget::updateRender() {
    // ! 安全恢复：只触发Qt重绘，实际渲染在paintGL中进行
    if (m_initialized) {
        // 只触发Qt的重绘事件，真正的渲染在paintGL()中安全执行
        update();
        // 静默模式：不输出日志避免刷屏
    }
}

void VisualizerWidget::mousePressEvent(QMouseEvent *event) {
    m_mousePressed = true;
    m_lastMousePosition = event->pos();
    m_activeMouseButton = event->button();
    
    // 设置鼠标捕获，确保即使鼠标移出窗口也能继续接收事件
    setMouseTracking(true);
    
    QOpenGLWidget::mousePressEvent(event);
}

void VisualizerWidget::mouseMoveEvent(QMouseEvent *event) {
    if (m_mousePressed && m_camera) {
        QPoint currentPos = event->pos();
        QPoint deltaPos = currentPos - m_lastMousePosition;
        
        // 根据按键类型执行不同操作
        switch (m_activeMouseButton) {
            case Qt::LeftButton:
                // 左键：旋转
                m_camera->rotate(static_cast<float>(deltaPos.x()), 
                               static_cast<float>(deltaPos.y()), 
                               0.01f);
                updateRender();
                emit cameraChanged();
                break;
                
            case Qt::RightButton:
                // 右键：平移
                m_camera->pan(static_cast<float>(-deltaPos.x()), 
                            static_cast<float>(deltaPos.y()), 
                            0.002f);
                updateRender();
                emit cameraChanged();
                break;
                
            case Qt::MiddleButton:
                {
                    // 中键：缩放（通过Y方向移动）
                    float zoomFactor = 1.0f + static_cast<float>(deltaPos.y()) * 0.01f;
                    m_camera->zoom(zoomFactor);
                    updateRender();
                    emit cameraChanged();
                    break;
                }
                
            default:
                // 其他按键不处理
                break;
        }
    }
    
    m_lastMousePosition = event->pos();
    QOpenGLWidget::mouseMoveEvent(event);
}

void VisualizerWidget::mouseReleaseEvent(QMouseEvent *event) {
    m_mousePressed = false;
    m_activeMouseButton = Qt::NoButton;
    
    // 停止鼠标捕获
    setMouseTracking(false);
    
    // 发出鼠标点击信号（简化版，不计算3D坐标）
    emit mouseClicked(event->position().x(), event->position().y(), 0.0, 0.0, 0.0);
    
    QOpenGLWidget::mouseReleaseEvent(event);
}

void VisualizerWidget::wheelEvent(QWheelEvent *event) {
    if (m_camera) {
        // 获取滚轮滚动量
        QPoint numDegrees = event->angleDelta() / 8;
        QPoint numSteps = numDegrees / 15; // 大多数鼠标每步15度
        
        if (numSteps.y() != 0) {
            // 计算缩放因子
            float zoomFactor = 1.0f;
            
            if (numSteps.y() > 0) {
                // 向前滚动：放大（缩小距离）
                zoomFactor = 0.9f;
            } else {
                // 向后滚动：缩小（增大距离）
                zoomFactor = 1.1f;
            }
            
            // 应用缩放
            m_camera->zoom(zoomFactor);
            updateRender();
            emit cameraChanged();
        }
    }
    
    QOpenGLWidget::wheelEvent(event);
}

void VisualizerWidget::onRenderTimerTimeout() {
    // ! 紧急修复：完全禁用定时器渲染，防止死循环
    qDebug() << "定时器渲染被禁用 - 使用事件驱动渲染";
    // 不执行任何渲染操作
}

void VisualizerWidget::initializeGL() {
    // ! 关键修复：只在OpenGL上下文准备好后初始化VTK
    if (!m_initialized) {
        qDebug() << "OpenGL上下文已准备，开始初始化VTK渲染管道";
        initializeVTKPipeline();
        
        // ! 修复：初始化完成后不自动启动渲染循环
        if (m_initialized) {
            // 只进行一次初始渲染，不启动连续渲染循环
            update(); // 使用Qt的重绘机制
            qDebug() << "VTK初始化成功，已触发初始渲染";
        }
    }
}

void VisualizerWidget::paintGL() {
    // ! 渐进式安全VTK渲染恢复
    if (!m_initialized || !m_renderWindow) {
        // 初始化失败时的后备渲染
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        return;
    }
    
    // * 根据VTK渲染安全性选择渲染模式
    if (m_vtkRenderingSafe) {
        // 完整VTK渲染模式
        renderWithVTK();
    } else {
        // 安全后备渲染模式
        renderFallback();
    }
}

void VisualizerWidget::resizeGL(int w, int h) {
    // 更新VTK渲染窗口大小
    if (m_renderWindow) {
        m_renderWindow->SetSize(w, h);
        // ! 修复：不在resizeGL中立即渲染，因为这会触发VTK交互器初始化
        // 渲染将通过定时器在稍后安全地进行
        qDebug() << "VTK窗口大小已更新为:" << w << "x" << h;
    }
}

void VisualizerWidget::initializeVTKPipeline() {
    try {
        qDebug() << "开始初始化VTK渲染管道...";
        
        // * 关键修复：手动创建VTK渲染窗口并与Qt OpenGL上下文集成
        
        // 创建VTK渲染窗口
        m_renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        if (!m_renderWindow) {
            qDebug() << "错误：无法创建VTK渲染窗口";
            return;
        }
        
        // 创建渲染器
        m_renderer = vtkSmartPointer<vtkRenderer>::New();
        if (!m_renderer) {
            qDebug() << "错误：无法创建VTK渲染器";
            return;
        }
        
        m_renderWindow->AddRenderer(m_renderer);
        
        // 设置窗口大小
        int w = width() > 0 ? width() : 800;
        int h = height() > 0 ? height() : 600;
        m_renderWindow->SetSize(w, h);
        qDebug() << "VTK窗口大小设置为:" << w << "x" << h;
        
        // ! 安全的交互器设置：避免WSL环境中的X11问题
        // 创建交互器但采用受控的初始化方式
        vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
        if (!interactor) {
            qDebug() << "错误：无法创建VTK交互器";
            return;
        }
        
        // 设置交互器到渲染窗口
        m_renderWindow->SetInteractor(interactor);
        
        // 设置交互样式
        vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = 
            vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
        if (style) {
            interactor->SetInteractorStyle(style);
        }
        
        // ! 重要安全措施：不调用 interactor->Initialize() 
        // 在WSL环境中避免X11显示系统冲突，改用Qt的事件系统
        qDebug() << "VTK交互器已安全配置（使用Qt事件处理）";
        
        // 启用深度缓冲和多重采样
        m_renderWindow->SetMultiSamples(4);
        
        // ! 关键修复：将VTK渲染窗口与Qt的OpenGL上下文绑定
        // 这确保VTK能在Qt管理的OpenGL上下文中正确渲染
        m_renderWindow->SetCurrentCursor(0); // 禁用VTK的光标管理
        m_renderWindow->SetDisplayId(nullptr); // 让VTK使用当前的OpenGL上下文
        m_renderWindow->SetWindowId(nullptr);  // 不创建独立的窗口
        
        // 设置默认视图属性
        setupDefaultView();
        
        // 初始化颜色查找表
        setupColorLUT();
        
        m_initialized = true;
        
        // * 验证VTK渲染管道是否安全可用
        if (testVTKRenderingSafety()) {
            m_vtkRenderingSafe = true;
            qDebug() << "VTK渲染管道初始化完成且验证安全";
        } else {
            qDebug() << "VTK渲染管道初始化完成但存在安全问题，将使用基础渲染模式";
        }
        
    } catch (const std::exception& e) {
        qDebug() << "VTK初始化异常:" << e.what();
        m_initialized = false;
    } catch (...) {
        qDebug() << "VTK初始化未知异常";
        m_initialized = false;
    }
}

bool VisualizerWidget::convertPCLToVTK(std::shared_ptr<core::PointCloud> pointCloud) {
    if (!pointCloud) {
        return false;
    }
    
    auto pclCloud = pointCloud->getPCLPointCloud();
    if (!pclCloud || pclCloud->empty()) {
        return false;
    }
    
    // 创建VTK点数据
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(pclCloud->size());
    
    // 创建颜色数组（基于Z坐标）
    vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New();
    colors->SetNumberOfComponents(1);
    colors->SetNumberOfTuples(pclCloud->size());
    colors->SetName("elevation");
    
    // 转换点坐标和计算颜色值
    for (size_t i = 0; i < pclCloud->size(); ++i) {
        const auto& pt = (*pclCloud)[i];
        points->SetPoint(i, pt.x, pt.y, pt.z);
        colors->SetValue(i, pt.z); // 使用Z坐标作为颜色值
    }
    
    // 创建PolyData
    if (!m_pointCloudPolyData) {
        m_pointCloudPolyData = vtkSmartPointer<vtkPolyData>::New();
    }
    m_pointCloudPolyData->SetPoints(points);
    m_pointCloudPolyData->GetPointData()->SetScalars(colors);
    
    // 使用VertexGlyphFilter来渲染点
    vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = 
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    vertexFilter->SetInputData(m_pointCloudPolyData);
    vertexFilter->Update();
    
    m_pointCloudPolyData->ShallowCopy(vertexFilter->GetOutput());
    
    return true;
}

void VisualizerWidget::setupDefaultView() {
    if (!m_renderer) {
        return;
    }
    
    // 设置背景颜色
    m_renderer->SetBackground(
        m_config.backgroundColor[0],
        m_config.backgroundColor[1],
        m_config.backgroundColor[2]);
    
    // 设置相机
    vtkCamera* camera = m_renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 10);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
    }
}

void VisualizerWidget::setupColorLUT() {
    m_colorLUT = vtkSmartPointer<vtkLookupTable>::New();
    m_colorLUT->SetHueRange(0.667, 0.0); // 蓝到红
    m_colorLUT->SetNumberOfColors(256);
    m_colorLUT->Build();
}

void VisualizerWidget::updateColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud) {
    if (!pointCloud || !m_pointCloudMapper || !m_colorLUT) {
        return;
    }
    
    // 根据配置的颜色模式更新映射
    switch (m_config.colorMode) {
        case visualization::VisualizationConfig::ColorMode::HEIGHT_Z:
            // Z坐标颜色映射已经在convertPCLToVTK中设置
            m_pointCloudMapper->SetLookupTable(m_colorLUT);
            m_pointCloudMapper->UseLookupTableScalarRangeOn();
            break;
        // case visualization::VisualizationConfig::ColorMode::SINGLE_COLOR:
        //     // 单一颜色
        //     m_pointCloudMapper->ScalarVisibilityOff();
        //     break;
        default:
            m_pointCloudMapper->SetLookupTable(m_colorLUT);
            break;
    }
}

void VisualizerWidget::setupCamera(const std::shared_ptr<core::PointCloud>& pointCloud) {
    if (!m_camera || !pointCloud) {
        return;
    }
    
    auto pclCloud = pointCloud->getPCLPointCloud();
    if (!pclCloud || pclCloud->empty()) {
        return;
    }
    
    // 计算点云边界
    core::PointT minPt, maxPt;
    pcl::getMinMax3D(*pclCloud, minPt, maxPt);
    
    // 使用Camera3D自动适配点云
    m_camera->autoFit(minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
    
    qDebug() << "相机已自动适配点云边界:"
             << "X[" << minPt.x << "," << maxPt.x << "]"
             << "Y[" << minPt.y << "," << maxPt.y << "]" 
             << "Z[" << minPt.z << "," << maxPt.z << "]";
}

void VisualizerWidget::applyColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud, 
                                        const std::string& cloudId) {
    // 这个方法现在通过updateColorMapping实现
    updateColorMapping(pointCloud);
}

void VisualizerWidget::createCoordinateSystemActor(double scale, const double origin[3]) {
    // 坐标系暂时禁用
    Q_UNUSED(scale);
    Q_UNUSED(origin);
    /*
    if (m_coordinateSystemAdded) {
        return;
    }
    
    // 创建坐标轴
    m_coordinateSystemActor = vtkSmartPointer<vtkAxesActor>::New();
    m_coordinateSystemActor->SetScale(scale);
    // 修复const转换问题
    double pos[3] = {origin[0], origin[1], origin[2]};
    m_coordinateSystemActor->SetPosition(pos);
    
    if (m_renderer) {
        m_renderer->AddActor(m_coordinateSystemActor);
        m_coordinateSystemAdded = true;
    }
    */
}

void VisualizerWidget::renderWithVTK() {
    try {
        // * 完整的VTK渲染流程
        makeCurrent(); // 确保OpenGL上下文是当前的
        
        // 设置VTK渲染窗口与当前OpenGL上下文同步
        m_renderWindow->SetSize(width(), height());
        
        // ! 关键：使用VTK渲染器而非渲染窗口直接渲染
        if (m_renderer) {
            // 清除之前的渲染缓冲
            glClearColor(
                static_cast<float>(m_config.backgroundColor[0]),
                static_cast<float>(m_config.backgroundColor[1]), 
                static_cast<float>(m_config.backgroundColor[2]), 
                1.0f
            );
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // * 安全的VTK渲染调用：直接使用渲染器而不是窗口
            m_renderer->Render();
        }
        
    } catch (const std::exception& e) {
        qDebug() << "VTK渲染异常:" << e.what() << "- 禁用VTK渲染";
        m_vtkRenderingSafe = false; // 禁用VTK渲染
        renderFallback(); // 切换到安全模式
    } catch (...) {
        qDebug() << "VTK渲染未知异常 - 禁用VTK渲染";
        m_vtkRenderingSafe = false;
        renderFallback();
    }
}

void VisualizerWidget::renderFallback() {
    // * 纯净的点云渲染（仅显示点云数据）
    glClearColor(
        static_cast<float>(m_config.backgroundColor[0]),
        static_cast<float>(m_config.backgroundColor[1]), 
        static_cast<float>(m_config.backgroundColor[2]), 
        1.0f
    );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 只显示点云数据，清除其他所有内容
    if (m_currentPointCloud) {
        auto pclCloud = m_currentPointCloud->getPCLPointCloud();
        if (pclCloud && !pclCloud->empty()) {
            renderPointCloudBasic(pclCloud);
        }
    }
}

bool VisualizerWidget::testVTKRenderingSafety() {
    if (!m_renderWindow || !m_renderer) {
        return false;
    }
    
    // ! 保守策略：由于WSL环境的复杂性，暂时禁用VTK渲染测试
    // 在生产环境中，直接使用后备渲染模式以确保稳定性
    qDebug() << "VTK渲染安全性测试：WSL环境下采用保守策略，使用基础渲染模式";
    return false;
    
    /*
    // 原始测试代码保留，以备未来在不同环境中使用
    try {
        makeCurrent();
        m_renderWindow->SetSize(100, 100);
        m_renderer->Render();
        qDebug() << "VTK渲染安全性测试通过";
        return true;
    } catch (const std::exception& e) {
        qDebug() << "VTK渲染安全性测试失败:" << e.what();
        return false;
    } catch (...) {
        qDebug() << "VTK渲染安全性测试发生未知异常";
        return false;
    }
    */
}

void VisualizerWidget::renderPointCloudBasic(core::PCLPointCloud::Ptr pclCloud) {
    if (!pclCloud || pclCloud->empty()) {
        qDebug() << "renderPointCloudBasic: 点云为空，无法渲染";
        return;
    }
    
    // 移除频繁的调试输出，避免终端刷屏
    // qDebug() << "开始基础OpenGL点云渲染，点数:" << pclCloud->size();
    
    // 计算点云边界
    core::PointT minPt, maxPt;
    pcl::getMinMax3D(*pclCloud, minPt, maxPt);
    
    // 移除调试输出，保持界面干净
    
    // 计算点云中心和范围
    float centerX = (minPt.x + maxPt.x) * 0.5f;
    float centerY = (minPt.y + maxPt.y) * 0.5f;
    float centerZ = (minPt.z + maxPt.z) * 0.5f;
    
    float spanX = maxPt.x - minPt.x;
    float spanY = maxPt.y - minPt.y;
    float spanZ = maxPt.z - minPt.z;
    float maxSpan = std::max({spanX, spanY, spanZ});
    
    qDebug() << "点云中心:" << centerX << centerY << centerZ << "最大跨度:" << maxSpan;
    
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glPointSize(std::max(2.0f, m_config.pointSize * 2.0f)); // 增大点尺寸便于观察
    
    // * 重新设计投影和视图变换以确保点云可见
    
    // 设置投影矩阵
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    // 使用透视投影，更适合3D可视化
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    
    // 根据点云数据的实际范围调整视野
    float viewDistance = maxSpan * 3.0f; // 相机距离
    if (viewDistance < 1.0f) viewDistance = 5.0f; // 最小距离
    
    // 使用透视投影
    const float fov = 45.0f; // 视野角度
    const float near_plane = 0.1f;
    const float far_plane = viewDistance * 10.0f;
    
    // 手动计算透视投影矩阵
    float f = 1.0f / tanf(fov * M_PI / 360.0f); // fov/2 的余切值
    glLoadIdentity();
    float projMatrix[16] = {
        f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, f, 0.0f, 0.0f,
        0.0f, 0.0f, (far_plane + near_plane) / (near_plane - far_plane), -1.0f,
        0.0f, 0.0f, (2.0f * far_plane * near_plane) / (near_plane - far_plane), 0.0f
    };
    glMultMatrixf(projMatrix);
    
    // 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // * 使用Camera3D的变换矩阵
    if (m_camera) {
        m_camera->applyTransform();
    } else {
        // 后备方案：使用固定视角
        glTranslatef(0.0f, 0.0f, -viewDistance);
        glRotatef(-30.0f, 1.0f, 0.0f, 0.0f); // 轻微俯视
        glRotatef(30.0f, 0.0f, 1.0f, 0.0f);  // 轻微侧视
        glTranslatef(-centerX, -centerY, -centerZ); // 将点云中心移到原点
    }
    
    // 渲染所有点
    glBegin(GL_POINTS);
    
    // 计算Z坐标范围用于颜色映射
    float zRange = maxPt.z - minPt.z;
    if (zRange < 1e-6f) zRange = 1.0f; // 避免除零
    
    size_t rendered_points = 0;
    for (size_t i = 0; i < pclCloud->size(); ++i) {
        const auto& pt = (*pclCloud)[i];
        
        // 检查点的有效性
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            // 基于Z坐标的颜色映射（蓝色到红色）
            float colorValue = (pt.z - minPt.z) / zRange;
            colorValue = std::max(0.0f, std::min(1.0f, colorValue));
            
            // 蓝色（低）到红色（高）的颜色渐变
            float red = colorValue;
            float green = 0.3f + 0.4f * (1.0f - std::abs(colorValue - 0.5f) * 2.0f);
            float blue = 1.0f - colorValue;
            
            glColor3f(red, green, blue);
            glVertex3f(pt.x, pt.y, pt.z);
            rendered_points++;
        }
    }
    
    glEnd();
    
    // 移除频繁的调试输出，避免终端刷屏
    // qDebug() << "基础OpenGL渲染完成，实际渲染点数:" << rendered_points;
    
    // 移除调试输出，保持界面纯净
    
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_DEPTH_TEST);
}

void VisualizerWidget::renderStatusText() {
    // 状态文字已禁用，保持纯净的点云显示
}

void VisualizerWidget::addVTK3DText(const QString& text, const double position[3], 
                                   const QString& textId, int fontSize) {
    Q_UNUSED(position);
    
    // 移除已存在的相同ID文本
    remove3DText(textId);
    
    // 创建3D文本（简化实现，使用2D文本覆盖）
    vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
    textActor->SetInput(text.toStdString().c_str());
    textActor->SetPosition(50, 50); // 屏幕坐标
    textActor->GetTextProperty()->SetFontSize(fontSize);
    textActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
    
    m_textActors[textId] = textActor;
    
    if (m_renderer) {
        m_renderer->AddActor2D(textActor);
    }
}

void VisualizerWidget::renderTestGeometry() {
    // 测试几何体已禁用，保持纯净的点云显示
    return;
    
    // 启用深度测试
    glEnable(GL_DEPTH_TEST);
    
    // 设置投影矩阵
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    float aspect = static_cast<float>(width()) / static_cast<float>(height());
    glOrtho(-2.0f * aspect, 2.0f * aspect, -2.0f, 2.0f, -10.0f, 10.0f);
    
    // 设置模型视图矩阵
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // 静态计数器用于简单动画
    static int frame_count = 0;
    frame_count++;
    
    // 渲染坐标轴
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    
    // X轴 - 红色
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);
    
    // Y轴 - 绿色
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    
    // Z轴 - 蓝色
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    
    glEnd();
    
    // 渲染一个简单的测试立方体（线框）
    glColor3f(1.0f, 1.0f, 1.0f); // 白色
    glTranslatef(1.5f, 0.0f, 0.0f);
    
    glBegin(GL_LINE_LOOP);
    // 前面
    glVertex3f(-0.2f, -0.2f,  0.2f);
    glVertex3f( 0.2f, -0.2f,  0.2f);
    glVertex3f( 0.2f,  0.2f,  0.2f);
    glVertex3f(-0.2f,  0.2f,  0.2f);
    glEnd();
    
    glBegin(GL_LINE_LOOP);
    // 后面
    glVertex3f(-0.2f, -0.2f, -0.2f);
    glVertex3f(-0.2f,  0.2f, -0.2f);
    glVertex3f( 0.2f,  0.2f, -0.2f);
    glVertex3f( 0.2f, -0.2f, -0.2f);
    glEnd();
    
    glBegin(GL_LINES);
    // 连接前后面的边
    glVertex3f(-0.2f, -0.2f,  0.2f); glVertex3f(-0.2f, -0.2f, -0.2f);
    glVertex3f( 0.2f, -0.2f,  0.2f); glVertex3f( 0.2f, -0.2f, -0.2f);
    glVertex3f( 0.2f,  0.2f,  0.2f); glVertex3f( 0.2f,  0.2f, -0.2f);
    glVertex3f(-0.2f,  0.2f,  0.2f); glVertex3f(-0.2f,  0.2f, -0.2f);
    glEnd();
    
    // 渲染一些测试点
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    
    // 彩色测试点
    for (int i = 0; i < 5; ++i) {
        float angle = i * 72.0f * M_PI / 180.0f; // 5个点围成圆形
        float x = 0.8f * cosf(angle);
        float y = 0.8f * sinf(angle);
        
        // 基于角度的颜色
        glColor3f(
            0.5f + 0.5f * cosf(angle),
            0.5f + 0.5f * sinf(angle),
            0.8f
        );
        glVertex3f(x - 1.5f, y, 0.0f);
    }
    
    glEnd();
    
    glDisable(GL_DEPTH_TEST);
    
    // 每60帧输出一次测试几何体信息（约2秒）
    if (frame_count % 60 == 0) {
        qDebug() << "OpenGL测试几何体已渲染 - 坐标轴 + 立方体 + 5个测试点";
    }
}

} // namespace ui
} // namespace pcl_viz