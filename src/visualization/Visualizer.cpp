#include "Visualizer.h"
#include <iostream>
#include <thread>
#include <chrono>

#include <pcl/visualization/point_cloud_color_handlers.h>

namespace pcl_viz {
namespace visualization {

Visualizer::Visualizer(const VisualizationConfig& config) 
    : config_(config) {
    initializeViewer();
}

Visualizer::~Visualizer() {
    if (viewer_) {
        viewer_->close();
    }
}

void Visualizer::initializeViewer() {
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(config_.windowTitle));
    
    // 设置窗口大小
    viewer_->setSize(config_.windowWidth, config_.windowHeight);
    
    // 设置背景颜色
    viewer_->setBackgroundColor(
        config_.backgroundColor[0], 
        config_.backgroundColor[1], 
        config_.backgroundColor[2]
    );
    
    // 添加坐标系
    if (config_.showCoordinateSystem) {
        viewer_->addCoordinateSystem(config_.coordinateSystemScale);
    }
    
    // 初始化摄像机参数
    viewer_->initCameraParameters();
    
    // 注册键盘回调
    registerKeyboardCallback();
}

bool Visualizer::setPointCloud(const std::shared_ptr<core::PointCloud>& pointCloud, 
                               const std::string& cloudId) {
    if (!pointCloud || pointCloud->empty()) {
        std::cerr << "错误: 点云为空或无效" << std::endl;
        return false;
    }

    // 移除现有的同名点云
    if (viewer_->contains(cloudId)) {
        viewer_->removePointCloud(cloudId);
    }

    // 保存当前点云引用
    currentPointCloud_ = pointCloud;
    
    // 应用颜色映射并添加到可视化器
    applyColorMapping(pointCloud, cloudId);
    
    // 设置点的大小
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
        config_.pointSize, 
        cloudId
    );
    
    // 自动设置摄像机位置
    if (config_.autoSetCamera) {
        setupCamera(pointCloud);
    }
    
    std::cout << "成功添加点云 '" << cloudId << "' 到可视化器" << std::endl;
    return true;
}

bool Visualizer::removePointCloud(const std::string& cloudId) {
    if (!viewer_->contains(cloudId)) {
        std::cerr << "警告: 点云 '" << cloudId << "' 不存在" << std::endl;
        return false;
    }
    
    viewer_->removePointCloud(cloudId);
    
    if (cloudId == "main_cloud") {
        currentPointCloud_.reset();
    }
    
    return true;
}

void Visualizer::applyColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud, 
                                  const std::string& cloudId) {
    auto pclCloud = pointCloud->getPCLPointCloud();
    
    switch (config_.colorMode) {
        case VisualizationConfig::ColorMode::HEIGHT_Z: {
            pcl::visualization::PointCloudColorHandlerGenericField<core::PointT> colorHandler(pclCloud, "z");
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
        case VisualizationConfig::ColorMode::NORMAL_X: {
            pcl::visualization::PointCloudColorHandlerGenericField<core::PointT> colorHandler(pclCloud, "normal_x");
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
        case VisualizationConfig::ColorMode::NORMAL_Y: {
            pcl::visualization::PointCloudColorHandlerGenericField<core::PointT> colorHandler(pclCloud, "normal_y");
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
        case VisualizationConfig::ColorMode::NORMAL_Z: {
            pcl::visualization::PointCloudColorHandlerGenericField<core::PointT> colorHandler(pclCloud, "normal_z");
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
        case VisualizationConfig::ColorMode::UNIFORM: {
            pcl::visualization::PointCloudColorHandlerCustom<core::PointT> colorHandler(
                pclCloud, 
                static_cast<int>(config_.uniformColor[0] * 255),
                static_cast<int>(config_.uniformColor[1] * 255),
                static_cast<int>(config_.uniformColor[2] * 255)
            );
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
        default: {
            // 默认使用高度着色
            pcl::visualization::PointCloudColorHandlerGenericField<core::PointT> colorHandler(pclCloud, "z");
            viewer_->addPointCloud<core::PointT>(pclCloud, colorHandler, cloudId);
            break;
        }
    }
}

void Visualizer::setupCamera(const std::shared_ptr<core::PointCloud>& pointCloud) {
    const auto& bbox = pointCloud->getBoundingBox();
    const auto center = bbox.getCenter();
    
    // 计算合适的摄像机距离
    float maxSpan = std::max({bbox.getXSpan(), bbox.getYSpan(), bbox.getZSpan()});
    float distance = maxSpan * config_.cameraDistance;
    
    // 设置摄像机位置（从左下后方观察）
    viewer_->setCameraPosition(
        center.x - distance, center.y - distance, center.z + distance,  // 摄像机位置
        center.x, center.y, center.z,                                   // 观察点
        0, 0, 1                                                         // 向上向量
    );
}

void Visualizer::run(bool blocking) {
    if (!viewer_) {
        std::cerr << "错误: 可视化器未初始化" << std::endl;
        return;
    }
    
    std::cout << "点云可视化器已启动。" << std::endl;
    std::cout << "控制说明:" << std::endl;
    std::cout << "  - 鼠标左键拖拽: 旋转视角" << std::endl;
    std::cout << "  - 鼠标右键拖拽: 平移视角" << std::endl;
    std::cout << "  - 滚轮: 缩放" << std::endl;
    std::cout << "  - 'q': 退出" << std::endl;
    std::cout << "  - 'h': 显示帮助信息" << std::endl;
    std::cout << "  - 'r': 重置摄像机" << std::endl;
    std::cout << "  - 's': 截屏" << std::endl;
    
    if (blocking) {
        // 阻塞模式：运行直到窗口关闭
        while (!viewer_->wasStopped()) {
            viewer_->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

bool Visualizer::spinOnce(int timeMs) {
    if (!viewer_) {
        return false;
    }
    
    viewer_->spinOnce(timeMs);
    return !viewer_->wasStopped();
}

bool Visualizer::wasStopped() const {
    return viewer_ ? viewer_->wasStopped() : true;
}

void Visualizer::resetCamera() {
    if (viewer_) {
        if (currentPointCloud_ && !currentPointCloud_->empty() && config_.autoSetCamera) {
            setupCamera(currentPointCloud_);
        } else {
            viewer_->resetCamera();
        }
        std::cout << "摄像机视角已重置" << std::endl;
    }
}

void Visualizer::updateConfig(const VisualizationConfig& config) {
    config_ = config;
    
    if (viewer_) {
        // 更新背景颜色
        viewer_->setBackgroundColor(
            config_.backgroundColor[0], 
            config_.backgroundColor[1], 
            config_.backgroundColor[2]
        );
        
        // 重新应用点云渲染（如果有的话）
        if (currentPointCloud_) {
            setPointCloud(currentPointCloud_, "main_cloud");
        }
    }
}

bool Visualizer::saveScreenshot(const std::string& filename) {
    if (!viewer_) {
        std::cerr << "错误: 可视化器未初始化" << std::endl;
        return false;
    }
    
    viewer_->saveScreenshot(filename);
    std::cout << "截屏已保存: " << filename << std::endl;
    return true;
}

void Visualizer::addText(const std::string& text, int x, int y, 
                        const std::string& textId, int fontSize) {
    if (viewer_) {
        if (viewer_->contains(textId)) {
            viewer_->removeText3D(textId);
        }
        viewer_->addText(text, x, y, fontSize, 1.0, 1.0, 1.0, textId);
    }
}

void Visualizer::removeText(const std::string& textId) {
    if (viewer_ && viewer_->contains(textId)) {
        viewer_->removeText3D(textId);
    }
}

void Visualizer::registerKeyboardCallback() {
    if (viewer_) {
        viewer_->registerKeyboardCallback(keyboardEventCallback, this);
    }
}

void Visualizer::keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void* cookie) {
    auto* visualizer = static_cast<Visualizer*>(cookie);
    if (visualizer) {
        visualizer->handleKeyboardEvent(event);
    }
}

void Visualizer::handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event) {
    if (event.keyDown()) {
        switch (event.getKeyCode()) {
            case 'r':
            case 'R':
                resetCamera();
                break;
            case 's':
            case 'S': {
                auto now = std::chrono::system_clock::now();
                auto timeT = std::chrono::system_clock::to_time_t(now);
                auto tm = *std::localtime(&timeT);
                char buffer[64];
                std::strftime(buffer, sizeof(buffer), "screenshot_%Y%m%d_%H%M%S.png", &tm);
                saveScreenshot(buffer);
                break;
            }
            case 'h':
            case 'H':
                std::cout << "\n=== 快捷键帮助 ===" << std::endl;
                std::cout << "r: 重置摄像机视角" << std::endl;
                std::cout << "s: 保存截屏" << std::endl;
                std::cout << "h: 显示此帮助信息" << std::endl;
                std::cout << "q: 退出程序" << std::endl;
                std::cout << "================\n" << std::endl;
                break;
        }
    }
}

} // namespace visualization
} // namespace pcl_viz