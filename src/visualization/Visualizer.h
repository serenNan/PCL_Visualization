#pragma once

#include <string>
#include <memory>

#include <pcl/visualization/pcl_visualizer.h>
#include "../core/PointCloud.h"

namespace pcl_viz {
namespace visualization {

/**
 * @brief 可视化配置结构
 */
struct VisualizationConfig {
    // 窗口设置
    std::string windowTitle = "PCL 点云可视化器";
    int windowWidth = 800;
    int windowHeight = 600;
    
    // 渲染设置
    float pointSize = 2.0f;
    bool showCoordinateSystem = true;
    float coordinateSystemScale = 1.0f;
    
    // 背景颜色 (RGB [0-1])
    double backgroundColor[3] = {0.0, 0.0, 0.0}; // 黑色
    
    // 相机设置
    bool autoSetCamera = true;
    double cameraDistance = 5.0;
    
    // 颜色映射
    enum class ColorMode {
        HEIGHT_Z,      // 根据Z坐标着色
        UNIFORM,       // 统一颜色
        NORMAL_X,      // 根据X法向量着色
        NORMAL_Y,      // 根据Y法向量着色
        NORMAL_Z       // 根据Z法向量着色
    };
    ColorMode colorMode = ColorMode::HEIGHT_Z;
    
    // 统一颜色设置 (仅在 UNIFORM 模式下使用)
    double uniformColor[3] = {1.0, 1.0, 1.0}; // 白色
};

/**
 * @brief 点云可视化器类
 * 
 * 封装 PCL Visualizer 功能，提供简单易用的点云可视化接口
 * 支持多种渲染模式和交互功能
 */
class Visualizer {
public:
    /**
     * @brief 构造函数
     * @param config 可视化配置
     */
    explicit Visualizer(const VisualizationConfig& config = VisualizationConfig{});

    /**
     * @brief 析构函数
     */
    ~Visualizer();

    // 禁用拷贝构造和拷贝赋值
    Visualizer(const Visualizer&) = delete;
    Visualizer& operator=(const Visualizer&) = delete;

    // 启用移动构造和移动赋值
    Visualizer(Visualizer&&) noexcept = default;
    Visualizer& operator=(Visualizer&&) noexcept = default;

    /**
     * @brief 设置要显示的点云
     * @param pointCloud 点云对象
     * @param cloudId 点云标识符，用于多点云管理
     * @return 是否设置成功
     */
    bool setPointCloud(const std::shared_ptr<core::PointCloud>& pointCloud, 
                       const std::string& cloudId = "main_cloud");

    /**
     * @brief 移除指定的点云
     * @param cloudId 点云标识符
     * @return 是否移除成功
     */
    bool removePointCloud(const std::string& cloudId = "main_cloud");

    /**
     * @brief 启动可视化器主循环
     * @param blocking 是否阻塞执行
     */
    void run(bool blocking = true);

    /**
     * @brief 执行一次渲染循环
     * @param timeMs 等待时间（毫秒）
     * @return 是否继续运行
     */
    bool spinOnce(int timeMs = 100);

    /**
     * @brief 检查可视化器是否被停止
     * @return 是否停止
     */
    bool wasStopped() const;

    /**
     * @brief 重置摄像机视角
     */
    void resetCamera();

    /**
     * @brief 更新可视化配置
     * @param config 新的配置
     */
    void updateConfig(const VisualizationConfig& config);

    /**
     * @brief 获取当前配置
     * @return 当前配置
     */
    const VisualizationConfig& getConfig() const { return config_; }

    /**
     * @brief 截屏保存
     * @param filename 保存的文件名
     * @return 是否保存成功
     */
    bool saveScreenshot(const std::string& filename);

    /**
     * @brief 添加文本显示
     * @param text 文本内容
     * @param x X坐标（像素）
     * @param y Y坐标（像素）
     * @param textId 文本标识符
     * @param fontSize 字体大小
     */
    void addText(const std::string& text, int x, int y, 
                 const std::string& textId = "info_text", int fontSize = 12);

    /**
     * @brief 移除文本
     * @param textId 文本标识符
     */
    void removeText(const std::string& textId = "info_text");

    /**
     * @brief 获取PCL可视化器指针（高级用法）
     * @return PCL可视化器智能指针
     */
    pcl::visualization::PCLVisualizer::Ptr getViewer() { return viewer_; }

private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;  ///< PCL 可视化器
    VisualizationConfig config_;                     ///< 可视化配置
    std::shared_ptr<core::PointCloud> currentPointCloud_; ///< 当前显示的点云

    /**
     * @brief 初始化可视化器
     */
    void initializeViewer();

    /**
     * @brief 设置摄像机位置
     * @param pointCloud 点云对象，用于计算最佳视角
     */
    void setupCamera(const std::shared_ptr<core::PointCloud>& pointCloud);

    /**
     * @brief 应用点云颜色映射
     * @param pointCloud 点云对象
     * @param cloudId 点云标识符
     */
    void applyColorMapping(const std::shared_ptr<core::PointCloud>& pointCloud, 
                          const std::string& cloudId);

    /**
     * @brief 注册键盘回调函数
     */
    void registerKeyboardCallback();

    /**
     * @brief 键盘事件处理函数
     * @param event 键盘事件
     * @param cookie 用户数据
     */
    static void keyboardEventCallback(const pcl::visualization::KeyboardEvent& event, void* cookie);

    /**
     * @brief 处理键盘事件
     * @param event 键盘事件
     */
    void handleKeyboardEvent(const pcl::visualization::KeyboardEvent& event);
};

} // namespace visualization
} // namespace pcl_viz