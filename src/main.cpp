#include <iostream>
#include <memory>
#include <string>

#include "core/PointCloudLoader.h"
#include "core/PointCloud.h"
#include "visualization/Visualizer.h"

/**
 * @brief 显示程序使用方法
 * @param programName 程序名称
 */
void showUsage(const std::string& programName) {
    std::cout << "使用方法: " << programName << " [点云文件路径]" << std::endl;
    std::cout << "示例:" << std::endl;
    std::cout << "  " << programName << " ../data/H103v2.asc" << std::endl;
    std::cout << "  " << programName << "  # 使用默认文件" << std::endl;
    std::cout << "\n支持的文件格式:" << std::endl;
    std::cout << "  - .asc (Geomagic Studio ASCII format)" << std::endl;
}

int main(int argc, char** argv) {
    using namespace pcl_viz;
    
    // 解析命令行参数
    std::string filename = "../../data/H103v2.asc";  // 默认文件路径
    
    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--help" || arg == "-h") {
            showUsage(argv[0]);
            return 0;
        }
        filename = arg;
    }
    
    std::cout << "=== PCL 点云可视化程序 ===" << std::endl;
    std::cout << "正在加载点云文件: " << filename << std::endl;
    
    try {
        // 步骤 1: 创建 PCL 点云对象
        core::PCLPointCloud::Ptr pclCloud(new core::PCLPointCloud);
        
        // 步骤 2: 使用加载器加载点云数据
        if (!core::PointCloudLoader::loadPointCloud(filename, pclCloud)) {
            std::cerr << "错误: 无法加载点云文件 " << filename << std::endl;
            std::cerr << "请检查文件路径是否正确，或使用 --help 查看使用方法" << std::endl;
            return -1;
        }
        
        // 步骤 3: 创建点云封装对象
        auto pointCloud = std::make_shared<core::PointCloud>(pclCloud, filename);
        
        // 步骤 4: 显示点云信息
        pointCloud->printInfo();
        
        // 步骤 5: 创建可视化器配置
        visualization::VisualizationConfig config;
        config.windowTitle = "PCL 点云可视化器 - " + filename;
        config.pointSize = 2.0f;
        config.colorMode = visualization::VisualizationConfig::ColorMode::HEIGHT_Z;
        config.showCoordinateSystem = true;
        config.coordinateSystemScale = 1.0f;
        
        // 步骤 6: 创建可视化器
        visualization::Visualizer visualizer(config);
        
        // 步骤 7: 设置点云到可视化器
        if (!visualizer.setPointCloud(pointCloud)) {
            std::cerr << "错误: 无法设置点云到可视化器" << std::endl;
            return -1;
        }
        
        // 步骤 8: 运行可视化器
        visualizer.run(true);  // 阻塞模式运行
        
    } catch (const std::exception& e) {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "未知异常发生" << std::endl;
        return -1;
    }
    
    std::cout << "程序正常结束" << std::endl;
    return 0;
}