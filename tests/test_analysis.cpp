#include <iostream>
#include <memory>
#include "../src/core/PointCloudLoader.h"
#include "../src/core/PointCloud.h"
#include "../src/analysis/PotholeDetector.h"

int main() {
    std::cout << "[DEBUG] 测试分析功能开始..." << std::endl;
    
    // 1. 加载点云
    pcl_viz::core::PCLPointCloud::Ptr pclCloud(new pcl_viz::core::PCLPointCloud);
    std::string filename = "../../data/H103v2.asc";
    
    std::cout << "[DEBUG] 加载点云文件: " << filename << std::endl;
    if (!pcl_viz::core::PointCloudLoader::loadPointCloud(filename, pclCloud)) {
        std::cerr << "[ERROR] 无法加载点云文件" << std::endl;
        return -1;
    }
    
    auto pointCloud = std::make_shared<pcl_viz::core::PointCloud>(pclCloud, filename);
    std::cout << "[DEBUG] 点云加载成功，点数: " << pointCloud->size() << std::endl;
    
    // 2. 创建分析器
    pcl_viz::analysis::PotholeDetector detector;
    detector.setProgressCallback([](const std::string& stage, int progress, const std::string& message) {
        std::cout << "[PROGRESS] " << stage << " (" << progress << "%): " << message << std::endl;
    });
    
    // 3. 执行分析
    std::cout << "[DEBUG] 开始分析..." << std::endl;
    auto result = detector.analyze(pointCloud);
    
    // 4. 显示结果
    std::cout << "[DEBUG] 分析完成!" << std::endl;
    std::cout << "[DEBUG] 分析成功: " << (result.analysisSuccessful ? "是" : "否") << std::endl;
    std::cout << "[DEBUG] 凹坑数量: " << result.potholes.size() << std::endl;
    std::cout << "[DEBUG] 有效凹坑数量: " << result.validPotholeCount << std::endl;
    
    if (!result.potholes.empty()) {
        const auto& firstPothole = result.potholes[0];
        std::cout << "[DEBUG] 第一个凹坑信息:" << std::endl;
        std::cout << "  深度: " << firstPothole.depth << " m" << std::endl;
        std::cout << "  体积: " << firstPothole.volume << " m³" << std::endl;
        std::cout << "  面积: " << firstPothole.area << " m²" << std::endl;
        std::cout << "  宽度: " << firstPothole.width << " m" << std::endl;
        std::cout << "  长度: " << firstPothole.length << " m" << std::endl;
        
        // 转换为毫米单位并显示
        std::cout << "[DEBUG] 毫米单位转换:" << std::endl;
        std::cout << "  深度: " << firstPothole.depth * 1000.0 << " mm" << std::endl;
        std::cout << "  体积: " << firstPothole.volume * 1e9 << " mm³" << std::endl;
        std::cout << "  面积: " << firstPothole.area * 1e6 << " mm²" << std::endl;
    }
    
    if (!result.errorMessage.empty()) {
        std::cout << "[DEBUG] 错误消息: " << result.errorMessage << std::endl;
    }
    
    return 0;
}