#include <iostream>
#include <memory>
#include <chrono>

#include "../src/core/PointCloud.h"
#include "../src/core/PointCloudLoader.h"
#include "../src/analysis/PotholeDetector.h"

int main() {
    std::cout << "=== 中央最大凹坑检测测试程序 ===" << std::endl;
    
    // 1. 加载点云数据
    std::cout << "\n1. 加载点云数据..." << std::endl;
    
    // 创建PCL点云对象
    auto pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    
    // 加载点云文件
    bool loadSuccess = pcl_viz::core::PointCloudLoader::loadPointCloud(
        "../../data/H103v2.asc", pclCloud);
    
    if (!loadSuccess || pclCloud->empty()) {
        std::cerr << "错误：无法加载点云数据 ../data/H103v2.asc" << std::endl;
        return -1;
    }
    
    // 创建PointCloud包装对象
    auto pointCloud = std::make_shared<pcl_viz::core::PointCloud>(pclCloud, "../../data/H103v2.asc");
    
    std::cout << "✓ 成功加载 " << pointCloud->size() << " 个点" << std::endl;
    
    // 2. 创建两个检测器进行对比
    std::cout << "\n2. 创建检测器..." << std::endl;
    pcl_viz::analysis::PotholeDetector defaultDetector;
    pcl_viz::analysis::PotholeDetector centralMaxDetector;
    
    // 设置默认参数（传统多凹坑检测）
    auto defaultParams = pcl_viz::analysis::createDefaultAnalysisParams();
    defaultDetector.setAnalysisParams(defaultParams);
    
    // 设置中央最大凹坑参数（新算法）
    auto centralMaxParams = pcl_viz::analysis::createCentralMaxPotholeParams();
    centralMaxDetector.setAnalysisParams(centralMaxParams);
    
    std::cout << "✓ 检测器配置完成" << std::endl;
    std::cout << "  - 默认参数: 多凹坑检测, 无中央区域过滤" << std::endl;
    std::cout << "  - 新参数: 中央区域=" << centralMaxParams.centralRegionRatio * 100 
              << "%, 单个最大凹坑检测" << std::endl;
    
    // 3. 执行传统检测
    std::cout << "\n3. 执行传统多凹坑检测..." << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    auto defaultResult = defaultDetector.analyze(pointCloud);
    auto defaultDuration = std::chrono::high_resolution_clock::now() - startTime;
    
    if (defaultResult.analysisSuccessful) {
        std::cout << "✓ 传统检测完成" << std::endl;
        std::cout << "  - 处理时间: " << defaultResult.processingTimeMs << " ms" << std::endl;
        std::cout << "  - 检测到凹坑数量: " << defaultResult.potholes.size() << std::endl;
        std::cout << "  - 有效凹坑数量: " << defaultResult.validPotholeCount << std::endl;
        if (!defaultResult.potholes.empty()) {
            std::cout << "  - 最大深度: " << defaultResult.maxPotholeDepth << " m" << std::endl;
            std::cout << "  - 总面积: " << defaultResult.totalPotholeArea << " m²" << std::endl;
            std::cout << "  - 总体积: " << defaultResult.totalPotholeVolume << " m³" << std::endl;
        }
    } else {
        std::cout << "✗ 传统检测失败: " << defaultResult.errorMessage << std::endl;
    }
    
    // 4. 执行中央最大凹坑检测
    std::cout << "\n4. 执行中央最大凹坑检测..." << std::endl;
    startTime = std::chrono::high_resolution_clock::now();
    auto centralResult = centralMaxDetector.analyze(pointCloud);
    auto centralDuration = std::chrono::high_resolution_clock::now() - startTime;
    
    if (centralResult.analysisSuccessful) {
        std::cout << "✓ 中央最大凹坑检测完成" << std::endl;
        std::cout << "  - 处理时间: " << centralResult.processingTimeMs << " ms" << std::endl;
        std::cout << "  - 检测到凹坑数量: " << centralResult.potholes.size() << std::endl;
        std::cout << "  - 有效凹坑数量: " << centralResult.validPotholeCount << std::endl;
        
        if (!centralResult.potholes.empty()) {
            const auto& pothole = centralResult.potholes[0];  // 应该只有一个凹坑
            std::cout << "  - 最大凹坑信息:" << std::endl;
            std::cout << "    * 最大深度: " << pothole.maxDepth << " m" << std::endl;
            std::cout << "    * 平均深度: " << pothole.depth << " m (应为0)" << std::endl;
            std::cout << "    * 面积: " << pothole.area << " m²" << std::endl;
            std::cout << "    * 体积: " << pothole.volume << " m³" << std::endl;
            std::cout << "    * 点数: " << pothole.pointCount << std::endl;
            std::cout << "    * 宽度×长度: " << pothole.width << " × " << pothole.length << " m" << std::endl;
            std::cout << "    * 中心位置: (" << pothole.center.x << ", " << pothole.center.y 
                      << ", " << pothole.center.z << ")" << std::endl;
            std::cout << "    * 最深点位置: (" << pothole.deepestPoint.x << ", " << pothole.deepestPoint.y 
                      << ", " << pothole.deepestPoint.z << ")" << std::endl;
            std::cout << "    * 置信度: " << pothole.confidence << std::endl;
        }
    } else {
        std::cout << "✗ 中央最大凹坑检测失败: " << centralResult.errorMessage << std::endl;
    }
    
    // 5. 对比分析
    std::cout << "\n5. 算法对比分析:" << std::endl;
    if (defaultResult.analysisSuccessful && centralResult.analysisSuccessful) {
        auto defaultMs = std::chrono::duration_cast<std::chrono::milliseconds>(defaultDuration).count();
        auto centralMs = std::chrono::duration_cast<std::chrono::milliseconds>(centralDuration).count();
        
        std::cout << "  性能对比:" << std::endl;
        std::cout << "    - 传统算法: " << defaultMs << " ms (" << defaultResult.potholes.size() << " 个凹坑)" << std::endl;
        std::cout << "    - 新算法: " << centralMs << " ms (" << centralResult.potholes.size() << " 个凹坑)" << std::endl;
        
        if (centralMs < defaultMs) {
            double speedup = (double)defaultMs / centralMs;
            std::cout << "    ✓ 新算法速度提升: " << speedup << "x" << std::endl;
        }
        
        std::cout << "  检测结果对比:" << std::endl;
        std::cout << "    - 传统算法检测到 " << defaultResult.potholes.size() << " 个凹坑" << std::endl;
        std::cout << "    - 新算法专注于 " << centralResult.potholes.size() << " 个最大凹坑" << std::endl;
        
        if (!defaultResult.potholes.empty() && !centralResult.potholes.empty()) {
            std::cout << "    - 最大深度对比: " << defaultResult.maxPotholeDepth 
                      << " vs " << centralResult.maxPotholeDepth << " m" << std::endl;
        }
    }
    
    std::cout << "\n=== 测试完成 ===" << std::endl;
    return 0;
}