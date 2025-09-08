#pragma once

#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl_viz {
namespace analysis {

using PointT = pcl::PointNormal;
using PCLPointCloud = pcl::PointCloud<PointT>;

/**
 * @brief 单个凹坑的几何信息
 */
struct PotholeInfo {
    // 基本几何参数
    double depth = 0.0;          ///< 平均深度 (m)
    double maxDepth = 0.0;       ///< 最大深度 (m) 
    double volume = 0.0;         ///< 体积 (m³)
    double area = 0.0;           ///< 投影面积 (m²)
    double width = 0.0;          ///< 宽度 (m)
    double length = 0.0;         ///< 长度 (m)
    
    // 位置信息
    PointT center;               ///< 凹坑中心点
    PointT minPoint;             ///< 边界框最小点
    PointT maxPoint;             ///< 边界框最大点
    PointT deepestPoint;         ///< 最深点位置 (新增)
    
    // 统计信息
    size_t pointCount = 0;       ///< 凹坑点数
    double confidence = 0.0;     ///< 检测置信度 [0, 1]
    
    // 点云数据
    PCLPointCloud::Ptr potholePoints;  ///< 凹坑点云
    std::vector<size_t> pointIndices;  ///< 原始点云中的索引
    
    PotholeInfo() {
        potholePoints = boost::make_shared<PCLPointCloud>();
        center.x = center.y = center.z = 0.0f;
        center.normal_x = center.normal_y = 0.0f; center.normal_z = 1.0f;
        minPoint.x = minPoint.y = minPoint.z = std::numeric_limits<float>::max();
        minPoint.normal_x = minPoint.normal_y = 0.0f; minPoint.normal_z = 1.0f;
        maxPoint.x = maxPoint.y = maxPoint.z = std::numeric_limits<float>::lowest();
        maxPoint.normal_x = maxPoint.normal_y = 0.0f; maxPoint.normal_z = 1.0f;
        deepestPoint.x = deepestPoint.y = deepestPoint.z = 0.0f;
        deepestPoint.normal_x = deepestPoint.normal_y = 0.0f; deepestPoint.normal_z = 1.0f;
    }
    
    /**
     * @brief 获取凹坑的长宽比
     * @return 长宽比
     */
    double getAspectRatio() const { 
        return (width > 0) ? (length / width) : 0.0; 
    }
    
    /**
     * @brief 获取平均深度与最大深度的比值
     * @return 深度均匀性指标 [0, 1]
     */
    double getDepthUniformity() const { 
        return (maxDepth > 0) ? (depth / maxDepth) : 0.0; 
    }
};

/**
 * @brief 分析参数配置
 */
struct AnalysisParams {
    // 凹坑检测参数
    double zThresholdPercentile = 20.0;   ///< Z轴阈值百分位数 [0, 100]
    double minPotholeArea = 0.001;        ///< 最小凹坑面积 (m²)
    size_t minPotholePoints = 10;         ///< 最小凹坑点数
    double clusteringRadius = 0.05;       ///< 聚类半径 (m)
    
    // RANSAC平面拟合参数
    bool useRANSACFitting = true;         ///< 是否使用RANSAC平面拟合
    double ransacThreshold = 0.01;        ///< RANSAC距离阈值 (m)
    int ransacMaxIterations = 1000;       ///< RANSAC最大迭代次数
    
    // 几何计算参数
    bool useConvexHull = true;            ///< 使用凸包计算面积
    bool enableMultipleDetection = true;  ///< 启用多凹坑检测
    double noiseFilterRadius = 0.02;      ///< 噪声滤波半径 (m)
    
    // 质量控制参数
    double minConfidenceThreshold = 0.3;  ///< 最小置信度阈值
    bool enableQualityFiltering = true;   ///< 启用质量过滤
    
    // 中央区域过滤参数 (新增)
    bool enableCentralRegionFilter = false;  ///< 启用中央区域过滤
    double centralRegionRatio = 0.6;          ///< 中央区域占比 [0.1, 1.0]
    bool detectSingleMaxPothole = false;     ///< 只检测最大单个凹坑
};

/**
 * @brief 分析结果汇总
 */
struct AnalysisResult {
    // 元数据
    std::chrono::system_clock::time_point analysisTime;  ///< 分析时间
    std::string sourceFile;                              ///< 源文件路径
    AnalysisParams params;                               ///< 使用的分析参数
    
    // 输入数据统计
    size_t totalPointCount = 0;          ///< 总点数
    double totalArea = 0.0;              ///< 总面积 (m²)
    double surfaceRoughness = 0.0;       ///< 表面粗糙度
    
    // 检测结果
    std::vector<PotholeInfo> potholes;   ///< 检测到的凹坑列表
    size_t validPotholeCount = 0;        ///< 有效凹坑数量
    
    // 汇总统计
    double totalPotholeArea = 0.0;       ///< 总凹坑面积 (m²)
    double totalPotholeVolume = 0.0;     ///< 总凹坑体积 (m³)
    double avgPotholeDepth = 0.0;        ///< 平均凹坑深度 (m)
    double maxPotholeDepth = 0.0;        ///< 最大凹坑深度 (m)
    
    // 质量指标
    bool analysisSuccessful = false;     ///< 分析是否成功
    double overallConfidence = 0.0;      ///< 整体置信度 [0, 1]
    std::string errorMessage;            ///< 错误信息（如有）
    double processingTimeMs = 0.0;       ///< 处理时间 (ms)
    
    AnalysisResult() {
        analysisTime = std::chrono::system_clock::now();
    }
    
    /**
     * @brief 获取有效凹坑数量
     * @return 有效凹坑数量
     */
    size_t getValidPotholeCount() const {
        return validPotholeCount;
    }
    
    /**
     * @brief 获取路面损伤率
     * @return 损伤率百分比
     */
    double getDamageRatio() const {
        return (totalArea > 0) ? (totalPotholeArea / totalArea * 100.0) : 0.0;
    }
    
    /**
     * @brief 计算汇总统计信息
     */
    void computeSummaryStatistics() {
        validPotholeCount = 0;
        totalPotholeArea = 0.0;
        totalPotholeVolume = 0.0;
        avgPotholeDepth = 0.0;
        maxPotholeDepth = 0.0;
        
        if (potholes.empty()) return;
        
        // * 不再计算平均深度，只处理最大深度
        for (const auto& pothole : potholes) {
            if (pothole.confidence >= params.minConfidenceThreshold) {
                validPotholeCount++;
                totalPotholeArea += pothole.area;
                totalPotholeVolume += pothole.volume;
                maxPotholeDepth = std::max(maxPotholeDepth, pothole.maxDepth);
            }
        }
        
        // 平均深度设为0，因为不再使用
        avgPotholeDepth = 0.0;
    }
    
    /**
     * @brief 导出为JSON字符串
     * @param prettyFormat 是否使用美化格式
     * @return JSON字符串
     */
    std::string toJson(bool prettyFormat = true) const;
    
    /**
     * @brief 从JSON字符串加载
     * @param jsonStr JSON字符串
     * @return 是否成功加载
     */
    bool fromJson(const std::string& jsonStr);
    
    /**
     * @brief 导出为CSV格式（凹坑列表）
     * @return CSV字符串
     */
    std::string toCsv() const;
    
    /**
     * @brief 导出为文本报告
     * @return 文本报告
     */
    std::string toTextReport() const;
};

} // namespace analysis
} // namespace pcl_viz