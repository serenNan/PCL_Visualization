#pragma once

#include "AnalysisResult.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

namespace pcl_viz {
namespace analysis {

using PointT = pcl::PointNormal;
using PCLPointCloud = pcl::PointCloud<PointT>;

/**
 * @brief 几何计算方法枚举
 */
enum class VolumeCalculationMethod {
    CONVEX_HULL,    ///< 使用凸包计算
    CONCAVE_HULL,   ///< 使用凹包计算  
    DELAUNAY_3D,    ///< 使用Delaunay三角化
    GRID_BASED,     ///< 基于网格的方法
    SIMPLE_AVERAGE  ///< 简单平均深度方法
};

enum class AreaCalculationMethod {
    CONVEX_HULL_2D, ///< 2D凸包投影
    CONCAVE_HULL_2D,///< 2D凹包投影
    TRIANGULATION,  ///< 三角网格面积
    BOUNDING_BOX    ///< 边界框面积
};

/**
 * @brief 几何计算配置参数
 */
struct GeometryCalculationParams {
    VolumeCalculationMethod volumeMethod = VolumeCalculationMethod::CONVEX_HULL;
    AreaCalculationMethod areaMethod = AreaCalculationMethod::CONVEX_HULL_2D;
    
    // 凹包参数
    double concaveHullAlpha = 0.1;
    
    // 网格化参数
    double gridResolution = 0.01;
    
    // 三角化参数
    double triangulationRadius = 0.025;
    int maxNearestNeighbors = 100;
    
    // 质量控制
    bool enableQualityCheck = true;
    double minValidArea = 1e-6;
    double minValidVolume = 1e-9;
};

/**
 * @brief 几何计算结果
 */
struct GeometryCalculationResult {
    // 面积计算结果
    double projectionArea = 0.0;        ///< 投影面积 (m²)
    double surfaceArea = 0.0;           ///< 表面面积 (m²)
    double convexHullArea = 0.0;        ///< 凸包面积 (m²)
    double concaveHullArea = 0.0;       ///< 凹包面积 (m²)
    
    // 体积计算结果
    double volume = 0.0;                ///< 体积 (m³)
    double convexHullVolume = 0.0;      ///< 凸包体积 (m³)
    double approximateVolume = 0.0;     ///< 近似体积 (m³)
    
    // 深度信息
    double avgDepth = 0.0;              ///< 平均深度 (m)
    double maxDepth = 0.0;              ///< 最大深度 (m)
    double minDepth = 0.0;              ///< 最小深度 (m)
    double depthVariance = 0.0;         ///< 深度方差
    
    // 尺寸信息
    double width = 0.0;                 ///< 宽度 (m)
    double length = 0.0;                ///< 长度 (m)
    double height = 0.0;                ///< 高度 (m)
    
    // 几何特征
    double aspectRatio = 0.0;           ///< 长宽比
    double compactness = 0.0;           ///< 紧凑度 (4π*Area/Perimeter²)
    double roundness = 0.0;             ///< 圆形度
    double elongation = 0.0;            ///< 伸长度
    
    // 质量指标
    bool calculationSuccessful = false;  ///< 计算是否成功
    double calculationConfidence = 0.0;  ///< 计算置信度 [0,1]
    std::string errorMessage;            ///< 错误信息
    
    // 调试信息
    size_t triangleCount = 0;           ///< 三角形数量
    size_t hullPointCount = 0;          ///< 凸包点数量
};

/**
 * @brief 几何计算器类
 * 
 * 提供高精度的几何参数计算功能，支持多种算法
 * 可独立使用或与PotholeDetector配合使用
 */
class GeometryCalculator {
public:
    /**
     * @brief 构造函数
     * @param params 计算参数
     */
    explicit GeometryCalculator(const GeometryCalculationParams& params = GeometryCalculationParams());
    
    /**
     * @brief 析构函数
     */
    ~GeometryCalculator() = default;
    
    // 禁用拷贝，启用移动
    GeometryCalculator(const GeometryCalculator&) = delete;
    GeometryCalculator& operator=(const GeometryCalculator&) = delete;
    GeometryCalculator(GeometryCalculator&&) = default;
    GeometryCalculator& operator=(GeometryCalculator&&) = default;
    
    /**
     * @brief 设置计算参数
     * @param params 新的计算参数
     */
    void setCalculationParams(const GeometryCalculationParams& params);
    
    /**
     * @brief 计算凹坑的完整几何参数
     * @param potholePoints 凹坑点云
     * @param surfaceHeight 参考表面高度
     * @return 几何计算结果
     */
    GeometryCalculationResult calculateGeometry(PCLPointCloud::Ptr potholePoints, double surfaceHeight = 0.0);
    
    /**
     * @brief 计算投影面积
     * @param points 点云
     * @param method 计算方法
     * @return 面积 (m²)
     */
    double calculateProjectionArea(PCLPointCloud::Ptr points, AreaCalculationMethod method = AreaCalculationMethod::CONVEX_HULL_2D);
    
    /**
     * @brief 计算体积
     * @param points 点云
     * @param surfaceHeight 表面高度
     * @param method 计算方法
     * @return 体积 (m³)
     */
    double calculateVolume(PCLPointCloud::Ptr points, double surfaceHeight, VolumeCalculationMethod method = VolumeCalculationMethod::CONVEX_HULL);
    
    /**
     * @brief 计算深度统计信息
     * @param points 点云
     * @param surfaceHeight 表面高度
     * @return {平均深度, 最大深度, 最小深度, 深度方差}
     */
    std::tuple<double, double, double, double> calculateDepthStatistics(PCLPointCloud::Ptr points, double surfaceHeight);
    
    /**
     * @brief 计算边界框尺寸
     * @param points 点云  
     * @return {宽度, 长度, 高度}
     */
    std::tuple<double, double, double> calculateBoundingBoxSize(PCLPointCloud::Ptr points);
    
    /**
     * @brief 计算几何形状特征
     * @param points 点云
     * @return {长宽比, 紧凑度, 圆形度, 伸长度}
     */
    std::tuple<double, double, double, double> calculateShapeFeatures(PCLPointCloud::Ptr points);
    
    /**
     * @brief 估算表面高度
     * @param allPoints 所有点云数据
     * @param method 估算方法 ("percentile", "mean", "ransac")
     * @param percentile 百分位数（用于percentile方法）
     * @return 估算的表面高度
     */
    static double estimateSurfaceHeight(PCLPointCloud::Ptr allPoints, const std::string& method = "percentile", double percentile = 75.0);

private:
    /**
     * @brief 使用凸包计算2D投影面积
     * @param points 点云
     * @return 面积
     */
    double calculateConvexHull2DArea(PCLPointCloud::Ptr points);
    
    /**
     * @brief 使用凹包计算2D投影面积
     * @param points 点云
     * @return 面积
     */
    double calculateConcaveHull2DArea(PCLPointCloud::Ptr points);
    
    /**
     * @brief 使用三角网格计算表面面积
     * @param points 点云
     * @return 面积
     */
    double calculateTriangulationArea(PCLPointCloud::Ptr points);
    
    /**
     * @brief 使用凸包方法计算体积
     * @param points 点云
     * @param surfaceHeight 表面高度
     * @return 体积
     */
    double calculateConvexHullVolume(PCLPointCloud::Ptr points, double surfaceHeight);
    
    /**
     * @brief 使用基于网格的方法计算体积
     * @param points 点云
     * @param surfaceHeight 表面高度
     * @return 体积
     */
    double calculateGridBasedVolume(PCLPointCloud::Ptr points, double surfaceHeight);
    
    /**
     * @brief 使用简单平均方法计算体积
     * @param points 点云
     * @param surfaceHeight 表面高度
     * @return 体积
     */
    double calculateSimpleAverageVolume(PCLPointCloud::Ptr points, double surfaceHeight);
    
    /**
     * @brief 计算紧凑度（4π*Area/Perimeter²）
     * @param points 点云
     * @return 紧凑度
     */
    double calculateCompactness(PCLPointCloud::Ptr points);
    
    /**
     * @brief 计算圆形度
     * @param points 点云
     * @return 圆形度
     */
    double calculateRoundness(PCLPointCloud::Ptr points);
    
    /**
     * @brief 验证计算结果的合理性
     * @param result 计算结果
     * @return 是否通过验证
     */
    bool validateCalculationResult(GeometryCalculationResult& result);
    
    /**
     * @brief 计算结果的置信度评估
     * @param result 计算结果
     * @param pointCount 输入点数
     * @return 置信度 [0,1]
     */
    double assessCalculationConfidence(const GeometryCalculationResult& result, size_t pointCount);

private:
    GeometryCalculationParams params_;  ///< 计算参数
};

/**
 * @brief 创建默认的几何计算参数
 * @return 默认参数配置
 */
GeometryCalculationParams createDefaultGeometryParams();

/**
 * @brief 创建高精度几何计算参数
 * @return 高精度参数配置
 */
GeometryCalculationParams createHighPrecisionGeometryParams();

/**
 * @brief 创建快速几何计算参数
 * @return 快速计算参数配置
 */
GeometryCalculationParams createFastGeometryParams();

} // namespace analysis
} // namespace pcl_viz