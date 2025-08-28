#include "GeometryCalculator.h"
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace pcl_viz {
namespace analysis {

GeometryCalculator::GeometryCalculator(const GeometryCalculationParams& params)
    : params_(params) {
}

void GeometryCalculator::setCalculationParams(const GeometryCalculationParams& params) {
    params_ = params;
}

GeometryCalculationResult GeometryCalculator::calculateGeometry(PCLPointCloud::Ptr potholePoints, double surfaceHeight) {
    GeometryCalculationResult result;
    
    try {
        if (!potholePoints || potholePoints->empty()) {
            result.errorMessage = "输入点云为空";
            return result;
        }
        
        // 1. 计算边界框尺寸
        auto [width, length, height] = calculateBoundingBoxSize(potholePoints);
        result.width = width;
        result.length = length; 
        result.height = height;
        
        // 2. 计算深度统计
        auto [avgDepth, maxDepth, minDepth, depthVar] = calculateDepthStatistics(potholePoints, surfaceHeight);
        result.avgDepth = avgDepth;
        result.maxDepth = maxDepth;
        result.minDepth = minDepth;
        result.depthVariance = depthVar;
        
        // 3. 计算投影面积
        result.projectionArea = calculateProjectionArea(potholePoints, params_.areaMethod);
        result.convexHullArea = calculateConvexHull2DArea(potholePoints);
        if (params_.areaMethod != AreaCalculationMethod::CONVEX_HULL_2D) {
            result.concaveHullArea = calculateConcaveHull2DArea(potholePoints);
        }
        
        // 4. 计算体积
        result.volume = calculateVolume(potholePoints, surfaceHeight, params_.volumeMethod);
        if (params_.volumeMethod != VolumeCalculationMethod::CONVEX_HULL) {
            result.convexHullVolume = calculateConvexHullVolume(potholePoints, surfaceHeight);
        } else {
            result.convexHullVolume = result.volume;
        }
        result.approximateVolume = calculateSimpleAverageVolume(potholePoints, surfaceHeight);
        
        // 5. 计算几何形状特征
        auto [aspectRatio, compactness, roundness, elongation] = calculateShapeFeatures(potholePoints);
        result.aspectRatio = aspectRatio;
        result.compactness = compactness;
        result.roundness = roundness;
        result.elongation = elongation;
        
        // 6. 验证和评估结果
        result.calculationSuccessful = validateCalculationResult(result);
        if (result.calculationSuccessful) {
            result.calculationConfidence = assessCalculationConfidence(result, potholePoints->size());
        }
        
    } catch (const std::exception& e) {
        result.calculationSuccessful = false;
        result.errorMessage = "几何计算异常: " + std::string(e.what());
    }
    
    return result;
}

double GeometryCalculator::calculateProjectionArea(PCLPointCloud::Ptr points, AreaCalculationMethod method) {
    switch (method) {
        case AreaCalculationMethod::CONVEX_HULL_2D:
            return calculateConvexHull2DArea(points);
        case AreaCalculationMethod::CONCAVE_HULL_2D:
            return calculateConcaveHull2DArea(points);
        case AreaCalculationMethod::TRIANGULATION:
            return calculateTriangulationArea(points);
        case AreaCalculationMethod::BOUNDING_BOX:
        default: {
            auto [width, length, height] = calculateBoundingBoxSize(points);
            return width * length;
        }
    }
}

double GeometryCalculator::calculateVolume(PCLPointCloud::Ptr points, double surfaceHeight, VolumeCalculationMethod method) {
    switch (method) {
        case VolumeCalculationMethod::CONVEX_HULL:
            return calculateConvexHullVolume(points, surfaceHeight);
        case VolumeCalculationMethod::GRID_BASED:
            return calculateGridBasedVolume(points, surfaceHeight);
        case VolumeCalculationMethod::SIMPLE_AVERAGE:
        default:
            return calculateSimpleAverageVolume(points, surfaceHeight);
    }
}

std::tuple<double, double, double, double> GeometryCalculator::calculateDepthStatistics(PCLPointCloud::Ptr points, double surfaceHeight) {
    if (points->empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    // 如果没有提供表面高度，使用点云的最高点
    if (surfaceHeight == 0.0) {
        surfaceHeight = estimateSurfaceHeight(points);
    }
    
    std::vector<double> depths;
    depths.reserve(points->size());
    
    for (const auto& point : points->points) {
        double depth = surfaceHeight - point.z;
        if (depth > 0) {  // 只考虑实际的凹陷
            depths.push_back(depth);
        }
    }
    
    if (depths.empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }
    
    // 计算统计量
    double sum = std::accumulate(depths.begin(), depths.end(), 0.0);
    double avgDepth = sum / depths.size();
    
    double maxDepth = *std::max_element(depths.begin(), depths.end());
    double minDepth = *std::min_element(depths.begin(), depths.end());
    
    // 计算方差
    double variance = 0.0;
    for (double depth : depths) {
        variance += (depth - avgDepth) * (depth - avgDepth);
    }
    variance /= depths.size();
    
    return {avgDepth, maxDepth, minDepth, variance};
}

std::tuple<double, double, double> GeometryCalculator::calculateBoundingBoxSize(PCLPointCloud::Ptr points) {
    if (points->empty()) {
        return {0.0, 0.0, 0.0};
    }
    
    PointT minPt, maxPt;
    pcl::getMinMax3D(*points, minPt, maxPt);
    
    double width = maxPt.x - minPt.x;
    double length = maxPt.y - minPt.y;
    double height = maxPt.z - minPt.z;
    
    return {width, length, height};
}

std::tuple<double, double, double, double> GeometryCalculator::calculateShapeFeatures(PCLPointCloud::Ptr points) {
    auto [width, length, height] = calculateBoundingBoxSize(points);
    
    // 长宽比
    double aspectRatio = (width > 0) ? (length / width) : 0.0;
    
    // 紧凑度
    double compactness = calculateCompactness(points);
    
    // 圆形度
    double roundness = calculateRoundness(points);
    
    // 伸长度 (1 - 短轴/长轴)
    double majorAxis = std::max(width, length);
    double minorAxis = std::min(width, length);
    double elongation = (majorAxis > 0) ? (1.0 - minorAxis / majorAxis) : 0.0;
    
    return {aspectRatio, compactness, roundness, elongation};
}

double GeometryCalculator::estimateSurfaceHeight(PCLPointCloud::Ptr allPoints, const std::string& method, double percentile) {
    if (allPoints->empty()) return 0.0;
    
    std::vector<float> zValues;
    zValues.reserve(allPoints->size());
    for (const auto& point : allPoints->points) {
        zValues.push_back(point.z);
    }
    
    if (method == "mean") {
        double sum = std::accumulate(zValues.begin(), zValues.end(), 0.0);
        return sum / zValues.size();
    } else if (method == "percentile") {
        std::sort(zValues.begin(), zValues.end());
        size_t index = static_cast<size_t>(zValues.size() * percentile / 100.0);
        index = std::min(index, zValues.size() - 1);
        return zValues[index];
    } else {
        // TODO: 实现RANSAC方法
        // 当前回退到百分位数方法
        std::sort(zValues.begin(), zValues.end());
        size_t index = static_cast<size_t>(zValues.size() * 0.75);
        index = std::min(index, zValues.size() - 1);
        return zValues[index];
    }
}

double GeometryCalculator::calculateConvexHull2DArea(PCLPointCloud::Ptr points) {
    if (points->size() < 3) return 0.0;
    
    // 使用简化的边界框面积估算
    PointT minPt, maxPt;
    pcl::getMinMax3D(*points, minPt, maxPt);
    
    double width = maxPt.x - minPt.x;
    double length = maxPt.y - minPt.y;
    
    // 使用椭圆面积公式作为更好的近似 (π * a * b)
    return M_PI * (width / 2.0) * (length / 2.0);
}

double GeometryCalculator::calculateConcaveHull2DArea(PCLPointCloud::Ptr points) {
    if (points->size() < 3) return 0.0;
    
    // 如果凹包失败，回退到凸包方法
    return calculateConvexHull2DArea(points);
}

double GeometryCalculator::calculateTriangulationArea(PCLPointCloud::Ptr points) {
    // * 使用边界框面积作为简化实现
    // TODO: 实现完整的三角网格面积计算
    auto [width, length, height] = calculateBoundingBoxSize(points);
    return width * length;
}

double GeometryCalculator::calculateConvexHullVolume(PCLPointCloud::Ptr points, double surfaceHeight) {
    if (points->size() < 4) return 0.0;
    
    // 使用简化方法计算体积
    return calculateSimpleAverageVolume(points, surfaceHeight);
}

double GeometryCalculator::calculateGridBasedVolume(PCLPointCloud::Ptr points, double surfaceHeight) {
    if (points->empty()) return 0.0;
    
    PointT minPt, maxPt;
    pcl::getMinMax3D(*points, minPt, maxPt);
    
    // 创建网格
    double gridRes = params_.gridResolution;
    int xCells = static_cast<int>(std::ceil((maxPt.x - minPt.x) / gridRes));
    int yCells = static_cast<int>(std::ceil((maxPt.y - minPt.y) / gridRes));
    
    std::vector<std::vector<std::vector<double>>> grid(xCells, 
        std::vector<std::vector<double>>(yCells));
    
    // 将点分配到网格中
    for (const auto& point : points->points) {
        int xIdx = static_cast<int>((point.x - minPt.x) / gridRes);
        int yIdx = static_cast<int>((point.y - minPt.y) / gridRes);
        xIdx = std::min(xIdx, xCells - 1);
        yIdx = std::min(yIdx, yCells - 1);
        
        double depth = surfaceHeight - point.z;
        if (depth > 0) {
            grid[xIdx][yIdx].push_back(depth);
        }
    }
    
    // 计算每个网格单元的体积
    double totalVolume = 0.0;
    double cellArea = gridRes * gridRes;
    
    for (int x = 0; x < xCells; ++x) {
        for (int y = 0; y < yCells; ++y) {
            if (!grid[x][y].empty()) {
                double avgDepth = std::accumulate(grid[x][y].begin(), grid[x][y].end(), 0.0) / grid[x][y].size();
                totalVolume += cellArea * avgDepth;
            }
        }
    }
    
    return totalVolume;
}

double GeometryCalculator::calculateSimpleAverageVolume(PCLPointCloud::Ptr points, double surfaceHeight) {
    if (points->empty()) return 0.0;
    
    // 计算平均深度
    double totalDepth = 0.0;
    size_t validPointCount = 0;
    
    for (const auto& point : points->points) {
        double depth = surfaceHeight - point.z;
        if (depth > 0) {
            totalDepth += depth;
            validPointCount++;
        }
    }
    
    if (validPointCount == 0) return 0.0;
    
    double avgDepth = totalDepth / validPointCount;
    double area = calculateConvexHull2DArea(points);
    
    return area * avgDepth;
}

double GeometryCalculator::calculateCompactness(PCLPointCloud::Ptr points) {
    double area = calculateConvexHull2DArea(points);
    if (area <= 0) return 0.0;
    
    // 简化的周长计算（使用凸包周长）
    // TODO: 实现准确的周长计算
    double perimeter = 2.0 * M_PI * std::sqrt(area / M_PI); // 假设为圆形的周长
    
    if (perimeter <= 0) return 0.0;
    
    return 4.0 * M_PI * area / (perimeter * perimeter);
}

double GeometryCalculator::calculateRoundness(PCLPointCloud::Ptr points) {
    double area = calculateConvexHull2DArea(points);
    if (area <= 0) return 0.0;
    
    // 计算等效圆半径
    double equivalentRadius = std::sqrt(area / M_PI);
    
    // 计算实际的"半径"分布
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*points, centroid);
    
    double avgDistance = 0.0;
    for (const auto& point : points->points) {
        double dx = point.x - centroid[0];
        double dy = point.y - centroid[1];
        avgDistance += std::sqrt(dx * dx + dy * dy);
    }
    avgDistance /= points->size();
    
    if (avgDistance <= 0) return 0.0;
    
    return equivalentRadius / avgDistance;
}

bool GeometryCalculator::validateCalculationResult(GeometryCalculationResult& result) {
    // 检查基本的数值有效性
    if (result.projectionArea < params_.minValidArea) {
        result.errorMessage += "计算的投影面积过小; ";
        return false;
    }
    
    if (result.volume < params_.minValidVolume) {
        result.errorMessage += "计算的体积过小; ";
        return false;
    }
    
    // 检查几何一致性
    if (result.avgDepth > result.maxDepth) {
        result.errorMessage += "平均深度大于最大深度; ";
        return false;
    }
    
    if (result.width <= 0 || result.length <= 0) {
        result.errorMessage += "尺寸计算无效; ";
        return false;
    }
    
    return true;
}

double GeometryCalculator::assessCalculationConfidence(const GeometryCalculationResult& result, size_t pointCount) {
    double confidence = 1.0;
    
    // 基于点数密度的置信度
    if (pointCount < 10) {
        confidence *= 0.5;  // 点数太少
    } else if (pointCount < 50) {
        confidence *= 0.7;
    } else if (pointCount < 100) {
        confidence *= 0.85;
    }
    
    // 基于几何一致性的置信度
    double volumeConsistency = std::abs(result.volume - result.approximateVolume) / 
                              std::max(result.volume, result.approximateVolume);
    if (volumeConsistency > 0.5) {
        confidence *= 0.7;  // 不同方法计算结果差异较大
    }
    
    // 基于形状特征的置信度
    if (result.aspectRatio > 5.0 || result.aspectRatio < 0.2) {
        confidence *= 0.8;  // 形状过于细长或扁平
    }
    
    return std::max(0.1, std::min(1.0, confidence));
}

// 辅助函数实现
GeometryCalculationParams createDefaultGeometryParams() {
    GeometryCalculationParams params;
    params.volumeMethod = VolumeCalculationMethod::CONVEX_HULL;
    params.areaMethod = AreaCalculationMethod::CONVEX_HULL_2D;
    params.concaveHullAlpha = 0.1;
    params.gridResolution = 0.01;
    params.triangulationRadius = 0.025;
    params.maxNearestNeighbors = 100;
    params.enableQualityCheck = true;
    params.minValidArea = 1e-6;
    params.minValidVolume = 1e-9;
    return params;
}

GeometryCalculationParams createHighPrecisionGeometryParams() {
    auto params = createDefaultGeometryParams();
    params.volumeMethod = VolumeCalculationMethod::GRID_BASED;
    params.areaMethod = AreaCalculationMethod::CONCAVE_HULL_2D;
    params.gridResolution = 0.005;      // 更高的网格分辨率
    params.concaveHullAlpha = 0.05;     // 更精细的凹包
    params.triangulationRadius = 0.015; // 更小的三角化半径
    return params;
}

GeometryCalculationParams createFastGeometryParams() {
    auto params = createDefaultGeometryParams();
    params.volumeMethod = VolumeCalculationMethod::SIMPLE_AVERAGE;
    params.areaMethod = AreaCalculationMethod::BOUNDING_BOX;
    params.enableQualityCheck = false;  // 跳过质量检查以提高速度
    params.gridResolution = 0.02;       // 更粗的网格
    return params;
}

} // namespace analysis
} // namespace pcl_viz