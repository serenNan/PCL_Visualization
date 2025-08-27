#include "PointCloud.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace pcl_viz {
namespace core {

PointCloud::PointCloud() 
    : pclPointCloud_(PCLPointCloud::Ptr(new PCLPointCloud))
    , metadataDirty_(true) {
    statistics_.loadTime = std::chrono::system_clock::now();
}

PointCloud::PointCloud(PCLPointCloud::Ptr pclCloud, const std::string& sourceFile)
    : pclPointCloud_(pclCloud ? pclCloud : PCLPointCloud::Ptr(new PCLPointCloud))
    , metadataDirty_(true) {
    statistics_.loadTime = std::chrono::system_clock::now();
    statistics_.sourceFile = sourceFile;
    
    if (pclPointCloud_ && !pclPointCloud_->empty()) {
        updateMetadata(true);
    }
}

bool PointCloud::empty() const {
    return !pclPointCloud_ || pclPointCloud_->empty();
}

size_t PointCloud::size() const {
    return pclPointCloud_ ? pclPointCloud_->size() : 0;
}

void PointCloud::updateMetadata(bool forceUpdate) {
    if (!metadataDirty_ && !forceUpdate) {
        return;
    }

    if (empty()) {
        statistics_.pointCount = 0;
        statistics_.hasDenseData = true;
        statistics_.hasNormals = false;
        statistics_.averagePointDistance = 0.0;
        statistics_.invalidPointCount = 0;
        metadataDirty_ = false;
        return;
    }

    computeStatistics();
    computeBoundingBox();
    
    metadataDirty_ = false;
}

void PointCloud::computeBoundingBox() {
    if (empty()) {
        return;
    }

    pcl::getMinMax3D(*pclPointCloud_, boundingBox_.minPoint, boundingBox_.maxPoint);
}

void PointCloud::computeStatistics() {
    if (empty()) {
        return;
    }

    statistics_.pointCount = pclPointCloud_->size();
    statistics_.hasDenseData = pclPointCloud_->is_dense;
    statistics_.hasNormals = true; // PointNormal 类型总是有法向量
    statistics_.invalidPointCount = 0;

    // 计算无效点数量
    for (const auto& point : pclPointCloud_->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            !std::isfinite(point.normal_x) || !std::isfinite(point.normal_y) || !std::isfinite(point.normal_z)) {
            statistics_.invalidPointCount++;
        }
    }

    // 计算平均点距离（简化版本，仅计算前100个点）
    if (pclPointCloud_->size() > 1) {
        double totalDistance = 0.0;
        size_t sampleSize = std::min(size_t(100), pclPointCloud_->size() - 1);
        
        for (size_t i = 0; i < sampleSize; ++i) {
            const auto& p1 = pclPointCloud_->points[i];
            const auto& p2 = pclPointCloud_->points[i + 1];
            
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double dz = p2.z - p1.z;
            
            totalDistance += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        statistics_.averagePointDistance = totalDistance / sampleSize;
    }

    // 更新密集性标志
    statistics_.hasDenseData = (statistics_.invalidPointCount == 0);
}

void PointCloud::printInfo() const {
    if (metadataDirty_) {
        const_cast<PointCloud*>(this)->updateMetadata();
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n=== 点云信息 ===" << std::endl;
    
    if (!statistics_.sourceFile.empty()) {
        std::cout << "源文件: " << statistics_.sourceFile << std::endl;
    }
    
    std::cout << "点数量: " << statistics_.pointCount << std::endl;
    std::cout << "是否密集: " << (statistics_.hasDenseData ? "是" : "否") << std::endl;
    std::cout << "包含法向量: " << (statistics_.hasNormals ? "是" : "否") << std::endl;
    
    if (statistics_.invalidPointCount > 0) {
        std::cout << "无效点数: " << statistics_.invalidPointCount << std::endl;
    }
    
    if (!empty()) {
        const auto& bbox = getBoundingBox();
        std::cout << "\n--- 边界框 ---" << std::endl;
        std::cout << "X 范围: [" << bbox.minPoint.x << ", " << bbox.maxPoint.x 
                  << "] (跨度: " << bbox.getXSpan() << ")" << std::endl;
        std::cout << "Y 范围: [" << bbox.minPoint.y << ", " << bbox.maxPoint.y 
                  << "] (跨度: " << bbox.getYSpan() << ")" << std::endl;
        std::cout << "Z 范围: [" << bbox.minPoint.z << ", " << bbox.maxPoint.z 
                  << "] (跨度: " << bbox.getZSpan() << ")" << std::endl;
        
        const auto center = bbox.getCenter();
        std::cout << "中心点: (" << center.x << ", " << center.y << ", " << center.z << ")" << std::endl;
        
        if (statistics_.averagePointDistance > 0.0) {
            std::cout << "平均点距离: " << statistics_.averagePointDistance << std::endl;
        }
    }
    
    // 显示加载时间
    auto timeT = std::chrono::system_clock::to_time_t(statistics_.loadTime);
    std::cout << "加载时间: " << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S") << std::endl;
    
    std::cout << "===============\n" << std::endl;
}

void PointCloud::setSourceFile(const std::string& sourceFile) {
    statistics_.sourceFile = sourceFile;
}

} // namespace core
} // namespace pcl_viz