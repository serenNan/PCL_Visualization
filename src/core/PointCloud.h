#pragma once

#include <string>
#include <memory>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

namespace pcl_viz {
namespace core {

using PointT = pcl::PointNormal;
using PCLPointCloud = pcl::PointCloud<PointT>;

/**
 * @brief 点云数据的边界框信息
 */
struct BoundingBox {
    PointT minPoint;
    PointT maxPoint;
    
    float getXSpan() const { return maxPoint.x - minPoint.x; }
    float getYSpan() const { return maxPoint.y - minPoint.y; }
    float getZSpan() const { return maxPoint.z - minPoint.z; }
    
    PointT getCenter() const {
        PointT center;
        center.x = (minPoint.x + maxPoint.x) / 2.0f;
        center.y = (minPoint.y + maxPoint.y) / 2.0f;
        center.z = (minPoint.z + maxPoint.z) / 2.0f;
        return center;
    }
};

/**
 * @brief 点云统计信息
 */
struct PointCloudStatistics {
    size_t pointCount = 0;
    bool hasDenseData = true;
    bool hasNormals = false;
    std::chrono::system_clock::time_point loadTime;
    std::string sourceFile;
    
    // 可扩展的统计信息
    double averagePointDistance = 0.0;
    size_t invalidPointCount = 0;
};

/**
 * @brief 点云数据封装类
 * 
 * 该类封装了 PCL 点云数据并提供额外的元信息管理功能，
 * 采用 RAII 原则管理资源，使用智能指针确保内存安全。
 */
class PointCloud {
public:
    /**
     * @brief 默认构造函数
     */
    PointCloud();

    /**
     * @brief 从 PCL 点云构造
     * @param pclCloud PCL 点云智能指针
     * @param sourceFile 源文件路径（可选）
     */
    explicit PointCloud(PCLPointCloud::Ptr pclCloud, 
                        const std::string& sourceFile = "");

    /**
     * @brief 析构函数
     */
    ~PointCloud() = default;

    // 禁用拷贝构造和拷贝赋值（避免大数据拷贝）
    PointCloud(const PointCloud&) = delete;
    PointCloud& operator=(const PointCloud&) = delete;

    // 启用移动构造和移动赋值
    PointCloud(PointCloud&&) noexcept = default;
    PointCloud& operator=(PointCloud&&) noexcept = default;

    /**
     * @brief 获取 PCL 点云数据
     * @return PCL 点云智能指针
     */
    PCLPointCloud::Ptr getPCLPointCloud() const { return pclPointCloud_; }

    /**
     * @brief 获取点云统计信息
     * @return 统计信息结构
     */
    const PointCloudStatistics& getStatistics() const { return statistics_; }

    /**
     * @brief 获取边界框信息
     * @return 边界框结构
     */
    const BoundingBox& getBoundingBox() const { return boundingBox_; }

    /**
     * @brief 检查点云是否为空
     * @return 是否为空
     */
    bool empty() const;

    /**
     * @brief 获取点数量
     * @return 点数量
     */
    size_t size() const;

    /**
     * @brief 检查是否有法向量数据
     * @return 是否有法向量
     */
    bool hasNormals() const { return statistics_.hasNormals; }

    /**
     * @brief 更新统计信息和边界框
     * @param forceUpdate 是否强制更新（即使数据未变化）
     */
    void updateMetadata(bool forceUpdate = false);

    /**
     * @brief 打印点云信息到控制台
     */
    void printInfo() const;

    /**
     * @brief 设置源文件路径
     * @param sourceFile 源文件路径
     */
    void setSourceFile(const std::string& sourceFile);

    /**
     * @brief 获取源文件路径
     * @return 源文件路径
     */
    const std::string& getSourceFile() const { return statistics_.sourceFile; }

private:
    PCLPointCloud::Ptr pclPointCloud_;  ///< PCL 点云数据
    PointCloudStatistics statistics_;   ///< 统计信息
    BoundingBox boundingBox_;          ///< 边界框信息
    mutable bool metadataDirty_;       ///< 元数据是否需要更新

    /**
     * @brief 计算边界框
     */
    void computeBoundingBox();

    /**
     * @brief 计算统计信息
     */
    void computeStatistics();
};

} // namespace core
} // namespace pcl_viz