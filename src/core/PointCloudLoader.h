#pragma once

#include <string>
#include <memory>
#include <vector>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl_viz {
namespace core {

using PointT = pcl::PointNormal;
using PCLPointCloud = pcl::PointCloud<PointT>;

/**
 * @brief 点云数据加载器，支持多种格式的点云文件读取
 * 
 * 采用策略模式设计，可以轻松扩展支持其他文件格式
 */
class PointCloudLoader {
public:
    /**
     * @brief 支持的文件格式枚举
     */
    enum class FileFormat {
        ASC,    // ASCII 格式 (Geomagic Studio)
        PCD,    // PCL 原生格式 (计划支持)
        PLY,    // 多边形文件格式 (计划支持)
        AUTO    // 自动检测格式
    };

    /**
     * @brief 加载点云文件
     * @param filename 文件路径
     * @param cloud 输出点云对象
     * @param format 文件格式，默认自动检测
     * @return 是否加载成功
     */
    static bool loadPointCloud(const std::string& filename, 
                              PCLPointCloud::Ptr cloud, 
                              FileFormat format = FileFormat::AUTO);

private:
    /**
     * @brief 从文件扩展名推断格式
     * @param filename 文件名
     * @return 推断的文件格式
     */
    static FileFormat detectFormat(const std::string& filename);

    /**
     * @brief 加载 ASC 格式文件 (Geomagic Studio 格式)
     * @param filename 文件路径
     * @param cloud 输出点云对象
     * @return 是否加载成功
     */
    static bool loadASCFile(const std::string& filename, PCLPointCloud::Ptr cloud);

    /**
     * @brief 验证点云数据的有效性
     * @param cloud 点云对象
     * @return 是否有效
     */
    static bool validatePointCloud(const PCLPointCloud::Ptr cloud);
};

} // namespace core
} // namespace pcl_viz