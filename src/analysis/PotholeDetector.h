#pragma once

#include "AnalysisResult.h"
#include "../core/PointCloud.h"
#include <functional>
#include <memory>
#include <future>

// PCL头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

namespace pcl_viz {
namespace analysis {

/**
 * @brief 进度回调函数类型
 * @param stage 当前阶段名称
 * @param progress 进度百分比 [0, 100]
 * @param message 状态消息
 */
using ProgressCallback = std::function<void(const std::string& stage, int progress, const std::string& message)>;

/**
 * @brief 凹坑检测器类
 * 
 * 实现基于PCL的凹坑检测算法，将Python原型移植到C++
 * 支持多种检测策略：Z阈值法、RANSAC平面拟合、聚类分析
 */
class PotholeDetector {
public:
    /**
     * @brief 默认构造函数
     */
    PotholeDetector();
    
    /**
     * @brief 析构函数
     */
    ~PotholeDetector() = default;
    
    // 禁用拷贝构造和拷贝赋值
    PotholeDetector(const PotholeDetector&) = delete;
    PotholeDetector& operator=(const PotholeDetector&) = delete;
    
    // 启用移动构造和移动赋值
    PotholeDetector(PotholeDetector&&) noexcept = default;
    PotholeDetector& operator=(PotholeDetector&&) noexcept = default;
    
    /**
     * @brief 设置分析参数
     * @param params 分析参数配置
     */
    void setAnalysisParams(const AnalysisParams& params);
    
    /**
     * @brief 获取当前分析参数
     * @return 分析参数配置
     */
    const AnalysisParams& getAnalysisParams() const { return params_; }
    
    /**
     * @brief 设置进度回调函数
     * @param callback 进度回调函数
     */
    void setProgressCallback(ProgressCallback callback) { progressCallback_ = callback; }
    
    /**
     * @brief 执行凹坑检测分析
     * @param pointCloud 输入点云
     * @return 分析结果
     */
    AnalysisResult analyze(std::shared_ptr<core::PointCloud> pointCloud);
    
    /**
     * @brief 异步执行凹坑检测分析
     * @param pointCloud 输入点云
     * @return 异步结果future
     */
    std::future<AnalysisResult> analyzeAsync(std::shared_ptr<core::PointCloud> pointCloud);
    
    /**
     * @brief 取消当前分析任务
     */
    void cancelAnalysis() { cancelled_ = true; }
    
    /**
     * @brief 检查分析是否被取消
     * @return 是否被取消
     */
    bool isCancelled() const { return cancelled_; }

private:
    /**
     * @brief 预处理点云数据
     * @param cloud PCL点云
     * @param result 分析结果
     * @return 预处理后的点云
     */
    PCLPointCloud::Ptr preprocessPointCloud(PCLPointCloud::Ptr cloud, AnalysisResult& result);
    
    /**
     * @brief 提取中央区域点云
     * @param cloud 输入点云
     * @param regionRatio 中央区域占比 [0.1, 1.0]
     * @return 中央区域点索引
     */
    pcl::PointIndices::Ptr extractCentralRegion(PCLPointCloud::Ptr cloud, double regionRatio);
    
    /**
     * @brief 使用Z阈值方法检测凹坑候选点
     * @param cloud 输入点云
     * @param result 分析结果
     * @return 凹坑候选点的索引
     */
    pcl::PointIndices::Ptr detectPotholeCandidatesZThreshold(
        PCLPointCloud::Ptr cloud, AnalysisResult& result);
    
    /**
     * @brief 使用RANSAC平面拟合检测凹坑候选点
     * @param cloud 输入点云
     * @param result 分析结果
     * @return 凹坑候选点的索引
     */
    pcl::PointIndices::Ptr detectPotholeCandidatesRANSAC(
        PCLPointCloud::Ptr cloud, AnalysisResult& result);
    
    /**
     * @brief 对凹坑候选点进行聚类分析
     * @param cloud 输入点云
     * @param candidateIndices 候选点索引
     * @param result 分析结果
     * @return 聚类结果
     */
    std::vector<pcl::PointIndices> clusterPotholeCandidates(
        PCLPointCloud::Ptr cloud, pcl::PointIndices::Ptr candidateIndices, AnalysisResult& result);
    
    /**
     * @brief 从聚类中选择最大的单个凹坑
     * @param cloud 输入点云
     * @param clusters 聚类结果
     * @param surfaceZ 表面高度
     * @return 最大凹坑的聚类索引，如果没有则返回空vector
     */
    std::vector<pcl::PointIndices> selectLargestPothole(
        PCLPointCloud::Ptr cloud, const std::vector<pcl::PointIndices>& clusters, double surfaceZ);
    
    /**
     * @brief 从点云聚类生成凹坑信息
     * @param cloud 输入点云
     * @param clusters 聚类结果
     * @param result 分析结果
     */
    void generatePotholeInfo(PCLPointCloud::Ptr cloud, 
                           const std::vector<pcl::PointIndices>& clusters, AnalysisResult& result);
    
    /**
     * @brief 计算单个凹坑的几何参数
     * @param cloud 输入点云
     * @param indices 凹坑点索引
     * @return 凹坑信息
     */
    PotholeInfo calculatePotholeGeometry(PCLPointCloud::Ptr cloud, const pcl::PointIndices& indices);
    
    /**
     * @brief 计算凹坑的投影面积（使用凸包）
     * @param potholePoints 凹坑点云
     * @return 投影面积
     */
    double calculateProjectionArea(PCLPointCloud::Ptr potholePoints);
    
    /**
     * @brief 计算凹坑体积
     * @param potholePoints 凹坑点云
     * @param surfaceZ 表面平均Z值
     * @return 体积
     */
    double calculateVolume(PCLPointCloud::Ptr potholePoints, double surfaceZ);
    
    /**
     * @brief 计算凹坑的置信度
     * @param pothole 凹坑信息
     * @param totalPoints 总点数
     * @return 置信度 [0, 1]
     */
    double calculateConfidence(const PotholeInfo& pothole, size_t totalPoints);
    
    /**
     * @brief 过滤无效的凹坑
     * @param result 分析结果（会被修改）
     */
    void filterInvalidPotholes(AnalysisResult& result);
    
    /**
     * @brief 计算表面粗糙度
     * @param cloud 点云
     * @return 表面粗糙度
     */
    double calculateSurfaceRoughness(PCLPointCloud::Ptr cloud);
    
    /**
     * @brief 报告进度
     * @param stage 阶段名称
     * @param progress 进度百分比
     * @param message 状态消息
     */
    void reportProgress(const std::string& stage, int progress, const std::string& message = "");
    
    /**
     * @brief 检查并处理取消请求
     * @return 如果被取消则返回true
     */
    bool checkCancellation();

private:
    AnalysisParams params_;                 ///< 分析参数配置
    ProgressCallback progressCallback_;     ///< 进度回调函数
    std::atomic<bool> cancelled_;           ///< 取消标志
    
    // PCL搜索树（用于加速邻域搜索）
    pcl::search::KdTree<PointT>::Ptr searchTree_;
    
    // 统计信息
    mutable std::mutex statsMutex_;         ///< 统计信息互斥锁
    size_t processedPointCount_;            ///< 已处理点数
    size_t totalPointCount_;                ///< 总点数
};

/**
 * @brief 创建默认的分析参数配置
 * @return 默认参数
 */
AnalysisParams createDefaultAnalysisParams();

/**
 * @brief 创建用于小型凹坑检测的参数配置
 * @return 小型凹坑检测参数
 */
AnalysisParams createSensitiveAnalysisParams();

/**
 * @brief 创建用于快速检测的参数配置
 * @return 快速检测参数
 */
AnalysisParams createFastAnalysisParams();

/**
 * @brief 创建专用于中央最大凹坑检测的参数配置
 * @return 中央最大凹坑检测参数
 */
AnalysisParams createCentralMaxPotholeParams();

} // namespace analysis
} // namespace pcl_viz