#include "PotholeDetector.h"
#include <chrono>
#include <algorithm>
#include <numeric>
#include <thread>

namespace pcl_viz {
namespace analysis {

PotholeDetector::PotholeDetector()
    : params_(createDefaultAnalysisParams())
    , cancelled_(false)
    , searchTree_(new pcl::search::KdTree<PointT>())
    , processedPointCount_(0)
    , totalPointCount_(0) {
}

void PotholeDetector::setAnalysisParams(const AnalysisParams& params) {
    params_ = params;
}

AnalysisResult PotholeDetector::analyze(std::shared_ptr<core::PointCloud> pointCloud) {
    auto startTime = std::chrono::high_resolution_clock::now();
    AnalysisResult result;
    
    try {
        // 初始化结果
        result.sourceFile = pointCloud->getSourceFile();
        result.params = params_;
        result.totalPointCount = pointCloud->size();
        totalPointCount_ = pointCloud->size();
        processedPointCount_ = 0;
        cancelled_ = false;
        
        reportProgress("初始化", 0, "开始凹坑检测分析");
        
        if (pointCloud->empty()) {
            result.analysisSuccessful = false;
            result.errorMessage = "输入点云为空";
            return result;
        }
        
        auto pclCloud = pointCloud->getPCLPointCloud();
        
        // 1. 预处理点云
        reportProgress("预处理", 10, "清理噪声和异常点");
        auto processedCloud = preprocessPointCloud(pclCloud, result);
        if (checkCancellation()) {
            result.errorMessage = "分析被用户取消";
            return result;
        }
        
        // 1.5 中央区域过滤（新增）
        if (params_.enableCentralRegionFilter) {
            reportProgress("预处理", 20, "提取中央区域点云");
            auto centralIndices = extractCentralRegion(processedCloud, params_.centralRegionRatio);
            
            if (centralIndices->indices.empty()) {
                reportProgress("完成", 100, "中央区域无有效点云数据");
                result.analysisSuccessful = true;
                result.overallConfidence = 1.0;
                return result;
            }
            
            // 从中央区域提取点云
            pcl::ExtractIndices<PointT> extract;
            auto centralCloud = boost::make_shared<PCLPointCloud>();
            extract.setInputCloud(processedCloud);
            extract.setIndices(centralIndices);
            extract.setNegative(false);
            extract.filter(*centralCloud);
            
            processedCloud = centralCloud;
            reportProgress("预处理", 25, 
                "中央区域提取完成，包含 " + std::to_string(processedCloud->size()) + " 个点");
        }
        
        // 2. 检测凹坑候选点
        pcl::PointIndices::Ptr candidateIndices;
        if (params_.useRANSACFitting) {
            reportProgress("检测", 30, "使用RANSAC方法检测凹坑候选点");
            candidateIndices = detectPotholeCandidatesRANSAC(processedCloud, result);
        } else {
            reportProgress("检测", 30, "使用Z阈值方法检测凹坑候选点");
            candidateIndices = detectPotholeCandidatesZThreshold(processedCloud, result);
        }
        
        if (checkCancellation()) {
            result.errorMessage = "分析被用户取消";
            return result;
        }
        
        if (!candidateIndices || candidateIndices->indices.empty()) {
            reportProgress("完成", 100, "未检测到凹坑区域");
            result.analysisSuccessful = true;
            result.overallConfidence = 1.0;
            return result;
        }
        
        // 3. 聚类分析
        reportProgress("聚类", 60, "对候选点进行聚类分析");
        auto clusters = clusterPotholeCandidates(processedCloud, candidateIndices, result);
        if (checkCancellation()) {
            result.errorMessage = "分析被用户取消";
            return result;
        }
        
        // 3.5 单个最大凹坑选择（新增）
        if (params_.detectSingleMaxPothole && !clusters.empty()) {
            reportProgress("筛选", 70, "选择最大的单个凹坑");
            
            // 计算表面高度（75%分位数）
            std::vector<float> allZ;
            allZ.reserve(processedCloud->size());
            for (const auto& point : processedCloud->points) {
                allZ.push_back(point.z);
            }
            std::sort(allZ.begin(), allZ.end());
            double surfaceZ = allZ[static_cast<size_t>(allZ.size() * 0.75)];
            
            clusters = selectLargestPothole(processedCloud, clusters, surfaceZ);
            
            if (clusters.empty()) {
                reportProgress("完成", 100, "未找到符合条件的最大凹坑");
                result.analysisSuccessful = true;
                result.overallConfidence = 1.0;
                return result;
            } else {
                reportProgress("筛选", 75, "已选择最大凹坑，包含 " + 
                    std::to_string(clusters[0].indices.size()) + " 个点");
            }
        }
        
        // 4. 生成凹坑信息
        reportProgress("计算", 80, "计算凹坑几何参数");
        generatePotholeInfo(processedCloud, clusters, result);
        if (checkCancellation()) {
            result.errorMessage = "分析被用户取消";
            return result;
        }
        
        // 5. 过滤和验证结果
        reportProgress("验证", 90, "验证和过滤结果");
        filterInvalidPotholes(result);
        
        // 6. 计算汇总统计
        result.computeSummaryStatistics();
        
        // 计算表面粗糙度
        result.surfaceRoughness = calculateSurfaceRoughness(processedCloud);
        
        // 计算整体置信度
        if (!result.potholes.empty()) {
            double totalConfidence = 0.0;
            for (const auto& pothole : result.potholes) {
                totalConfidence += pothole.confidence;
            }
            result.overallConfidence = totalConfidence / result.potholes.size();
        } else {
            result.overallConfidence = 1.0; // 没有检测到凹坑，但分析成功
        }
        
        result.analysisSuccessful = true;
        reportProgress("完成", 100, 
            "分析完成，检测到 " + std::to_string(result.validPotholeCount) + " 个有效凹坑");
        
    } catch (const std::exception& e) {
        result.analysisSuccessful = false;
        result.errorMessage = "分析过程中发生异常: " + std::string(e.what());
        reportProgress("错误", 0, result.errorMessage);
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    result.processingTimeMs = duration.count();
    
    return result;
}

std::future<AnalysisResult> PotholeDetector::analyzeAsync(std::shared_ptr<core::PointCloud> pointCloud) {
    return std::async(std::launch::async, [this, pointCloud]() {
        return this->analyze(pointCloud);
    });
}

PCLPointCloud::Ptr PotholeDetector::preprocessPointCloud(PCLPointCloud::Ptr cloud, AnalysisResult& result) {
    (void)result; // 避免未使用参数警告
    auto processedCloud = boost::make_shared<PCLPointCloud>(*cloud);
    
    // 统计异常值移除
    if (params_.noiseFilterRadius > 0) {
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(processedCloud);
        sor.setMeanK(50);  // 邻域点数
        sor.setStddevMulThresh(1.0);  // 标准差倍数
        sor.filter(*processedCloud);
        
        reportProgress("预处理", 15, "完成噪声过滤");
    }
    
    return processedCloud;
}

pcl::PointIndices::Ptr PotholeDetector::extractCentralRegion(PCLPointCloud::Ptr cloud, double regionRatio) {
    auto centralIndices = boost::make_shared<pcl::PointIndices>();
    
    if (cloud->empty() || regionRatio <= 0.0 || regionRatio > 1.0) {
        return centralIndices;
    }
    
    // 计算边界框
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    // 计算中央区域的边界
    double totalWidth = maxPt.x - minPt.x;
    double totalLength = maxPt.y - minPt.y;
    double marginX = totalWidth * (1.0 - regionRatio) / 2.0;
    double marginY = totalLength * (1.0 - regionRatio) / 2.0;
    
    double centralMinX = minPt.x + marginX;
    double centralMaxX = maxPt.x - marginX;
    double centralMinY = minPt.y + marginY;
    double centralMaxY = maxPt.y - marginY;
    
    // 预分配索引容器
    centralIndices->indices.reserve(static_cast<size_t>(cloud->size() * regionRatio * regionRatio));
    
    // 筛选中央区域的点
    for (size_t i = 0; i < cloud->size(); ++i) {
        const auto& point = cloud->points[i];
        if (point.x >= centralMinX && point.x <= centralMaxX &&
            point.y >= centralMinY && point.y <= centralMaxY) {
            centralIndices->indices.push_back(static_cast<int>(i));
        }
    }
    
    return centralIndices;
}

pcl::PointIndices::Ptr PotholeDetector::detectPotholeCandidatesZThreshold(
    PCLPointCloud::Ptr cloud, AnalysisResult& result) {
    (void)result; // 避免未使用参数警告
    
    auto candidateIndices = boost::make_shared<pcl::PointIndices>();
    
    // 计算Z值统计信息
    std::vector<float> zValues;
    zValues.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        zValues.push_back(point.z);
    }
    
    std::sort(zValues.begin(), zValues.end());
    
    // 计算阈值
    size_t thresholdIndex = static_cast<size_t>(zValues.size() * params_.zThresholdPercentile / 100.0);
    float zThreshold = zValues[thresholdIndex];
    
    // 找出低于阈值的点
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (cloud->points[i].z < zThreshold) {
            candidateIndices->indices.push_back(i);
        }
    }
    
    return candidateIndices;
}

pcl::PointIndices::Ptr PotholeDetector::detectPotholeCandidatesRANSAC(
    PCLPointCloud::Ptr cloud, AnalysisResult& result) {
    (void)result; // 避免未使用参数警告
    
    // 使用RANSAC拟合平面
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(params_.ransacMaxIterations);
    seg.setDistanceThreshold(params_.ransacThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    
    // 找出不在平面上的点（可能的凹坑点）
    auto candidateIndices = boost::make_shared<pcl::PointIndices>();
    std::set<int> inlierSet(inliers->indices.begin(), inliers->indices.end());
    
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (inlierSet.find(i) == inlierSet.end()) {
            candidateIndices->indices.push_back(i);
        }
    }
    
    return candidateIndices;
}

std::vector<pcl::PointIndices> PotholeDetector::clusterPotholeCandidates(
    PCLPointCloud::Ptr cloud, pcl::PointIndices::Ptr candidateIndices, AnalysisResult& result) {
    (void)result; // 避免未使用参数警告
    
    // 提取候选点云
    pcl::ExtractIndices<PointT> extract;
    auto candidateCloud = boost::make_shared<PCLPointCloud>();
    extract.setInputCloud(cloud);
    extract.setIndices(candidateIndices);
    extract.setNegative(false);
    extract.filter(*candidateCloud);
    
    if (candidateCloud->empty()) {
        return std::vector<pcl::PointIndices>();
    }
    
    // 欧几里得聚类
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(params_.clusteringRadius);
    ec.setMinClusterSize(params_.minPotholePoints);
    ec.setMaxClusterSize(candidateCloud->size());
    ec.setSearchMethod(searchTree_);
    ec.setInputCloud(candidateCloud);
    ec.extract(clusterIndices);
    
    // 将聚类索引映射回原始点云索引
    std::vector<pcl::PointIndices> originalClusterIndices;
    for (const auto& cluster : clusterIndices) {
        pcl::PointIndices originalCluster;
        for (int idx : cluster.indices) {
            originalCluster.indices.push_back(candidateIndices->indices[idx]);
        }
        originalClusterIndices.push_back(originalCluster);
    }
    
    return originalClusterIndices;
}

std::vector<pcl::PointIndices> PotholeDetector::selectLargestPothole(
    PCLPointCloud::Ptr cloud, const std::vector<pcl::PointIndices>& clusters, double surfaceZ) {
    
    if (clusters.empty()) {
        return std::vector<pcl::PointIndices>();
    }
    
    size_t maxClusterIndex = 0;
    double maxScore = 0.0;
    
    // * 基于深度×点数的综合评分选择最大凹坑
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& cluster = clusters[i];
        
        if (cluster.indices.empty()) continue;
        
        // 计算该聚类的最大深度和平均深度
        double maxDepth = 0.0;
        double totalDepth = 0.0;
        size_t validPointCount = 0;
        
        for (int idx : cluster.indices) {
            const auto& point = cloud->points[idx];
            double depth = surfaceZ - point.z;
            if (depth > 0) {
                maxDepth = std::max(maxDepth, depth);
                totalDepth += depth;
                validPointCount++;
            }
        }
        
        if (validPointCount == 0) continue;
        
        // 综合评分：最大深度 × 点数量权重 × 深度一致性权重
        double avgDepth = totalDepth / validPointCount;
        double depthConsistency = (maxDepth > 0) ? (avgDepth / maxDepth) : 0.0; // [0,1]
        double score = maxDepth * std::sqrt(validPointCount) * (0.5 + 0.5 * depthConsistency);
        
        if (score > maxScore) {
            maxScore = score;
            maxClusterIndex = i;
        }
    }
    
    // 返回最大评分的单个聚类
    if (maxScore > 0) {
        return {clusters[maxClusterIndex]};
    }
    
    return std::vector<pcl::PointIndices>();
}

void PotholeDetector::generatePotholeInfo(PCLPointCloud::Ptr cloud, 
                                        const std::vector<pcl::PointIndices>& clusters, 
                                        AnalysisResult& result) {
    
    result.potholes.clear();
    result.potholes.reserve(clusters.size());
    
    for (const auto& cluster : clusters) {
        if (checkCancellation()) break;
        
        auto potholeInfo = calculatePotholeGeometry(cloud, cluster);
        
        // 计算置信度
        potholeInfo.confidence = calculateConfidence(potholeInfo, cloud->size());
        
        result.potholes.push_back(std::move(potholeInfo));
    }
}

PotholeInfo PotholeDetector::calculatePotholeGeometry(PCLPointCloud::Ptr cloud, 
                                                     const pcl::PointIndices& indices) {
    PotholeInfo pothole;
    pothole.pointCount = indices.indices.size();
    // 将int转换为size_t
    pothole.pointIndices.clear();
    pothole.pointIndices.reserve(indices.indices.size());
    for (int idx : indices.indices) {
        pothole.pointIndices.push_back(static_cast<size_t>(idx));
    }
    
    // 提取凹坑点云
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    auto indices_ptr = boost::make_shared<pcl::PointIndices>(indices);
    extract.setIndices(indices_ptr);
    extract.setNegative(false);
    extract.filter(*pothole.potholePoints);
    
    if (pothole.potholePoints->empty()) {
        return pothole;
    }
    
    // 计算边界框
    pcl::getMinMax3D(*pothole.potholePoints, pothole.minPoint, pothole.maxPoint);
    
    // 计算中心点
    pothole.center.x = (pothole.minPoint.x + pothole.maxPoint.x) / 2.0f;
    pothole.center.y = (pothole.minPoint.y + pothole.maxPoint.y) / 2.0f;
    pothole.center.z = (pothole.minPoint.z + pothole.maxPoint.z) / 2.0f;
    
    // 计算尺寸
    pothole.width = pothole.maxPoint.x - pothole.minPoint.x;
    pothole.length = pothole.maxPoint.y - pothole.minPoint.y;
    
    // * 计算最大深度信息（去掉平均深度）
    double minZ = std::numeric_limits<double>::max();
    size_t deepestPointIndex = 0;
    
    // 找到最深点和最深点索引
    for (size_t i = 0; i < pothole.potholePoints->size(); ++i) {
        const auto& point = pothole.potholePoints->points[i];
        if (static_cast<double>(point.z) < minZ) {
            minZ = static_cast<double>(point.z);
            deepestPointIndex = i;
        }
    }
    
    // ! 修复表面高度估算 - 使用95%分位数作为路面参考
    // 对于路面扫描数据，大部分点应该接近路面，凹坑是少数低点
    std::vector<float> allZ;
    allZ.reserve(cloud->size());
    for (const auto& point : cloud->points) {
        allZ.push_back(point.z);
    }
    std::sort(allZ.begin(), allZ.end());
    
    // 使用95%分位数作为路面表面参考高度，这样能更准确地反映实际路面水平
    double surfaceZ = allZ[static_cast<size_t>(allZ.size() * 0.95)];
    
    // ! 只计算最大深度，不计算平均深度
    pothole.maxDepth = surfaceZ - minZ;
    pothole.depth = 0.0;  // 平均深度设为0，表示不使用
    
    // 设置最深点位置
    if (deepestPointIndex < pothole.potholePoints->size()) {
        pothole.deepestPoint = pothole.potholePoints->points[deepestPointIndex];
    }
    
    // 计算投影面积
    pothole.area = calculateProjectionArea(pothole.potholePoints);
    
    // 计算体积
    pothole.volume = calculateVolume(pothole.potholePoints, surfaceZ);
    
    return pothole;
}

double PotholeDetector::calculateProjectionArea(PCLPointCloud::Ptr potholePoints) {
    if (potholePoints->size() < 3) {
        return 0.0;
    }
    
    // 使用简化的边界框面积估算
    PointT minPt, maxPt;
    pcl::getMinMax3D(*potholePoints, minPt, maxPt);
    
    double width = maxPt.x - minPt.x;
    double length = maxPt.y - minPt.y;
    
    // 使用椭圆面积公式作为更好的近似 (π * a * b)
    return M_PI * (width / 2.0) * (length / 2.0);
}

double PotholeDetector::calculateVolume(PCLPointCloud::Ptr potholePoints, double surfaceZ) {
    if (potholePoints->empty()) {
        return 0.0;
    }
    
    // 简化的体积计算：面积 × 平均深度
    double totalDepth = 0.0;
    for (const auto& point : potholePoints->points) {
        double depth = surfaceZ - point.z;
        if (depth > 0) {
            totalDepth += depth;
        }
    }
    
    double avgDepth = totalDepth / potholePoints->size();
    double area = calculateProjectionArea(potholePoints);
    
    return area * avgDepth;
}

double PotholeDetector::calculateConfidence(const PotholeInfo& pothole, size_t totalPoints) {
    double confidence = 1.0;
    
    // 基于点数的置信度
    double pointRatio = static_cast<double>(pothole.pointCount) / totalPoints;
    double pointConfidence = std::min(1.0, pointRatio * 100.0); // 点数占比的100倍，最大为1
    
    // 基于面积的置信度
    double areaConfidence = (pothole.area >= params_.minPotholeArea) ? 1.0 : 0.5;
    
    // 基于深度的置信度（使用最大深度而不是平均深度）
    double depthConfidence = std::min(1.0, pothole.maxDepth * 50.0); // 最大深度*50，最大为1
    
    // 基于几何形状的置信度
    double aspectRatio = pothole.getAspectRatio();
    double shapeConfidence = 1.0;
    if (aspectRatio > 3.0 || aspectRatio < 0.3) {
        shapeConfidence = 0.7; // 过于细长或过于扁平的形状置信度降低
    }
    
    // 综合置信度
    confidence = (pointConfidence * 0.3 + areaConfidence * 0.3 + 
                  depthConfidence * 0.2 + shapeConfidence * 0.2);
    
    return std::max(0.0, std::min(1.0, confidence));
}

void PotholeDetector::filterInvalidPotholes(AnalysisResult& result) {
    auto it = std::remove_if(result.potholes.begin(), result.potholes.end(),
        [this](const PotholeInfo& pothole) {
            return pothole.area < params_.minPotholeArea ||
                   pothole.pointCount < params_.minPotholePoints ||
                   (params_.enableQualityFiltering && 
                    pothole.confidence < params_.minConfidenceThreshold);
        });
    
    result.potholes.erase(it, result.potholes.end());
}

double PotholeDetector::calculateSurfaceRoughness(PCLPointCloud::Ptr cloud) {
    if (cloud->size() < 3) return 0.0;
    
    double totalVariation = 0.0;
    size_t count = 0;
    
    // 使用简单的Z值标准差作为粗糙度指标
    double meanZ = 0.0;
    for (const auto& point : cloud->points) {
        meanZ += point.z;
    }
    meanZ /= cloud->size();
    
    for (const auto& point : cloud->points) {
        double diff = point.z - meanZ;
        totalVariation += diff * diff;
        count++;
    }
    
    return std::sqrt(totalVariation / count);
}

void PotholeDetector::reportProgress(const std::string& stage, int progress, const std::string& message) {
    if (progressCallback_) {
        progressCallback_(stage, progress, message);
    }
}

bool PotholeDetector::checkCancellation() {
    return cancelled_.load();
}

// 辅助函数实现
AnalysisParams createDefaultAnalysisParams() {
    AnalysisParams params;
    params.zThresholdPercentile = 20.0;
    params.minPotholeArea = 0.001;  // 1平方厘米
    params.minPotholePoints = 10;
    params.clusteringRadius = 0.05; // 5厘米
    params.useRANSACFitting = true;
    params.ransacThreshold = 0.01;  // 1厘米
    params.ransacMaxIterations = 1000;
    params.useConvexHull = true;
    params.enableMultipleDetection = true;
    params.noiseFilterRadius = 0.02; // 2厘米
    params.minConfidenceThreshold = 0.3;
    params.enableQualityFiltering = true;
    
    // 新增的中央区域过滤参数
    params.enableCentralRegionFilter = false;  // 默认关闭
    params.centralRegionRatio = 0.6;           // 中央60%区域
    params.detectSingleMaxPothole = false;    // 默认检测多个凹坑
    
    return params;
}

AnalysisParams createSensitiveAnalysisParams() {
    auto params = createDefaultAnalysisParams();
    params.zThresholdPercentile = 30.0;  // 更高的阈值，检测更多候选点
    params.minPotholeArea = 0.0005;      // 更小的最小面积
    params.minPotholePoints = 5;         // 更少的最小点数
    params.clusteringRadius = 0.03;      // 更小的聚类半径
    params.minConfidenceThreshold = 0.2; // 更低的置信度要求
    return params;
}

AnalysisParams createFastAnalysisParams() {
    auto params = createDefaultAnalysisParams();
    params.useRANSACFitting = false;     // 使用更快的Z阈值方法
    params.ransacMaxIterations = 100;    // 减少RANSAC迭代次数
    params.enableMultipleDetection = false; // 只检测最显著的凹坑
    params.noiseFilterRadius = 0.0;      // 跳过噪声过滤
    params.enableQualityFiltering = false; // 跳过质量过滤
    return params;
}

AnalysisParams createCentralMaxPotholeParams() {
    auto params = createDefaultAnalysisParams();
    
    // ! 启用中央区域过滤和单坑检测 - 针对自动深度分析优化
    params.enableCentralRegionFilter = true;   // 启用中央区域过滤
    params.centralRegionRatio = 0.8;           // 扩大到中央80%区域，获取更多数据
    params.detectSingleMaxPothole = true;     // 只检测最大单个凹坑
    
    // * 优化参数设置以提高单坑检测精度和深度分析能力
    params.enableMultipleDetection = false;   // 关闭多凹坑检测模式
    params.minPotholePoints = 3;              // 进一步降低最小点数要求，提高灵敏度
    params.minPotholeArea = 0.0005;           // 更小的最小面积阈值（0.5平方厘米）
    params.minConfidenceThreshold = 0.15;     // 更低的置信度要求，提高检测率
    
    // * 使用Z阈值方法进行更快速的深度分析
    params.useRANSACFitting = false;          // 改为Z阈值方法，更适合深度分析
    params.zThresholdPercentile = 15.0;       // 更敏感的Z阈值（15%分位数）
    
    // * 聚类参数优化
    params.clusteringRadius = 0.04;           // 稍大的聚类半径，聚合更多相关点
    
    // * 启用质量过滤确保结果可靠性
    params.enableQualityFiltering = true;
    params.noiseFilterRadius = 0.015;         // 适度的噪声过滤
    
    return params;
}

} // namespace analysis
} // namespace pcl_viz