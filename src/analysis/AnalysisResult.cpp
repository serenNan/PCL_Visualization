#include "AnalysisResult.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace pcl_viz {
namespace analysis {

std::string AnalysisResult::toJson(bool prettyFormat) const {
    std::ostringstream json;
    const std::string indent = prettyFormat ? "  " : "";
    const std::string newline = prettyFormat ? "\n" : "";
    
    json << "{" << newline;
    
    // 元数据
    auto time_t = std::chrono::system_clock::to_time_t(analysisTime);
    json << indent << "\"metadata\": {" << newline;
    json << indent << indent << "\"analysisTime\": \"" << std::ctime(&time_t);
    json.seekp(-1, std::ios_base::cur); // 移除换行符
    json << "\"," << newline;
    json << indent << indent << "\"sourceFile\": \"" << sourceFile << "\"," << newline;
    json << indent << indent << "\"processingTimeMs\": " << processingTimeMs << newline;
    json << indent << "}," << newline;
    
    // 输入数据统计
    json << indent << "\"inputStatistics\": {" << newline;
    json << indent << indent << "\"totalPointCount\": " << totalPointCount << "," << newline;
    json << indent << indent << "\"totalArea\": " << std::fixed << std::setprecision(6) << totalArea << "," << newline;
    json << indent << indent << "\"surfaceRoughness\": " << std::fixed << std::setprecision(6) << surfaceRoughness << newline;
    json << indent << "}," << newline;
    
    // 分析参数
    json << indent << "\"analysisParams\": {" << newline;
    json << indent << indent << "\"zThresholdPercentile\": " << params.zThresholdPercentile << "," << newline;
    json << indent << indent << "\"minPotholeArea\": " << std::fixed << std::setprecision(6) << params.minPotholeArea << "," << newline;
    json << indent << indent << "\"minPotholePoints\": " << params.minPotholePoints << "," << newline;
    json << indent << indent << "\"clusteringRadius\": " << std::fixed << std::setprecision(3) << params.clusteringRadius << "," << newline;
    json << indent << indent << "\"useRANSACFitting\": " << (params.useRANSACFitting ? "true" : "false") << newline;
    json << indent << "}," << newline;
    
    // 汇总结果
    json << indent << "\"summary\": {" << newline;
    json << indent << indent << "\"analysisSuccessful\": " << (analysisSuccessful ? "true" : "false") << "," << newline;
    json << indent << indent << "\"overallConfidence\": " << std::fixed << std::setprecision(3) << overallConfidence << "," << newline;
    json << indent << indent << "\"validPotholeCount\": " << validPotholeCount << "," << newline;
    json << indent << indent << "\"totalPotholeArea\": " << std::fixed << std::setprecision(6) << totalPotholeArea << "," << newline;
    json << indent << indent << "\"totalPotholeVolume\": " << std::fixed << std::setprecision(8) << totalPotholeVolume << "," << newline;
    json << indent << indent << "\"avgPotholeDepth\": " << std::fixed << std::setprecision(6) << avgPotholeDepth << "," << newline;
    json << indent << indent << "\"maxPotholeDepth\": " << std::fixed << std::setprecision(6) << maxPotholeDepth << "," << newline;
    json << indent << indent << "\"damageRatio\": " << std::fixed << std::setprecision(2) << getDamageRatio() << newline;
    json << indent << "}," << newline;
    
    // 凹坑详细信息
    json << indent << "\"potholes\": [" << newline;
    for (size_t i = 0; i < potholes.size(); ++i) {
        const auto& pothole = potholes[i];
        json << indent << indent << "{" << newline;
        json << indent << indent << indent << "\"id\": " << i << "," << newline;
        json << indent << indent << indent << "\"center\": [" << std::fixed << std::setprecision(6) 
             << pothole.center.x << ", " << pothole.center.y << ", " << pothole.center.z << "]," << newline;
        json << indent << indent << indent << "\"depth\": " << std::fixed << std::setprecision(6) << pothole.depth << "," << newline;
        json << indent << indent << indent << "\"maxDepth\": " << std::fixed << std::setprecision(6) << pothole.maxDepth << "," << newline;
        json << indent << indent << indent << "\"volume\": " << std::fixed << std::setprecision(8) << pothole.volume << "," << newline;
        json << indent << indent << indent << "\"area\": " << std::fixed << std::setprecision(6) << pothole.area << "," << newline;
        json << indent << indent << indent << "\"width\": " << std::fixed << std::setprecision(6) << pothole.width << "," << newline;
        json << indent << indent << indent << "\"length\": " << std::fixed << std::setprecision(6) << pothole.length << "," << newline;
        json << indent << indent << indent << "\"pointCount\": " << pothole.pointCount << "," << newline;
        json << indent << indent << indent << "\"confidence\": " << std::fixed << std::setprecision(3) << pothole.confidence << newline;
        json << indent << indent << "}";
        if (i < potholes.size() - 1) json << ",";
        json << newline;
    }
    json << indent << "]" << newline;
    
    if (!errorMessage.empty()) {
        json << "," << newline << indent << "\"errorMessage\": \"" << errorMessage << "\"" << newline;
    }
    
    json << "}";
    return json.str();
}

std::string AnalysisResult::toCsv() const {
    std::ostringstream csv;
    
    // CSV头部
    csv << "ID,CenterX,CenterY,CenterZ,Depth,MaxDepth,Volume,Area,Width,Length,PointCount,Confidence,AspectRatio,DepthUniformity\n";
    
    // 数据行
    for (size_t i = 0; i < potholes.size(); ++i) {
        const auto& pothole = potholes[i];
        csv << i << ","
            << std::fixed << std::setprecision(6) << pothole.center.x << ","
            << std::fixed << std::setprecision(6) << pothole.center.y << ","
            << std::fixed << std::setprecision(6) << pothole.center.z << ","
            << std::fixed << std::setprecision(6) << pothole.depth << ","
            << std::fixed << std::setprecision(6) << pothole.maxDepth << ","
            << std::fixed << std::setprecision(8) << pothole.volume << ","
            << std::fixed << std::setprecision(6) << pothole.area << ","
            << std::fixed << std::setprecision(6) << pothole.width << ","
            << std::fixed << std::setprecision(6) << pothole.length << ","
            << pothole.pointCount << ","
            << std::fixed << std::setprecision(3) << pothole.confidence << ","
            << std::fixed << std::setprecision(3) << pothole.getAspectRatio() << ","
            << std::fixed << std::setprecision(3) << pothole.getDepthUniformity() << "\n";
    }
    
    return csv.str();
}

std::string AnalysisResult::toTextReport() const {
    std::ostringstream report;
    
    // 报告头部
    report << "=== 点云凹坑分析报告 ===\n\n";
    
    // 基本信息
    auto time_t = std::chrono::system_clock::to_time_t(analysisTime);
    report << "分析时间: " << std::ctime(&time_t);
    report << "源文件: " << sourceFile << "\n";
    report << "处理时间: " << std::fixed << std::setprecision(2) << processingTimeMs << " ms\n";
    report << "分析状态: " << (analysisSuccessful ? "成功" : "失败") << "\n";
    if (!errorMessage.empty()) {
        report << "错误信息: " << errorMessage << "\n";
    }
    report << "\n";
    
    // 输入数据统计
    report << "=== 输入数据统计 ===\n";
    report << "总点数: " << totalPointCount << "\n";
    report << "总面积: " << std::fixed << std::setprecision(6) << totalArea << " m²\n";
    report << "表面粗糙度: " << std::fixed << std::setprecision(6) << surfaceRoughness << " m\n\n";
    
    // 分析参数
    report << "=== 分析参数 ===\n";
    report << "Z轴阈值百分位数: " << params.zThresholdPercentile << "%\n";
    report << "最小凹坑面积: " << std::fixed << std::setprecision(6) << params.minPotholeArea << " m²\n";
    report << "最小凹坑点数: " << params.minPotholePoints << "\n";
    report << "聚类半径: " << std::fixed << std::setprecision(3) << params.clusteringRadius << " m\n";
    report << "使用RANSAC拟合: " << (params.useRANSACFitting ? "是" : "否") << "\n\n";
    
    // 汇总结果
    report << "=== 检测结果汇总 ===\n";
    report << "整体置信度: " << std::fixed << std::setprecision(1) << (overallConfidence * 100) << "%\n";
    report << "检测到的凹坑总数: " << potholes.size() << "\n";
    report << "有效凹坑数量: " << validPotholeCount << "\n";
    report << "总凹坑面积: " << std::fixed << std::setprecision(6) << totalPotholeArea << " m²\n";
    report << "总凹坑体积: " << std::fixed << std::setprecision(8) << totalPotholeVolume << " m³\n";
    report << "平均凹坑深度: " << std::fixed << std::setprecision(6) << avgPotholeDepth << " m\n";
    report << "最大凹坑深度: " << std::fixed << std::setprecision(6) << maxPotholeDepth << " m\n";
    report << "路面损伤率: " << std::fixed << std::setprecision(2) << getDamageRatio() << "%\n\n";
    
    // 详细凹坑信息
    if (!potholes.empty()) {
        report << "=== 凹坑详细信息 ===\n";
        for (size_t i = 0; i < potholes.size(); ++i) {
            const auto& pothole = potholes[i];
            report << "凹坑 #" << (i + 1) << ":\n";
            report << "  中心位置: (" 
                   << std::fixed << std::setprecision(4) << pothole.center.x << ", "
                   << std::fixed << std::setprecision(4) << pothole.center.y << ", "
                   << std::fixed << std::setprecision(4) << pothole.center.z << ")\n";
            report << "  尺寸: " << std::fixed << std::setprecision(4) << pothole.length 
                   << " × " << std::fixed << std::setprecision(4) << pothole.width << " m\n";
            report << "  深度: 平均 " << std::fixed << std::setprecision(4) << pothole.depth 
                   << " m, 最大 " << std::fixed << std::setprecision(4) << pothole.maxDepth << " m\n";
            report << "  面积: " << std::fixed << std::setprecision(6) << pothole.area << " m²\n";
            report << "  体积: " << std::fixed << std::setprecision(8) << pothole.volume << " m³\n";
            report << "  点数: " << pothole.pointCount << "\n";
            report << "  置信度: " << std::fixed << std::setprecision(1) << (pothole.confidence * 100) << "%\n";
            report << "  长宽比: " << std::fixed << std::setprecision(2) << pothole.getAspectRatio() << "\n";
            report << "  深度均匀性: " << std::fixed << std::setprecision(2) << pothole.getDepthUniformity() << "\n";
            if (i < potholes.size() - 1) report << "\n";
        }
    }
    
    return report.str();
}

bool AnalysisResult::fromJson(const std::string& jsonStr) {
    // ! 简化实现，实际项目中建议使用JSON库如nlohmann/json
    // TODO: 实现完整的JSON解析功能
    (void)jsonStr; // 避免未使用参数警告
    return false;
}

} // namespace analysis
} // namespace pcl_viz