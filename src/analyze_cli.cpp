#include <iostream>
#include <memory>
#include <string>

#include "core/PointCloudLoader.h"
#include "core/PointCloud.h"
#include "analysis/PotholeDetector.h"

int main(int argc, char** argv) {
    using namespace pcl_viz;
    if (argc < 2) {
        std::cerr << "Usage: pcl_analyze_cli <pointcloud.asc>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    core::PCLPointCloud::Ptr pclCloud(new core::PCLPointCloud);
    if (!core::PointCloudLoader::loadPointCloud(filename, pclCloud)) {
        std::cerr << "Failed to load: " << filename << std::endl;
        return 2;
    }

    auto cloud = std::make_shared<core::PointCloud>(pclCloud, filename);

    analysis::PotholeDetector detector;
    // Start from the central-max preset which suits single pit detection
    detector.setAnalysisParams(analysis::createCentralMaxPotholeParams());

    auto result = detector.analyze(cloud);
    if (!result.analysisSuccessful) {
        std::cerr << "Analyze failed: " << result.errorMessage << std::endl;
        return 3;
    }

    std::cout.setf(std::ios::fixed); std::cout.precision(5);
    std::cout << "valid_potholes: " << result.validPotholeCount << "\n";
    std::cout << "union_area: " << result.totalPotholeArea << "\n";      // units follow input data
    std::cout << "union_volume: " << result.totalPotholeVolume << "\n";
    std::cout << "max_depth: " << result.maxPotholeDepth << "\n";
    if (!result.potholes.empty()) {
        const auto& p = result.potholes.front();
        std::cout << "first.area: " << p.area << "\n";
        std::cout << "first.volume: " << p.volume << "\n";
        std::cout << "first.width: " << p.width << "\n";
        std::cout << "first.length: " << p.length << "\n";
        std::cout << "first.max_depth: " << p.maxDepth << "\n";
        std::cout << "first.points: " << p.pointCount << "\n";
    }

    return 0;
}
