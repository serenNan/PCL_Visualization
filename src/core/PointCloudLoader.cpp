#include "PointCloudLoader.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace pcl_viz {
namespace core {

bool PointCloudLoader::loadPointCloud(const std::string& filename, 
                                     PCLPointCloud::Ptr cloud, 
                                     FileFormat format) {
    if (!cloud) {
        std::cerr << "错误: 点云对象为空指针" << std::endl;
        return false;
    }

    // 清空现有点云数据
    cloud->clear();

    // 自动检测格式
    if (format == FileFormat::AUTO) {
        format = detectFormat(filename);
    }

    bool success = false;
    switch (format) {
        case FileFormat::ASC:
            success = loadASCFile(filename, cloud);
            break;
        case FileFormat::PCD:
        case FileFormat::PLY:
            std::cerr << "错误: 暂不支持该格式，计划在后续版本中实现" << std::endl;
            return false;
        default:
            std::cerr << "错误: 未知文件格式" << std::endl;
            return false;
    }

    if (success) {
        success = validatePointCloud(cloud);
    }

    return success;
}

PointCloudLoader::FileFormat PointCloudLoader::detectFormat(const std::string& filename) {
    // 提取文件扩展名
    size_t dotPos = filename.find_last_of('.');
    if (dotPos == std::string::npos) {
        std::cerr << "警告: 无法识别文件格式，使用默认 ASC 格式" << std::endl;
        return FileFormat::ASC;
    }

    std::string extension = filename.substr(dotPos + 1);
    
    // 转换为小写
    std::transform(extension.begin(), extension.end(), extension.begin(), 
                   [](unsigned char c) { return std::tolower(c); });

    if (extension == "asc") {
        return FileFormat::ASC;
    } else if (extension == "pcd") {
        return FileFormat::PCD;
    } else if (extension == "ply") {
        return FileFormat::PLY;
    }

    std::cerr << "警告: 未知文件扩展名 '" << extension << "'，使用默认 ASC 格式" << std::endl;
    return FileFormat::ASC;
}

bool PointCloudLoader::loadASCFile(const std::string& filename, PCLPointCloud::Ptr cloud) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "错误: 无法打开文件 " << filename << std::endl;
        return false;
    }

    std::string line;
    int lineCount = 0;
    int validPoints = 0;
    int invalidLines = 0;

    std::cout << "正在加载 ASC 文件: " << filename << std::endl;

    while (std::getline(file, line)) {
        lineCount++;
        
        // 跳过空行和注释行
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        float x, y, z, nx, ny, nz;

        // 解析点坐标和法向量: X Y Z Normal_X Normal_Y Normal_Z
        if (iss >> x >> y >> z >> nx >> ny >> nz) {
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.normal_x = nx;
            point.normal_y = ny;
            point.normal_z = nz;

            // 验证点的有效性
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                std::isfinite(nx) && std::isfinite(ny) && std::isfinite(nz)) {
                cloud->points.push_back(point);
                validPoints++;
            } else {
                invalidLines++;
                if (invalidLines <= 5) { // 只显示前5个错误
                    std::cerr << "警告: 第 " << lineCount << " 行包含无效数值: " << line << std::endl;
                }
            }
        } else {
            invalidLines++;
            if (invalidLines <= 5) {
                std::cerr << "警告: 第 " << lineCount << " 行数据格式错误: " << line << std::endl;
            }
        }
    }

    file.close();

    // 设置点云属性
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = (invalidLines == 0);

    std::cout << "ASC 文件加载完成:" << std::endl;
    std::cout << "  - 有效点数: " << validPoints << std::endl;
    std::cout << "  - 无效行数: " << invalidLines << std::endl;
    std::cout << "  - 总处理行数: " << lineCount << std::endl;

    return validPoints > 0;
}

bool PointCloudLoader::validatePointCloud(const PCLPointCloud::Ptr cloud) {
    if (!cloud || cloud->points.empty()) {
        std::cerr << "错误: 点云为空" << std::endl;
        return false;
    }

    size_t invalidPoints = 0;
    for (const auto& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            invalidPoints++;
        }
    }

    if (invalidPoints > 0) {
        std::cerr << "警告: 发现 " << invalidPoints << " 个无效点" << std::endl;
    }

    std::cout << "点云验证完成: " << cloud->points.size() << " 个有效点" << std::endl;
    return true;
}

} // namespace core
} // namespace pcl_viz