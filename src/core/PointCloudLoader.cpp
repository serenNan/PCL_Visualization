#include "PointCloudLoader.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <optional>
#include <array>

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
    char detectedSeparator = '\0';
    bool separatorDetected = false;

    std::cout << "正在加载 ASC 文件: " << filename << std::endl;

    // * 用于检测分隔符的 lambda 函数
    auto detectSeparator = [](const std::string& line) -> char {
        // 检查逗号分隔符
        if (line.find(',') != std::string::npos) {
            return ',';
        }
        // 检查是否包含数字和空格（空格分隔符）
        bool hasDigit = std::any_of(line.begin(), line.end(), ::isdigit);
        bool hasSpace = line.find(' ') != std::string::npos;
        if (hasDigit && hasSpace) {
            return ' ';
        }
        return '\0'; // 无法检测
    };

    // * 解析一行数据的 lambda 函数，支持不同分隔符
    auto parseLine = [](const std::string& line, char separator) -> std::optional<std::array<float, 6>> {
        std::vector<float> values;
        
        if (separator == ',') {
            // 逗号分隔符处理
            std::stringstream ss(line);
            std::string token;
            
            while (std::getline(ss, token, ',')) {
                // 去除前后空格
                token.erase(0, token.find_first_not_of(" \t"));
                token.erase(token.find_last_not_of(" \t") + 1);
                
                if (!token.empty()) {
                    try {
                        values.push_back(std::stof(token));
                    } catch (const std::exception&) {
                        return std::nullopt;
                    }
                }
            }
        } else {
            // 空格分隔符处理（默认）
            std::istringstream iss(line);
            float value;
            while (iss >> value) {
                values.push_back(value);
            }
        }
        
        // 确保有 6 个值（x, y, z, nx, ny, nz）
        if (values.size() != 6) {
            return std::nullopt;
        }
        
        std::array<float, 6> result;
        std::copy(values.begin(), values.end(), result.begin());
        return result;
    };

    // * 验证点是否为有效非零点的 lambda
    auto isValidPoint = [](const std::array<float, 6>& values) -> bool {
        // 检查数值有效性
        if (!std::all_of(values.begin(), values.end(), [](float v) { return std::isfinite(v); })) {
            return false;
        }
        
        // 检查是否为全零点（通常是无效数据）
        constexpr float epsilon = 1e-6f;
        bool allZero = std::all_of(values.begin(), values.begin() + 3, 
                                  [](float v) { return std::abs(v) < epsilon; });
        
        return !allZero;
    };

    while (std::getline(file, line)) {
        lineCount++;
        
        // 跳过空行和注释行
        if (line.empty() || line[0] == '#') {
            continue;
        }

        // ! 自动检测分隔符（只在第一个有效数据行进行）
        if (!separatorDetected) {
            detectedSeparator = detectSeparator(line);
            if (detectedSeparator != '\0') {
                separatorDetected = true;
                std::cout << "检测到分隔符: '" 
                         << (detectedSeparator == ',' ? "逗号" : "空格") 
                         << "'" << std::endl;
            } else {
                std::cerr << "警告: 第 " << lineCount << " 行无法检测分隔符，跳过: " << line << std::endl;
                invalidLines++;
                continue;
            }
        }

        // 解析当前行
        auto parseResult = parseLine(line, detectedSeparator);
        
        if (parseResult && isValidPoint(*parseResult)) {
            const auto& [x, y, z, nx, ny, nz] = *parseResult;
            
            PointT point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.normal_x = nx;
            point.normal_y = ny;
            point.normal_z = nz;

            cloud->points.push_back(point);
            validPoints++;
        } else {
            invalidLines++;
            if (invalidLines <= 5) { // 只显示前5个错误
                std::string reason = parseResult ? "无效数值或全零点" : "数据格式错误";
                std::cerr << "警告: 第 " << lineCount << " 行" << reason << ": " << line << std::endl;
            }
        }
    }

    file.close();

    // 设置点云属性
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = (invalidLines == 0);

    std::cout << "ASC 文件加载完成:" << std::endl;
    std::cout << "  - 分隔符类型: " << (detectedSeparator == ',' ? "逗号" : "空格") << std::endl;
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