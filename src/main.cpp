#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>

typedef pcl::PointNormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ASCLoader {
public:
    static bool loadASCFile(const std::string& filename, PointCloud::Ptr cloud) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: 无法打开文件 " << filename << std::endl;
            return false;
        }
        
        std::string line;
        int lineCount = 0;
        
        while (std::getline(file, line)) {
            lineCount++;
            
            // 跳过注释行
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
                
                cloud->points.push_back(point);
            } else {
                std::cerr << "Warning: 第 " << lineCount << " 行数据格式错误: " << line << std::endl;
            }
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;
        
        std::cout << "成功加载 " << cloud->points.size() << " 个点" << std::endl;
        return true;
    }
};

void printCloudInfo(const PointCloud::Ptr cloud) {
    if (cloud->points.empty()) {
        std::cout << "点云为空" << std::endl;
        return;
    }
    
    // 计算边界框
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    
    std::cout << "=== 点云信息 ===" << std::endl;
    std::cout << "点数量: " << cloud->points.size() << std::endl;
    std::cout << "X 范围: [" << minPt.x << ", " << maxPt.x << "] (跨度: " << maxPt.x - minPt.x << ")" << std::endl;
    std::cout << "Y 范围: [" << minPt.y << ", " << maxPt.y << "] (跨度: " << maxPt.y - minPt.y << ")" << std::endl;
    std::cout << "Z 范围: [" << minPt.z << ", " << maxPt.z << "] (跨度: " << maxPt.z - minPt.z << ")" << std::endl;
    std::cout << "===============" << std::endl;
}

int main(int argc, char** argv) {
    std::string filename = "../../data/H103v2.asc";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "正在加载点云文件: " << filename << std::endl;
    
    // 创建点云对象
    PointCloud::Ptr cloud(new PointCloud);
    
    // 加载 ASC 文件
    if (!ASCLoader::loadASCFile(filename, cloud)) {
        std::cerr << "Failed to load point cloud!" << std::endl;
        return -1;
    }
    
    // 打印点云信息
    printCloudInfo(cloud);
    
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("PCL 点云可视化器"));
    
    // 设置背景颜色为黑色
    viewer->setBackgroundColor(0, 0, 0);
    
    // 添加点云到可视化器
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> rgb(cloud, "z");
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    
    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    
    // 初始化摄像机
    viewer->initCameraParameters();
    
    // 设置摄像机位置以更好地查看数据
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    float centerX = (minPt.x + maxPt.x) / 2.0;
    float centerY = (minPt.y + maxPt.y) / 2.0;
    float centerZ = (minPt.z + maxPt.z) / 2.0;
    
    viewer->setCameraPosition(
        centerX - 5, centerY - 5, centerZ + 5,  // 摄像机位置
        centerX, centerY, centerZ,              // 观察点
        0, 0, 1                                 // 向上向量
    );
    
    std::cout << "点云可视化器已启动。按 'q' 退出，'h' 显示帮助信息。" << std::endl;
    
    // 主循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
    
    return 0;
}