#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非GUI后端
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull
import pandas as pd

def load_asc_file(filename):
    """加载 ASC 点云文件"""
    points = []
    normals = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) == 6:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                normals.append([float(parts[3]), float(parts[4]), float(parts[5])])
    
    return np.array(points), np.array(normals)

def analyze_pointcloud(points):
    """分析点云数据特征"""
    analysis = {
        '点数量': len(points),
        'X范围': (points[:, 0].min(), points[:, 0].max()),
        'Y范围': (points[:, 1].min(), points[:, 1].max()),
        'Z范围': (points[:, 2].min(), points[:, 2].max()),
        'X跨度': points[:, 0].max() - points[:, 0].min(),
        'Y跨度': points[:, 1].max() - points[:, 1].min(),
        'Z跨度': points[:, 2].max() - points[:, 2].min(),
        '中心点': points.mean(axis=0),
        '标准差': points.std(axis=0)
    }
    return analysis

def detect_pothole(points, z_threshold_percentile=20):
    """检测凹坑区域"""
    z_values = points[:, 2]
    z_threshold = np.percentile(z_values, z_threshold_percentile)
    
    # 找出低于阈值的点（可能是凹坑）
    pothole_mask = z_values < z_threshold
    pothole_points = points[pothole_mask]
    surface_points = points[~pothole_mask]
    
    if len(pothole_points) > 0:
        # 估算凹坑深度
        avg_surface_z = surface_points[:, 2].mean()
        min_pothole_z = pothole_points[:, 2].min()
        max_depth = avg_surface_z - min_pothole_z
        avg_depth = avg_surface_z - pothole_points[:, 2].mean()
        
        # 估算凹坑面积（使用凸包投影）
        if len(pothole_points) > 3:
            try:
                hull_2d = ConvexHull(pothole_points[:, :2])
                area = hull_2d.volume  # 在2D中，volume实际上是面积
            except:
                area = 0
        else:
            area = 0
        
        # 估算体积（简化方法）
        volume = area * avg_depth if area > 0 else 0
        
        return {
            '凹坑点数': len(pothole_points),
            '最大深度': max_depth,
            '平均深度': avg_depth,
            '投影面积': area,
            '估算体积': volume,
            '凹坑中心': pothole_points.mean(axis=0),
            'X尺寸': pothole_points[:, 0].max() - pothole_points[:, 0].min(),
            'Y尺寸': pothole_points[:, 1].max() - pothole_points[:, 1].min(),
        }, pothole_mask
    
    return None, None

def visualize_pointcloud(points, normals, pothole_mask=None):
    """创建点云可视化"""
    fig = plt.figure(figsize=(20, 15))
    
    # 1. 3D点云视图
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    if pothole_mask is not None:
        # 用不同颜色显示凹坑和表面
        ax1.scatter(points[~pothole_mask, 0], points[~pothole_mask, 1], 
                   points[~pothole_mask, 2], c='blue', s=1, alpha=0.5, label='表面')
        ax1.scatter(points[pothole_mask, 0], points[pothole_mask, 1], 
                   points[pothole_mask, 2], c='red', s=2, alpha=0.8, label='凹坑')
    else:
        scatter = ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                            c=points[:, 2], cmap='viridis', s=1)
        plt.colorbar(scatter, ax=ax1, label='Z值')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D点云视图')
    if pothole_mask is not None:
        ax1.legend()
    
    # 2. XY平面投影（俯视图）
    ax2 = fig.add_subplot(2, 3, 2)
    if pothole_mask is not None:
        ax2.scatter(points[~pothole_mask, 0], points[~pothole_mask, 1], 
                   c='blue', s=1, alpha=0.5, label='表面')
        ax2.scatter(points[pothole_mask, 0], points[pothole_mask, 1], 
                   c='red', s=2, alpha=0.8, label='凹坑')
        ax2.legend()
    else:
        scatter2 = ax2.scatter(points[:, 0], points[:, 1], c=points[:, 2], 
                              cmap='viridis', s=1)
        plt.colorbar(scatter2, ax=ax2, label='Z值')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('XY平面投影（俯视图）')
    ax2.set_aspect('equal')
    
    # 3. XZ平面投影（侧视图）
    ax3 = fig.add_subplot(2, 3, 3)
    if pothole_mask is not None:
        ax3.scatter(points[~pothole_mask, 0], points[~pothole_mask, 2], 
                   c='blue', s=1, alpha=0.5, label='表面')
        ax3.scatter(points[pothole_mask, 0], points[pothole_mask, 2], 
                   c='red', s=2, alpha=0.8, label='凹坑')
        ax3.legend()
    else:
        ax3.scatter(points[:, 0], points[:, 2], c=points[:, 2], 
                   cmap='viridis', s=1)
    ax3.set_xlabel('X')
    ax3.set_ylabel('Z')
    ax3.set_title('XZ平面投影（侧视图）')
    
    # 4. YZ平面投影（侧视图）
    ax4 = fig.add_subplot(2, 3, 4)
    if pothole_mask is not None:
        ax4.scatter(points[~pothole_mask, 1], points[~pothole_mask, 2], 
                   c='blue', s=1, alpha=0.5, label='表面')
        ax4.scatter(points[pothole_mask, 1], points[pothole_mask, 2], 
                   c='red', s=2, alpha=0.8, label='凹坑')
        ax4.legend()
    else:
        ax4.scatter(points[:, 1], points[:, 2], c=points[:, 2], 
                   cmap='viridis', s=1)
    ax4.set_xlabel('Y')
    ax4.set_ylabel('Z')
    ax4.set_title('YZ平面投影（侧视图）')
    
    # 5. Z值分布直方图
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.hist(points[:, 2], bins=50, edgecolor='black', alpha=0.7)
    ax5.set_xlabel('Z值')
    ax5.set_ylabel('频次')
    ax5.set_title('Z值分布直方图')
    ax5.axvline(points[:, 2].mean(), color='red', linestyle='--', 
                label=f'平均值: {points[:, 2].mean():.4f}')
    ax5.legend()
    
    # 6. 热力图（Z值分布）
    ax6 = fig.add_subplot(2, 3, 6)
    # 创建网格
    xi = np.linspace(points[:, 0].min(), points[:, 0].max(), 50)
    yi = np.linspace(points[:, 1].min(), points[:, 1].max(), 50)
    Xi, Yi = np.meshgrid(xi, yi)
    
    # 插值Z值
    from scipy.interpolate import griddata
    Zi = griddata((points[:, 0], points[:, 1]), points[:, 2], 
                  (Xi, Yi), method='linear')
    
    contour = ax6.contourf(Xi, Yi, Zi, levels=20, cmap='viridis')
    plt.colorbar(contour, ax=ax6, label='Z值')
    ax6.set_xlabel('X')
    ax6.set_ylabel('Y')
    ax6.set_title('Z值热力图')
    ax6.set_aspect('equal')
    
    plt.tight_layout()
    plt.savefig('pointcloud_analysis.png', dpi=150, bbox_inches='tight')
    # plt.show()  # 在没有X服务器的环境中注释掉显示

def main():
    filename = 'data/H103v2.asc'
    print(f"正在分析点云文件: {filename}")
    
    # 加载数据
    points, normals = load_asc_file(filename)
    print(f"成功加载 {len(points)} 个点")
    
    # 分析数据
    analysis = analyze_pointcloud(points)
    print("\n=== 点云数据分析 ===")
    for key, value in analysis.items():
        if isinstance(value, tuple):
            print(f"{key}: {value[0]:.4f} 到 {value[1]:.4f}")
        elif isinstance(value, np.ndarray):
            print(f"{key}: [{value[0]:.4f}, {value[1]:.4f}, {value[2]:.4f}]")
        elif isinstance(value, float):
            print(f"{key}: {value:.4f}")
        else:
            print(f"{key}: {value}")
    
    # 检测凹坑
    pothole_info, pothole_mask = detect_pothole(points)
    if pothole_info:
        print("\n=== 凹坑检测结果 ===")
        for key, value in pothole_info.items():
            if isinstance(value, np.ndarray):
                print(f"{key}: [{value[0]:.4f}, {value[1]:.4f}, {value[2]:.4f}]")
            elif isinstance(value, float):
                print(f"{key}: {value:.6f}")
            else:
                print(f"{key}: {value}")
    
    # 可视化
    print("\n正在生成可视化图表...")
    visualize_pointcloud(points, normals, pothole_mask)
    
    # 生成详细报告
    print("\n=== 数据形态分析 ===")
    print(f"数据呈现特征:")
    print(f"1. X-Y平面: 约 {analysis['X跨度']:.2f} × {analysis['Y跨度']:.2f} 的矩形区域")
    print(f"2. Z方向变化: {analysis['Z跨度']:.4f}（相对较小，表明是近似平面）")
    print(f"3. 数据密度: {len(points) / (analysis['X跨度'] * analysis['Y跨度']):.1f} 点/单位面积")
    
    if pothole_info:
        print(f"\n4. 凹坑特征:")
        print(f"   - 位置: 中心位于 ({pothole_info['凹坑中心'][0]:.4f}, {pothole_info['凹坑中心'][1]:.4f})")
        print(f"   - 尺寸: {pothole_info['X尺寸']:.4f} × {pothole_info['Y尺寸']:.4f}")
        print(f"   - 深度: 最大 {pothole_info['最大深度']:.6f}, 平均 {pothole_info['平均深度']:.6f}")
        print(f"   - 面积: {pothole_info['投影面积']:.6f}")
        print(f"   - 体积: {pothole_info['估算体积']:.8f}")

if __name__ == "__main__":
    main()