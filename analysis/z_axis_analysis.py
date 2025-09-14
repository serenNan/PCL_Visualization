#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Z轴数据分析脚本
分析H103v2.asc文件中的Z轴数据，计算统计信息和分布特征
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # 使用无GUI后端
from scipy import stats
import pandas as pd
from pathlib import Path

def load_point_cloud_data(file_path):
    """
    加载点云数据文件
    格式: X Y Z Nx Ny Nz
    """
    try:
        data = np.loadtxt(file_path)
        print(f"成功加载数据：{data.shape[0]} 个点，{data.shape[1]} 个维度")
        return data
    except Exception as e:
        print(f"加载文件失败：{e}")
        return None

def analyze_z_axis(data):
    """
    分析Z轴数据统计特征
    """
    z_values = data[:, 2]  # 第3列是Z轴数据

    # 基本统计信息
    z_min = np.min(z_values)
    z_max = np.max(z_values)
    z_range = z_max - z_min
    z_mean = np.mean(z_values)
    z_std = np.std(z_values)
    z_median = np.median(z_values)

    # 分布特征
    q25 = np.percentile(z_values, 25)
    q75 = np.percentile(z_values, 75)
    iqr = q75 - q25

    # 偏度和峰度
    skewness = stats.skew(z_values)
    kurtosis = stats.kurtosis(z_values)

    # 变异系数（相对标准差）
    cv = z_std / z_mean if z_mean != 0 else 0

    print("=" * 60)
    print("Z轴统计分析结果")
    print("=" * 60)
    print(f"1. 基本范围统计：")
    print(f"   最小值：{z_min:.6f} m")
    print(f"   最大值：{z_max:.6f} m")
    print(f"   范围：{z_range:.6f} m")
    print()
    print(f"2. 中心趋势统计：")
    print(f"   均值：{z_mean:.6f} m")
    print(f"   中位数：{z_median:.6f} m")
    print(f"   标准差：{z_std:.6f} m")
    print(f"   变异系数：{cv:.4f}")
    print()
    print(f"3. 分布特征：")
    print(f"   第一四分位数 (Q1)：{q25:.6f} m")
    print(f"   第三四分位数 (Q3)：{q75:.6f} m")
    print(f"   四分位距 (IQR)：{iqr:.6f} m")
    print(f"   偏度：{skewness:.4f} {'(右偏)' if skewness > 0 else '(左偏)' if skewness < 0 else '(对称)'}")
    print(f"   峰度：{kurtosis:.4f} {'(尖峰)' if kurtosis > 0 else '(平峰)' if kurtosis < 0 else '(正态)'}")

    return {
        'z_min': z_min, 'z_max': z_max, 'z_range': z_range,
        'z_mean': z_mean, 'z_std': z_std, 'z_median': z_median,
        'q25': q25, 'q75': q75, 'iqr': iqr,
        'skewness': skewness, 'kurtosis': kurtosis, 'cv': cv,
        'z_values': z_values
    }

def check_horizontal_scan(stats_dict):
    """
    判断是否为水平扫描
    基于Z轴变化范围和标准差
    """
    z_range = stats_dict['z_range']
    z_std = stats_dict['z_std']
    cv = stats_dict['cv']

    print("\n" + "=" * 60)
    print("4. 水平扫描判断：")
    print("=" * 60)

    # 判断标准：
    # - 范围小于1m且变异系数小于0.1：非常水平
    # - 范围小于2m且变异系数小于0.2：基本水平
    # - 否则：有明显高程变化

    if z_range < 1.0 and cv < 0.1:
        scan_type = "非常水平的扫描"
        is_horizontal = True
    elif z_range < 2.0 and cv < 0.2:
        scan_type = "基本水平的扫描"
        is_horizontal = True
    else:
        scan_type = "有明显高程变化的扫描"
        is_horizontal = False

    print(f"   扫描类型：{scan_type}")
    print(f"   Z轴范围：{z_range:.6f} m")
    print(f"   相对变异：{cv:.4f}")
    print(f"   是否水平：{'是' if is_horizontal else '否'}")

    return is_horizontal, scan_type

def analyze_z_pattern(data, stats_dict):
    """
    分析Z轴变化模式，检测倾斜和坡度
    """
    x_values = data[:, 0]  # X坐标
    y_values = data[:, 1]  # Y坐标
    z_values = stats_dict['z_values']

    print("\n" + "=" * 60)
    print("5. Z轴变化模式分析：")
    print("=" * 60)

    # 计算Z与X、Y的线性相关性
    corr_zx = np.corrcoef(z_values, x_values)[0, 1]
    corr_zy = np.corrcoef(z_values, y_values)[0, 1]

    print(f"   Z-X相关系数：{corr_zx:.4f}")
    print(f"   Z-Y相关系数：{corr_zy:.4f}")

    # 线性拟合计算坡度
    # Z = a*X + b*Y + c 平面拟合
    A = np.column_stack([x_values, y_values, np.ones(len(x_values))])
    coeffs, residuals, rank, s = np.linalg.lstsq(A, z_values, rcond=None)

    slope_x = coeffs[0]  # X方向坡度
    slope_y = coeffs[1]  # Y方向坡度

    # 计算总体坡度和坡向
    total_slope = np.sqrt(slope_x**2 + slope_y**2)
    slope_angle = np.degrees(np.arctan(total_slope))
    slope_direction = np.degrees(np.arctan2(slope_y, slope_x))

    # 计算拟合优度
    z_pred = A @ coeffs
    r2 = 1 - np.sum((z_values - z_pred)**2) / np.sum((z_values - np.mean(z_values))**2)

    print(f"   X方向坡度：{slope_x:.6f} (升高/单位距离)")
    print(f"   Y方向坡度：{slope_y:.6f} (升高/单位距离)")
    print(f"   总体坡度：{total_slope:.6f}")
    print(f"   坡度角：{slope_angle:.2f}°")
    print(f"   坡向：{slope_direction:.1f}° (0°为东，90°为北)")
    print(f"   平面拟合优度 R²：{r2:.4f}")

    # 坡度解释
    if total_slope < 0.01:
        slope_desc = "几乎平坦"
    elif total_slope < 0.05:
        slope_desc = "轻微倾斜"
    elif total_slope < 0.1:
        slope_desc = "中等倾斜"
    else:
        slope_desc = "明显倾斜"

    print(f"   坡度特征：{slope_desc}")

    # 检测局部变化（可能的凹坑）
    # 计算每个点与拟合平面的偏差
    deviations = z_values - z_pred
    dev_std = np.std(deviations)

    # 找出明显偏离平面的点
    outliers_low = deviations < -2 * dev_std  # 明显低于平面的点（可能的凹坑）
    outliers_high = deviations > 2 * dev_std   # 明显高于平面的点（可能的凸起）

    n_low_outliers = np.sum(outliers_low)
    n_high_outliers = np.sum(outliers_high)

    print(f"   平面偏差标准差：{dev_std:.6f} m")
    print(f"   低于平面的异常点：{n_low_outliers} 个 (可能的凹陷)")
    print(f"   高于平面的异常点：{n_high_outliers} 个 (可能的凸起)")

    if n_low_outliers > 0:
        min_deviation = np.min(deviations[outliers_low])
        print(f"   最深凹陷：{min_deviation:.6f} m")

    return {
        'corr_zx': corr_zx, 'corr_zy': corr_zy,
        'slope_x': slope_x, 'slope_y': slope_y,
        'total_slope': total_slope, 'slope_angle': slope_angle,
        'slope_direction': slope_direction, 'r2': r2,
        'slope_desc': slope_desc,
        'deviations': deviations, 'dev_std': dev_std,
        'n_low_outliers': n_low_outliers, 'n_high_outliers': n_high_outliers
    }

def create_visualizations(data, stats_dict, pattern_dict):
    """
    创建数据可视化图表
    """
    plt.style.use('default')
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('H103v2.asc Z轴数据分析', fontsize=16, fontweight='bold')

    z_values = stats_dict['z_values']
    x_values = data[:, 0]
    y_values = data[:, 1]

    # 1. Z轴直方图
    axes[0, 0].hist(z_values, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
    axes[0, 0].axvline(stats_dict['z_mean'], color='red', linestyle='--', label=f'均值: {stats_dict["z_mean"]:.3f}')
    axes[0, 0].axvline(stats_dict['z_median'], color='orange', linestyle='--', label=f'中位数: {stats_dict["z_median"]:.3f}')
    axes[0, 0].set_xlabel('Z值 (m)')
    axes[0, 0].set_ylabel('频数')
    axes[0, 0].set_title('Z轴数据分布直方图')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)

    # 2. 3D散点图投影到XY平面，用颜色表示Z值
    scatter = axes[0, 1].scatter(x_values, y_values, c=z_values, cmap='viridis', s=20, alpha=0.8)
    axes[0, 1].set_xlabel('X (m)')
    axes[0, 1].set_ylabel('Y (m)')
    axes[0, 1].set_title('点云XY投影 (颜色代表Z值)')
    axes[0, 1].set_aspect('equal')
    plt.colorbar(scatter, ax=axes[0, 1], label='Z (m)')

    # 3. Z值变化趋势（按点序号）
    point_indices = np.arange(len(z_values))
    axes[1, 0].plot(point_indices, z_values, 'b-', alpha=0.7, linewidth=1)
    axes[1, 0].axhline(stats_dict['z_mean'], color='red', linestyle='--', alpha=0.8)
    axes[1, 0].fill_between(point_indices,
                           stats_dict['z_mean'] - stats_dict['z_std'],
                           stats_dict['z_mean'] + stats_dict['z_std'],
                           alpha=0.2, color='red', label='±1σ')
    axes[1, 0].set_xlabel('点序号')
    axes[1, 0].set_ylabel('Z值 (m)')
    axes[1, 0].set_title('Z轴变化趋势')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # 4. 平面拟合偏差分布
    deviations = pattern_dict['deviations']
    axes[1, 1].hist(deviations, bins=30, alpha=0.7, color='lightcoral', edgecolor='black')
    axes[1, 1].axvline(0, color='red', linestyle='-', linewidth=2, label='拟合平面')
    axes[1, 1].axvline(-2*pattern_dict['dev_std'], color='blue', linestyle='--', label='-2σ (凹陷阈值)')
    axes[1, 1].axvline(2*pattern_dict['dev_std'], color='blue', linestyle='--', label='+2σ (凸起阈值)')
    axes[1, 1].set_xlabel('偏差 (m)')
    axes[1, 1].set_ylabel('频数')
    axes[1, 1].set_title('与拟合平面的偏差分布')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存图片
    output_path = '/home/serenNan/work/PCL_Visualization/analysis/z_axis_analysis.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"\n可视化图表已保存至：{output_path}")
    plt.close()

def main():
    """
    主函数
    """
    data_file = '/home/serenNan/work/PCL_Visualization/data/H103v2.asc'

    print("开始分析H103v2.asc文件中的Z轴数据...")
    print("=" * 60)

    # 加载数据
    data = load_point_cloud_data(data_file)
    if data is None:
        return

    # 分析Z轴统计特征
    stats_dict = analyze_z_axis(data)

    # 判断是否为水平扫描
    is_horizontal, scan_type = check_horizontal_scan(stats_dict)

    # 分析Z轴变化模式
    pattern_dict = analyze_z_pattern(data, stats_dict)

    # 创建可视化
    create_visualizations(data, stats_dict, pattern_dict)

    # 综合结论
    print("\n" + "=" * 60)
    print("综合分析结论：")
    print("=" * 60)
    print(f"• 点云包含 {len(stats_dict['z_values'])} 个测量点")
    print(f"• Z轴变化范围：{stats_dict['z_range']:.3f} m")
    print(f"• 扫描特征：{scan_type}")
    print(f"• 坡度特征：{pattern_dict['slope_desc']} (坡度角 {pattern_dict['slope_angle']:.1f}°)")

    if pattern_dict['n_low_outliers'] > 0:
        print(f"• 发现 {pattern_dict['n_low_outliers']} 个可能的凹陷点")
    if pattern_dict['n_high_outliers'] > 0:
        print(f"• 发现 {pattern_dict['n_high_outliers']} 个可能的凸起点")

    print(f"• 平面拟合质量：R² = {pattern_dict['r2']:.3f}")

    if stats_dict['z_range'] > 0.2:
        print("• 建议：Z轴变化较大，适合进行凹坑检测分析")
    else:
        print("• 建议：Z轴变化较小，可能需要更精细的检测算法")

if __name__ == "__main__":
    main()