#pragma once

#include <array>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace pcl_viz {
namespace ui {

/**
 * @brief 3D相机控制类
 * 
 * 实现轨迹球相机控制，支持鼠标拖动旋转、滚轮缩放等交互功能
 */
class Camera3D {
public:
    /**
     * @brief 构造函数
     */
    Camera3D();

    /**
     * @brief 设置相机位置
     * @param eyeX 相机X坐标
     * @param eyeY 相机Y坐标
     * @param eyeZ 相机Z坐标
     */
    void setPosition(float eyeX, float eyeY, float eyeZ);

    /**
     * @brief 设置观察目标点
     * @param targetX 目标X坐标
     * @param targetY 目标Y坐标
     * @param targetZ 目标Z坐标
     */
    void setTarget(float targetX, float targetY, float targetZ);

    /**
     * @brief 设置上向量
     * @param upX 上向量X分量
     * @param upY 上向量Y分量
     * @param upZ 上向量Z分量
     */
    void setUp(float upX, float upY, float upZ);

    /**
     * @brief 鼠标旋转（轨迹球算法）
     * @param deltaX X方向鼠标移动量（像素）
     * @param deltaY Y方向鼠标移动量（像素）
     * @param sensitivity 敏感度
     */
    void rotate(float deltaX, float deltaY, float sensitivity = 0.01f);

    /**
     * @brief 缩放相机距离
     * @param scaleFactor 缩放因子（>1放大，<1缩小）
     */
    void zoom(float scaleFactor);

    /**
     * @brief 平移相机
     * @param deltaX X方向平移量
     * @param deltaY Y方向平移量
     * @param sensitivity 敏感度
     */
    void pan(float deltaX, float deltaY, float sensitivity = 0.001f);

    /**
     * @brief 重置相机到默认位置
     */
    void reset();

    /**
     * @brief 根据点云边界自动设置相机位置
     * @param minX 最小X坐标
     * @param maxX 最大X坐标
     * @param minY 最小Y坐标
     * @param maxY 最大Y坐标
     * @param minZ 最小Z坐标
     * @param maxZ 最大Z坐标
     */
    void autoFit(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

    /**
     * @brief 应用相机变换到OpenGL（使用gluLookAt等效变换）
     */
    void applyTransform();

    /**
     * @brief 获取相机位置
     * @return 相机位置数组 [x, y, z]
     */
    const std::array<float, 3>& getPosition() const { return m_eye; }

    /**
     * @brief 获取目标位置
     * @return 目标位置数组 [x, y, z]
     */
    const std::array<float, 3>& getTarget() const { return m_target; }

    /**
     * @brief 获取上向量
     * @return 上向量数组 [x, y, z]
     */
    const std::array<float, 3>& getUp() const { return m_up; }

    /**
     * @brief 获取相机到目标的距离
     * @return 距离值
     */
    float getDistance() const;

private:
    /**
     * @brief 归一化向量
     * @param vec 待归一化的向量
     */
    void normalize(std::array<float, 3>& vec);

    /**
     * @brief 向量叉积
     * @param a 向量a
     * @param b 向量b
     * @return 叉积结果
     */
    std::array<float, 3> cross(const std::array<float, 3>& a, const std::array<float, 3>& b);

    /**
     * @brief 向量点积
     * @param a 向量a
     * @param b 向量b
     * @return 点积结果
     */
    float dot(const std::array<float, 3>& a, const std::array<float, 3>& b);

    /**
     * @brief 向量长度
     * @param vec 向量
     * @return 长度
     */
    float length(const std::array<float, 3>& vec) const;

    /**
     * @brief 绕任意轴旋转向量
     * @param vec 待旋转向量
     * @param axis 旋转轴
     * @param angle 旋转角度（弧度）
     * @return 旋转后的向量
     */
    std::array<float, 3> rotateAroundAxis(const std::array<float, 3>& vec, 
                                         const std::array<float, 3>& axis, 
                                         float angle);

    std::array<float, 3> m_eye;        ///< 相机位置
    std::array<float, 3> m_target;     ///< 观察目标点
    std::array<float, 3> m_up;         ///< 上向量
    
    // 默认参数
    std::array<float, 3> m_defaultEye;
    std::array<float, 3> m_defaultTarget;
    std::array<float, 3> m_defaultUp;
    
    float m_minDistance;    ///< 最小缩放距离
    float m_maxDistance;    ///< 最大缩放距离
};

} // namespace ui
} // namespace pcl_viz