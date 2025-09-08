#include "Camera3D.h"
#include <algorithm>
#include <cmath>

// OpenGL 相关头文件
#ifdef _WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

namespace pcl_viz {
namespace ui {

Camera3D::Camera3D()
    : m_eye({0.0f, 0.0f, 5.0f})
    , m_target({0.0f, 0.0f, 0.0f})
    , m_up({0.0f, 1.0f, 0.0f})
    , m_defaultEye({0.0f, 0.0f, 5.0f})
    , m_defaultTarget({0.0f, 0.0f, 0.0f})
    , m_defaultUp({0.0f, 1.0f, 0.0f})
    , m_minDistance(0.1f)
    , m_maxDistance(100.0f)
    , m_verticalAngle(0.0f)
    , m_horizontalAngle(0.0f)
    , m_maxVerticalAngle(85.0f * M_PI / 180.0f)   // 85度转弧度
    , m_minVerticalAngle(-85.0f * M_PI / 180.0f)  // -85度转弧度
{
}

void Camera3D::setPosition(float eyeX, float eyeY, float eyeZ) {
    m_eye = {eyeX, eyeY, eyeZ};
}

void Camera3D::setTarget(float targetX, float targetY, float targetZ) {
    m_target = {targetX, targetY, targetZ};
}

void Camera3D::setUp(float upX, float upY, float upZ) {
    m_up = {upX, upY, upZ};
    normalize(m_up);
}

void Camera3D::rotateLegacy(float deltaX, float deltaY, float sensitivity) {
    // 轨迹球旋转算法
    
    // 计算相机到目标的向量
    std::array<float, 3> direction = {
        m_eye[0] - m_target[0],
        m_eye[1] - m_target[1], 
        m_eye[2] - m_target[2]
    };
    float distance = length(direction);
    
    // 计算右向量和真正的上向量
    std::array<float, 3> forward = {-direction[0], -direction[1], -direction[2]};
    normalize(forward);
    
    std::array<float, 3> right = cross(forward, m_up);
    normalize(right);
    
    std::array<float, 3> realUp = cross(right, forward);
    normalize(realUp);
    
    // 水平旋转（绕上向量）
    float horizontalAngle = -deltaX * sensitivity;
    direction = rotateAroundAxis(direction, realUp, horizontalAngle);
    
    // 垂直旋转（绕右向量）
    float verticalAngle = -deltaY * sensitivity;
    direction = rotateAroundAxis(direction, right, verticalAngle);
    
    // 更新相机位置，保持距离不变
    normalize(direction);
    m_eye[0] = m_target[0] + direction[0] * distance;
    m_eye[1] = m_target[1] + direction[1] * distance;
    m_eye[2] = m_target[2] + direction[2] * distance;
}

void Camera3D::rotate(float deltaX, float deltaY, int windowWidth, int windowHeight, float sensitivity) {
    // 改进的轨迹球旋转算法，支持自适应灵敏度和角度限制
    
    // 计算相机到目标的向量和距离
    std::array<float, 3> direction = {
        m_eye[0] - m_target[0],
        m_eye[1] - m_target[1], 
        m_eye[2] - m_target[2]
    };
    float distance = length(direction);
    
    // 自适应灵敏度计算
    float adaptiveSensitivity = calculateAdaptiveSensitivity(windowWidth, windowHeight, sensitivity);
    
    // 标准化鼠标移动量到 [-1, 1] 范围
    float normalizedDeltaX = (2.0f * deltaX) / static_cast<float>(windowWidth);
    float normalizedDeltaY = (2.0f * deltaY) / static_cast<float>(windowHeight);
    
    // 计算相机坐标系
    std::array<float, 3> forward = {-direction[0], -direction[1], -direction[2]};
    normalize(forward);
    
    std::array<float, 3> right = cross(forward, m_up);
    normalize(right);
    
    std::array<float, 3> realUp = cross(right, forward);
    normalize(realUp);
    
    // 计算旋转角度
    float horizontalAngle = -normalizedDeltaX * adaptiveSensitivity;
    float verticalAngle = -normalizedDeltaY * adaptiveSensitivity;
    
    // 限制垂直旋转角度，防止翻转
    float currentVerticalAngle = m_verticalAngle + verticalAngle;
    currentVerticalAngle = clampVerticalAngle(currentVerticalAngle);
    verticalAngle = currentVerticalAngle - m_verticalAngle;
    
    // 应用旋转
    direction = rotateAroundAxis(direction, realUp, horizontalAngle);
    direction = rotateAroundAxis(direction, right, verticalAngle);
    
    // 更新相机位置，保持距离不变
    normalize(direction);
    m_eye[0] = m_target[0] + direction[0] * distance;
    m_eye[1] = m_target[1] + direction[1] * distance;
    m_eye[2] = m_target[2] + direction[2] * distance;
    
    // 更新角度追踪
    m_horizontalAngle += horizontalAngle;
    m_verticalAngle = currentVerticalAngle;
}

void Camera3D::zoom(float scaleFactor) {
    // 计算相机到目标的向量
    std::array<float, 3> direction = {
        m_eye[0] - m_target[0],
        m_eye[1] - m_target[1],
        m_eye[2] - m_target[2]
    };
    
    float currentDistance = length(direction);
    float newDistance = currentDistance * scaleFactor;
    
    // 限制缩放范围
    newDistance = std::max(m_minDistance, std::min(m_maxDistance, newDistance));
    
    // 更新相机位置
    normalize(direction);
    m_eye[0] = m_target[0] + direction[0] * newDistance;
    m_eye[1] = m_target[1] + direction[1] * newDistance;
    m_eye[2] = m_target[2] + direction[2] * newDistance;
}

void Camera3D::pan(float deltaX, float deltaY, float sensitivity) {
    // 计算相机坐标系
    std::array<float, 3> forward = {
        m_target[0] - m_eye[0],
        m_target[1] - m_eye[1],
        m_target[2] - m_eye[2]
    };
    normalize(forward);
    
    std::array<float, 3> right = cross(forward, m_up);
    normalize(right);
    
    std::array<float, 3> realUp = cross(right, forward);
    normalize(realUp);
    
    // 计算平移向量
    float distance = getDistance();
    float panScale = distance * sensitivity;
    
    std::array<float, 3> panVector = {
        right[0] * deltaX * panScale + realUp[0] * deltaY * panScale,
        right[1] * deltaX * panScale + realUp[1] * deltaY * panScale,
        right[2] * deltaX * panScale + realUp[2] * deltaY * panScale
    };
    
    // 同时移动相机和目标点
    m_eye[0] += panVector[0];
    m_eye[1] += panVector[1];
    m_eye[2] += panVector[2];
    
    m_target[0] += panVector[0];
    m_target[1] += panVector[1];
    m_target[2] += panVector[2];
}

void Camera3D::reset() {
    m_eye = m_defaultEye;
    m_target = m_defaultTarget;
    m_up = m_defaultUp;
    
    // 更新角度追踪
    updateAngleTracking();
}

void Camera3D::autoFit(float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
    // 计算点云中心
    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;
    float centerZ = (minZ + maxZ) * 0.5f;
    
    // 计算点云范围
    float spanX = maxX - minX;
    float spanY = maxY - minY;
    float spanZ = maxZ - minZ;
    float maxSpan = std::max({spanX, spanY, spanZ});
    
    // 设置目标点为点云中心
    setTarget(centerX, centerY, centerZ);
    
    // 计算合适的相机距离
    float distance = maxSpan * 2.5f; // 留出足够空间观察
    if (distance < 1.0f) distance = 5.0f; // 最小距离
    
    // 设置相机位置（从右上方观察）
    setPosition(
        centerX + distance * 0.5f,
        centerY + distance * 0.3f, 
        centerZ + distance * 0.8f
    );
    
    // 更新缩放限制
    m_minDistance = maxSpan * 0.1f;
    m_maxDistance = maxSpan * 10.0f;
    
    // 保存为默认位置
    m_defaultEye = m_eye;
    m_defaultTarget = m_target;
    m_defaultUp = m_up;
    
    // 更新角度追踪
    updateAngleTracking();
}

void Camera3D::applyTransform() {
    // 手动实现 gluLookAt 等效变换
    // 计算相机坐标系
    std::array<float, 3> forward = {
        m_target[0] - m_eye[0],
        m_target[1] - m_eye[1],
        m_target[2] - m_eye[2]
    };
    normalize(forward);
    
    std::array<float, 3> right = cross(forward, m_up);
    normalize(right);
    
    std::array<float, 3> realUp = cross(right, forward);
    
    // 构建视图变换矩阵
    float viewMatrix[16] = {
        right[0],      realUp[0],      -forward[0],     0.0f,
        right[1],      realUp[1],      -forward[1],     0.0f,
        right[2],      realUp[2],      -forward[2],     0.0f,
        -dot(right, m_eye), -dot(realUp, m_eye), dot(forward, m_eye), 1.0f
    };
    
    // 应用变换矩阵
    glMultMatrixf(viewMatrix);
}

float Camera3D::getDistance() const {
    std::array<float, 3> direction = {
        m_eye[0] - m_target[0],
        m_eye[1] - m_target[1],
        m_eye[2] - m_target[2]
    };
    return length(direction);
}

void Camera3D::normalize(std::array<float, 3>& vec) {
    float len = length(vec);
    if (len > 1e-6f) {
        vec[0] /= len;
        vec[1] /= len;
        vec[2] /= len;
    }
}

std::array<float, 3> Camera3D::cross(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

float Camera3D::dot(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float Camera3D::length(const std::array<float, 3>& vec) const {
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

std::array<float, 3> Camera3D::rotateAroundAxis(const std::array<float, 3>& vec, 
                                               const std::array<float, 3>& axis, 
                                               float angle) {
    // 罗德里格旋转公式 (Rodrigues' rotation formula)
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    
    std::array<float, 3> k = axis; // 假设轴已归一化
    float dotProduct = dot(vec, k);
    std::array<float, 3> crossProduct = cross(k, vec);
    
    return {
        vec[0] * cosAngle + crossProduct[0] * sinAngle + k[0] * dotProduct * (1 - cosAngle),
        vec[1] * cosAngle + crossProduct[1] * sinAngle + k[1] * dotProduct * (1 - cosAngle),
        vec[2] * cosAngle + crossProduct[2] * sinAngle + k[2] * dotProduct * (1 - cosAngle)
    };
}

float Camera3D::calculateAdaptiveSensitivity(int windowWidth, int windowHeight, float baseSensitivity) const {
    // 基于相机距离、窗口大小和点云尺度的自适应灵敏度算法
    
    float distance = getDistance();
    float windowSize = std::sqrt(static_cast<float>(windowWidth * windowHeight));
    
    // 基础公式：灵敏度与距离成正比，与窗口大小成反比
    // 这确保了远距离时不会过于敏感，近距离时不会过于迟钝
    float adaptiveScale = distance / (windowSize * 0.01f);
    
    // 限制灵敏度范围，避免极端值
    adaptiveScale = std::max(0.1f, std::min(10.0f, adaptiveScale));
    
    return baseSensitivity * adaptiveScale;
}

float Camera3D::clampVerticalAngle(float angle) const {
    return std::max(m_minVerticalAngle, std::min(m_maxVerticalAngle, angle));
}

void Camera3D::updateAngleTracking() {
    // 根据当前相机位置更新角度追踪
    // 这个函数在相机位置发生非旋转变化时调用（如缩放、平移等）
    
    std::array<float, 3> direction = {
        m_eye[0] - m_target[0],
        m_eye[1] - m_target[1],
        m_eye[2] - m_target[2]
    };
    normalize(direction);
    
    // 计算水平角度（绕Y轴）
    m_horizontalAngle = std::atan2(direction[0], direction[2]);
    
    // 计算垂直角度
    float horizontalDistance = std::sqrt(direction[0] * direction[0] + direction[2] * direction[2]);
    m_verticalAngle = std::atan2(-direction[1], horizontalDistance);
    
    // 确保垂直角度在有效范围内
    m_verticalAngle = clampVerticalAngle(m_verticalAngle);
}

} // namespace ui
} // namespace pcl_viz