#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QFrame>
#include <QtCore/QTimer>

#include "../visualization/Visualizer.h"

namespace pcl_viz {
namespace ui {

/**
 * @brief 坑洞检测参数配置
 */
struct PotholeDetectionConfig {
    // 预处理参数
    double downsampleVoxelSize;      ///< 下采样体素大小 (mm)
    bool enableStatisticalFilter;    ///< 启用统计滤波
    int statisticalFilterK;          ///< 统计滤波邻居数
    double statisticalFilterStddev;  ///< 统计滤波标准差倍数

    // 表面重建参数  
    double searchRadius;             ///< 搜索半径 (mm)
    int polynomialOrder;             ///< 多项式阶数
    
    // 坑洞检测参数
    double depthThreshold;           ///< 深度阈值 (mm)
    double areaThreshold;            ///< 最小面积阈值 (mm²)
    double volumeThreshold;          ///< 最小体积阈值 (mm³)
    double slopeAngle;               ///< 坡度角度阈值 (度)

    // 聚类参数
    double clusterTolerance;         ///< 聚类容差 (mm)
    int minClusterSize;              ///< 最小聚类大小
    int maxClusterSize;              ///< 最大聚类大小

    PotholeDetectionConfig() 
        : downsampleVoxelSize(1.0), enableStatisticalFilter(true)
        , statisticalFilterK(50), statisticalFilterStddev(1.0)
        , searchRadius(5.0), polynomialOrder(2)
        , depthThreshold(2.0), areaThreshold(100.0), volumeThreshold(50.0)
        , slopeAngle(15.0), clusterTolerance(2.0)
        , minClusterSize(10), maxClusterSize(10000) {}
};

/**
 * @brief 控制参数面板
 * 
 * 提供各种参数调节控件，包括可视化参数、检测参数等
 * 允许用户实时调整算法参数并查看效果
 */
class ControlPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit ControlPanel(QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~ControlPanel() override;

    /**
     * @brief 获取当前可视化配置
     * @return 可视化配置
     */
    visualization::VisualizationConfig getVisualizationConfig() const;

    /**
     * @brief 设置可视化配置
     * @param config 可视化配置
     */
    void setVisualizationConfig(const visualization::VisualizationConfig& config);

    /**
     * @brief 获取坑洞检测配置
     * @return 检测配置
     */
    PotholeDetectionConfig getDetectionConfig() const;

    /**
     * @brief 设置坑洞检测配置
     * @param config 检测配置
     */
    void setDetectionConfig(const PotholeDetectionConfig& config);

    /**
     * @brief 重置为默认参数
     */
    void resetToDefaults();

    /**
     * @brief 启用/禁用所有控件
     * @param enabled 是否启用
     */
    void setControlsEnabled(bool enabled);

public slots:
    /**
     * @brief 开始坑洞检测分析
     */
    void onStartAnalysis();

    /**
     * @brief 停止分析
     */
    void onStopAnalysis();

    /**
     * @brief 重置参数
     */
    void onResetParameters();

    /**
     * @brief 保存当前参数配置
     */
    void onSaveConfig();

    /**
     * @brief 加载参数配置
     */
    void onLoadConfig();

signals:
    /**
     * @brief 可视化配置改变信号
     * @param config 新的配置
     */
    void visualizationConfigChanged(const visualization::VisualizationConfig& config);

    /**
     * @brief 检测配置改变信号
     * @param config 新的配置
     */
    void detectionConfigChanged(const PotholeDetectionConfig& config);

    /**
     * @brief 请求开始分析信号
     * @param config 检测配置
     */
    void analysisRequested(const PotholeDetectionConfig& config);

    /**
     * @brief 请求停止分析信号
     */
    void analysisStopRequested();

    /**
     * @brief 参数重置信号
     */
    void parametersReset();

private slots:
    /**
     * @brief 处理点大小改变
     * @param value 新的点大小
     */
    void onPointSizeChanged(double value);

    /**
     * @brief 处理颜色模式改变
     * @param index 颜色模式索引
     */
    void onColorModeChanged(int index);

    /**
     * @brief 处理背景颜色改变
     * @param checked 是否选中暗色背景
     */
    void onBackgroundColorChanged(bool checked);

    /**
     * @brief 处理坐标系显示切换
     * @param checked 是否显示坐标系
     */
    void onCoordinateSystemToggled(bool checked);

    /**
     * @brief 处理检测参数改变
     */
    void onDetectionParameterChanged();

    /**
     * @brief 延迟发送参数改变信号
     */
    void onParameterChangeDelayed();

private:
    /**
     * @brief 初始化UI界面
     */
    void initializeUI();

    /**
     * @brief 创建可视化控制组
     * @return 可视化控制组合框
     */
    QGroupBox* createVisualizationGroup();

    /**
     * @brief 创建预处理参数组
     * @return 预处理参数组合框
     */
    QGroupBox* createPreprocessingGroup();

    /**
     * @brief 创建检测参数组
     * @return 检测参数组合框
     */
    QGroupBox* createDetectionGroup();

    /**
     * @brief 创建聚类参数组
     * @return 聚类参数组合框
     */
    QGroupBox* createClusteringGroup();

    /**
     * @brief 创建操作按钮组
     * @return 操作按钮布局
     */
    QVBoxLayout* createActionButtons();

    /**
     * @brief 创建参数控制项
     * @param layout 父布局
     * @param row 行号
     * @param labelText 标签文本
     * @param minValue 最小值
     * @param maxValue 最大值
     * @param defaultValue 默认值
     * @param stepSize 步长
     * @param decimals 小数位数
     * @return 创建的SpinBox指针
     */
    QDoubleSpinBox* createParameterControl(QGridLayout* layout, int row,
                                          const QString& labelText,
                                          double minValue, double maxValue,
                                          double defaultValue, double stepSize = 0.1,
                                          int decimals = 2);

    /**
     * @brief 创建整数参数控制项
     * @param layout 父布局
     * @param row 行号
     * @param labelText 标签文本
     * @param minValue 最小值
     * @param maxValue 最大值
     * @param defaultValue 默认值
     * @return 创建的SpinBox指针
     */
    QSpinBox* createIntParameterControl(QGridLayout* layout, int row,
                                       const QString& labelText,
                                       int minValue, int maxValue,
                                       int defaultValue);

    /**
     * @brief 连接所有信号和槽
     */
    void connectSignalsAndSlots();

    /**
     * @brief 更新UI控件值（不触发信号）
     */
    void updateControlsWithoutSignals();

    /**
     * @brief 设置控件样式
     */
    void setupStyles();

    // 布局组件
    QVBoxLayout* m_mainLayout;                   ///< 主布局

    // 可视化控制组件
    QGroupBox* m_visualizationGroup;             ///< 可视化组
    QDoubleSpinBox* m_pointSizeSpinBox;          ///< 点大小控制
    QComboBox* m_colorModeComboBox;              ///< 颜色模式选择
    QCheckBox* m_darkBackgroundCheckBox;         ///< 暗色背景选择
    QCheckBox* m_coordinateSystemCheckBox;       ///< 坐标系显示选择

    // 预处理参数组件
    QGroupBox* m_preprocessingGroup;             ///< 预处理组
    QDoubleSpinBox* m_voxelSizeSpinBox;          ///< 下采样体素大小
    QCheckBox* m_statisticalFilterCheckBox;      ///< 统计滤波开关
    QSpinBox* m_filterKSpinBox;                  ///< 滤波邻居数
    QDoubleSpinBox* m_filterStddevSpinBox;       ///< 滤波标准差

    // 检测参数组件
    QGroupBox* m_detectionGroup;                 ///< 检测组
    QDoubleSpinBox* m_searchRadiusSpinBox;       ///< 搜索半径
    QSpinBox* m_polynomialOrderSpinBox;          ///< 多项式阶数
    QDoubleSpinBox* m_depthThresholdSpinBox;     ///< 深度阈值
    QDoubleSpinBox* m_areaThresholdSpinBox;      ///< 面积阈值
    QDoubleSpinBox* m_volumeThresholdSpinBox;    ///< 体积阈值
    QDoubleSpinBox* m_slopeAngleSpinBox;         ///< 坡度角度

    // 聚类参数组件
    QGroupBox* m_clusteringGroup;                ///< 聚类组
    QDoubleSpinBox* m_clusterToleranceSpinBox;   ///< 聚类容差
    QSpinBox* m_minClusterSizeSpinBox;           ///< 最小聚类大小
    QSpinBox* m_maxClusterSizeSpinBox;           ///< 最大聚类大小

    // 操作按钮
    QPushButton* m_startAnalysisButton;          ///< 开始分析按钮
    QPushButton* m_stopAnalysisButton;           ///< 停止分析按钮
    QPushButton* m_resetButton;                  ///< 重置按钮
    QPushButton* m_saveConfigButton;             ///< 保存配置按钮
    QPushButton* m_loadConfigButton;             ///< 加载配置按钮

    // 配置数据
    visualization::VisualizationConfig m_vizConfig;     ///< 可视化配置
    PotholeDetectionConfig m_detectionConfig;           ///< 检测配置

    // 控制状态
    bool m_analysisRunning;                      ///< 分析是否正在运行
    bool m_updatingControls;                     ///< 是否正在更新控件
    QString m_lastConfigPath;                    ///< 上次配置文件路径

    // 延迟更新定时器
    QTimer* m_parameterUpdateTimer;              ///< 参数更新延迟定时器
};

} // namespace ui
} // namespace pcl_viz