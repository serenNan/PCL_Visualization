#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QStyle>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QFrame>
#include <QtCore/QTimer>

#include "../core/PointCloud.h"

namespace pcl_viz {
namespace ui {

/**
 * @brief 坑洞测量结果
 */
struct PotholeResult {
    double volume;         ///< 体积 (mm³)
    double area;           ///< 面积 (mm²)
    double diameter;       ///< 直径/尺寸 (mm)
    double maxDepth;       ///< 最大深度 (mm)
    int pointCount;        ///< 点云数量
    bool isValid;          ///< 结果是否有效

    PotholeResult()
        : volume(0.0), area(0.0), diameter(0.0)
        , maxDepth(0.0), pointCount(0), isValid(false) {}
};

/**
 * @brief 结果显示面板
 * 
 * 显示坑洞检测和测量结果的面板，包括深度、体积、面积等信息
 * 提供清晰的数据展示和导出功能
 */
class ResultPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit ResultPanel(QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~ResultPanel() override;

    /**
     * @brief 更新显示结果
     * @param result 坑洞测量结果
     */
    void updateResults(const PotholeResult& result);

    /**
     * @brief 清空所有结果显示
     */
    void clearResults();

    /**
     * @brief 设置点云信息
     * @param pointCloud 点云对象
     */
    void setPointCloudInfo(std::shared_ptr<core::PointCloud> pointCloud);

    /**
     * @brief 设置分析状态
     * @param analyzing 是否正在分析
     * @param progress 进度百分比 (0-100)
     */
    void setAnalysisStatus(bool analyzing, int progress = 0);

    /**
     * @brief 获取当前结果
     * @return 当前的测量结果
     */
    const PotholeResult& getCurrentResult() const { return m_currentResult; }

    /**
     * @brief 设置结果有效性
     * @param valid 结果是否有效
     */
    void setResultsValid(bool valid);
    
private:
    /**
     * @brief 导出结果为JSON格式
     * @param filename 文件名
     * @return 是否成功
     */
    bool exportAsJson(const QString& filename);
    
    /**
     * @brief 导出结果为CSV格式
     * @param filename 文件名
     * @return 是否成功
     */
    bool exportAsCsv(const QString& filename);
    
    /**
     * @brief 导出结果为文本格式
     * @param filename 文件名
     * @return 是否成功
     */
    bool exportAsText(const QString& filename);

public slots:
    /**
     * @brief 导出结果到文件
     */
    void onExportResults();

    /**
     * @brief 复制结果到剪贴板
     */
    void onCopyToClipboard();

    /**
     * @brief 刷新显示
     */
    void onRefreshDisplay();

    /**
     * @brief 重置所有结果
     */
    void onResetResults();

signals:
    /**
     * @brief 请求重新分析信号
     */
    void analysisRequested();

    /**
     * @brief 结果导出完成信号
     * @param filename 导出的文件名
     * @param success 是否成功
     */
    void resultsExported(const QString& filename, bool success);

private slots:
    /**
     * @brief 更新时间显示
     */
    void updateTimeDisplay();

private:
    /**
     * @brief 初始化UI界面
     */
    void initializeUI();

    /**
     * @brief 创建测量结果组
     * @return 测量结果组合框
     */
    QGroupBox* createMeasurementGroup();

    /**
     * @brief 创建点云信息组
     * @return 点云信息组合框  
     */
    QGroupBox* createPointCloudInfoGroup();

    /**
     * @brief 创建分析状态组
     * @return 分析状态组合框
     */
    QGroupBox* createAnalysisStatusGroup();

    /**
     * @brief 创建操作按钮组
     * @return 操作按钮布局
     */
    QHBoxLayout* createActionButtons();

    /**
     * @brief 创建带单位的数值显示标签
     * @param value 数值
     * @param unit 单位
     * @param precision 精度
     * @return 格式化的字符串
     */
    QString formatValueWithUnit(double value, const QString& unit, int precision = 2) const;

    /**
     * @brief 创建结果项显示
     * @param layout 父布局
     * @param row 行号
     * @param labelText 标签文本
     * @param valueEdit 值编辑框引用
     * @param unit 单位文本
     */
    void createResultItem(QGridLayout* layout, int row, const QString& labelText, 
                         QLineEdit*& valueEdit, const QString& unit = "");

    /**
     * @brief 更新单个结果显示
     * @param edit 编辑框
     * @param value 数值
     * @param unit 单位
     * @param precision 精度
     */
    void updateResultDisplay(QLineEdit* edit, double value, const QString& unit = "", int precision = 2);

    /**
     * @brief 设置控件样式
     */
    void setupStyles();

    // 布局组件
    QVBoxLayout* m_mainLayout;                   ///< 主布局

    // 测量结果显示
    QGroupBox* m_measurementGroup;               ///< 测量结果组
    QLineEdit* m_volumeEdit;                     ///< 体积显示
    QLineEdit* m_areaEdit;                       ///< 面积显示
    QLineEdit* m_diameterEdit;                   ///< 直径/尺寸显示
    QLineEdit* m_maxDepthEdit;                   ///< 最大深度显示

    // 点云信息显示
    QGroupBox* m_pointCloudGroup;                ///< 点云信息组
    QLabel* m_fileNameLabel;                     ///< 文件名标签
    QLabel* m_pointCountLabel;                   ///< 点数量标签
    QLabel* m_boundsLabel;                       ///< 边界信息标签

    // 分析状态显示
    QGroupBox* m_statusGroup;                    ///< 状态组
    QLabel* m_analysisStatusLabel;               ///< 分析状态标签
    QProgressBar* m_analysisProgressBar;         ///< 分析进度条
    QLabel* m_timestampLabel;                    ///< 时间戳标签

    // 操作按钮
    QPushButton* m_exportButton;                 ///< 导出按钮
    QPushButton* m_copyButton;                   ///< 复制按钮
    QPushButton* m_refreshButton;                ///< 刷新按钮
    QPushButton* m_resetButton;                  ///< 重置按钮

    // 数据成员
    PotholeResult m_currentResult;               ///< 当前测量结果
    std::shared_ptr<core::PointCloud> m_currentPointCloud;  ///< 当前点云
    QTimer* m_timeUpdateTimer;                   ///< 时间更新定时器
    
    // 状态成员
    bool m_analyzing;                            ///< 是否正在分析
    bool m_resultsValid;                         ///< 结果是否有效
    QString m_lastExportPath;                    ///< 上次导出路径
};

} // namespace ui
} // namespace pcl_viz