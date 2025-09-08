#pragma once

#include <memory>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QWidget>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QLabel>
#include <QTimer>
#include <QActionGroup>
#include <future>
#include <memory>

#include "../core/PointCloud.h"
#include "../analysis/AnalysisResult.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QActionGroup;
QT_END_NAMESPACE

namespace pcl_viz {

namespace analysis {
    class PotholeDetector;
}

namespace ui {
    class VisualizerWidget;
    class ResultPanel;
}

namespace ui {

/**
 * @brief 主窗口类
 * 
 * 应用程序的主窗口，负责整体布局和各个组件的协调
 * 包含菜单栏、工具栏、状态栏和主要的UI组件
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit MainWindow(QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~MainWindow() override;

    /**
     * @brief 加载点云文件
     * @param filename 文件路径
     * @return 是否加载成功
     */
    bool loadPointCloud(const QString& filename);

public slots:
    /**
     * @brief 打开文件对话框加载点云
     */
    void onOpenFile();

    /**
     * @brief 保存当前场景截图
     */
    void onSaveScreenshot();

    /**
     * @brief 显示关于对话框
     */
    void onAbout();

    /**
     * @brief 退出应用程序
     */
    void onExit();

    /**
     * @brief 重置相机视角
     */
    void onResetCamera();

    /**
     * @brief 显示/隐藏结果面板
     */
    void onToggleResultPanel();

    /**
     * @brief 开始凹坑检测分析
     */
    void onStartAnalysis();

    /**
     * @brief 停止当前分析
     */
    void onStopAnalysis();

    /**
     * @brief 打开分析参数设置对话框
     */
    void onAnalysisSettings();


signals:
    /**
     * @brief 点云加载完成信号
     * @param pointCloud 加载的点云对象
     */
    void pointCloudLoaded(std::shared_ptr<core::PointCloud> pointCloud);

    /**
     * @brief 状态更新信号
     * @param message 状态消息
     */
    void statusUpdate(const QString& message);

    /**
     * @brief 分析完成信号
     * @param result 分析结果
     */
    void analysisCompleted(const analysis::AnalysisResult& result);

    /**
     * @brief 分析进度更新信号
     * @param stage 当前阶段
     * @param progress 进度百分比
     * @param message 状态消息
     */
    void analysisProgressUpdated(const QString& stage, int progress, const QString& message);

protected:
    /**
     * @brief 关闭事件处理
     * @param event 关闭事件
     */
    void closeEvent(QCloseEvent *event) override;

private slots:
    /**
     * @brief 更新状态栏时间显示
     */
    void updateStatusTime();

    /**
     * @brief 处理点云数据更新
     * @param pointCloud 更新的点云对象
     */
    void onPointCloudUpdated(std::shared_ptr<core::PointCloud> pointCloud);

    /**
     * @brief 处理分析结果更新
     * @param result 完整的分析结果
     */
    void onAnalysisResultUpdated(const analysis::AnalysisResult& result);

    /**
     * @brief 处理分析进度更新
     * @param stage 当前阶段
     * @param progress 进度百分比
     * @param message 状态消息
     */
    void onAnalysisProgressUpdated(const QString& stage, int progress, const QString& message);

    /**
     * @brief 处理分析完成
     * @param result 分析结果
     */
    void onAnalysisFinished(const analysis::AnalysisResult& result);

    /**
     * @brief 主题改变时的处理函数
     */
    void onThemeChanged();

    /**
     * @brief 字体大小改变时的处理函数
     */
    void onFontSizeChanged();

    /**
     * @brief 处理主题切换
     */
    void onLightTheme();
    void onDarkTheme();
    void onHighContrastTheme();

    /**
     * @brief 处理字体大小切换
     */
    void onSmallFont();
    void onNormalFont();
    void onLargeFont();
    void onExtraLargeFont();

private:
    /**
     * @brief 初始化UI界面
     */
    void initializeUI();

    /**
     * @brief 创建菜单栏
     */
    void createMenuBar();

    /**
     * @brief 创建工具栏
     */
    void createToolBar();

    /**
     * @brief 创建状态栏
     */
    void createStatusBar();

    /**
     * @brief 创建中央窗口部件
     */
    void createCentralWidget();

    /**
     * @brief 连接信号和槽
     */
    void connectSignalsAndSlots();

    /**
     * @brief 设置分析引擎
     */
    void setupAnalysisEngine();

    /**
     * @brief 自动开始深度分析
     */
    void startAutomaticAnalysis();

    /**
     * @brief 应用主题样式
     */
    void applyTheme();

    /**
     * @brief 添加主题相关菜单项
     */
    void addThemeMenuItems();

    /**
     * @brief 读取应用程序设置
     */
    void readSettings();

    /**
     * @brief 保存应用程序设置
     */
    void writeSettings();

    // UI 组件指针
    QWidget* m_centralWidget;                    ///< 中央窗口部件
    QHBoxLayout* m_mainLayout;                   ///< 主布局
    QSplitter* m_horizontalSplitter;             ///< 水平分割器

    // 功能面板
    VisualizerWidget* m_visualizerWidget;        ///< 3D可视化组件
    ResultPanel* m_resultPanel;                  ///< 结果显示面板

    // 菜单和动作
    QMenu* m_fileMenu;                           ///< 文件菜单
    QMenu* m_viewMenu;                           ///< 视图菜单
    QMenu* m_helpMenu;                           ///< 帮助菜单

    QAction* m_openAction;                       ///< 打开文件动作
    QAction* m_saveScreenshotAction;             ///< 保存截图动作
    QAction* m_exitAction;                       ///< 退出动作
    QAction* m_resetCameraAction;                ///< 重置相机动作
    QAction* m_toggleResultPanelAction;          ///< 切换结果面板动作
    QAction* m_aboutAction;                      ///< 关于动作

    // 分析相关动作
    QAction* m_startAnalysisAction;              ///< 开始分析动作
    QAction* m_stopAnalysisAction;               ///< 停止分析动作
    QAction* m_analysisSettingsAction;           ///< 分析设置动作

    // 主题相关动作
    QAction* m_lightThemeAction;                 ///< 浅色主题动作
    QAction* m_darkThemeAction;                  ///< 深色主题动作
    QAction* m_highContrastThemeAction;          ///< 高对比度主题动作
    QActionGroup* m_themeActionGroup;            ///< 主题动作组

    // 字体大小相关动作
    QAction* m_smallFontAction;                  ///< 小字体动作
    QAction* m_normalFontAction;                 ///< 正常字体动作
    QAction* m_largeFontAction;                  ///< 大字体动作
    QAction* m_extraLargeFontAction;             ///< 特大字体动作
    QActionGroup* m_fontSizeActionGroup;         ///< 字体大小动作组

    // 工具栏
    QToolBar* m_mainToolBar;                     ///< 主工具栏

    // 状态栏组件
    QLabel* m_statusLabel;                       ///< 状态标签
    QLabel* m_timeLabel;                         ///< 时间标签
    QProgressBar* m_progressBar;                 ///< 进度条

    // 定时器
    QTimer* m_statusTimer;                       ///< 状态栏时间更新定时器

    // 数据成员
    std::shared_ptr<core::PointCloud> m_currentPointCloud;  ///< 当前加载的点云
    QString m_currentFileName;                   ///< 当前文件名
    QString m_lastOpenDirectory;                 ///< 上次打开的目录

    // 窗口状态
    bool m_resultPanelVisible;                   ///< 结果面板可见性

    // 分析相关成员
    std::unique_ptr<analysis::PotholeDetector> m_potholeDetector;  ///< 凹坑检测器
    analysis::AnalysisParams m_analysisParams;  ///< 分析参数
    analysis::AnalysisResult m_lastAnalysisResult; ///< 上次分析结果
    bool m_analysisInProgress;                   ///< 分析是否进行中
    std::future<analysis::AnalysisResult> m_analysisFuture; ///< 异步分析结果
};

} // namespace ui
} // namespace pcl_viz