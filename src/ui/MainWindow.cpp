#include "MainWindow.h"
#include "VisualizerWidget.h"
#include "ResultPanel.h"
#include "ThemeManager.h"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtCore/QTimer>
#include <QtCore/QDateTime>
#include <QtCore/QSettings>
#include <QtCore/QStandardPaths>
#include <QtCore/QDir>
#include <QtCore/QDebug>
#include <QtGui/QCloseEvent>
#include <QtGui/QKeySequence>

#include "../core/PointCloudLoader.h"
#include "../analysis/PotholeDetector.h"

namespace pcl_viz {
namespace ui {

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_centralWidget(nullptr)
    , m_mainLayout(nullptr)
    , m_horizontalSplitter(nullptr)
    , m_visualizerWidget(nullptr)
    , m_resultPanel(nullptr)
    , m_fileMenu(nullptr)
    , m_viewMenu(nullptr)
    , m_helpMenu(nullptr)
    , m_openAction(nullptr)
    , m_saveScreenshotAction(nullptr)
    , m_exitAction(nullptr)
    , m_resetCameraAction(nullptr)
    , m_toggleResultPanelAction(nullptr)
    , m_aboutAction(nullptr)
    , m_startAnalysisAction(nullptr)
    , m_stopAnalysisAction(nullptr)
    , m_analysisSettingsAction(nullptr)
    , m_lightThemeAction(nullptr)
    , m_darkThemeAction(nullptr)
    , m_highContrastThemeAction(nullptr)
    , m_themeActionGroup(nullptr)
    , m_smallFontAction(nullptr)
    , m_normalFontAction(nullptr)
    , m_largeFontAction(nullptr)
    , m_extraLargeFontAction(nullptr)
    , m_fontSizeActionGroup(nullptr)
    , m_mainToolBar(nullptr)
    , m_statusLabel(nullptr)
    , m_timeLabel(nullptr)
    , m_progressBar(nullptr)
    , m_statusTimer(nullptr)
    , m_currentPointCloud(nullptr)
    , m_resultPanelVisible(true)
    , m_potholeDetector(std::make_unique<analysis::PotholeDetector>())
    , m_analysisParams(analysis::createDefaultAnalysisParams())
    , m_analysisInProgress(false)
{
    // 设置窗口属性
    setWindowTitle("PCL 点云可视化与坑洞检测系统");
    setMinimumSize(1200, 800);
    resize(1600, 1000);

    // 初始化界面
    initializeUI();
    
    // 设置分析器
    setupAnalysisEngine();
    
    // 连接信号和槽
    connectSignalsAndSlots();
    
    // 应用主题样式
    applyTheme();
    
    // 读取设置
    readSettings();
    
    // 启动状态栏时间更新
    m_statusTimer = new QTimer(this);
    connect(m_statusTimer, &QTimer::timeout, this, &MainWindow::updateStatusTime);
    m_statusTimer->start(1000); // 每秒更新一次
    
    // 初始状态栏信息
    updateStatusTime();
    m_statusLabel->setText("就绪 - 请加载点云文件");
}

MainWindow::~MainWindow() {
    // 保存设置
    writeSettings();
}

bool MainWindow::loadPointCloud(const QString& filename) {
    if (filename.isEmpty()) {
        return false;
    }

    // 显示进度
    m_progressBar->setVisible(true);
    m_progressBar->setValue(0);
    m_statusLabel->setText("正在加载点云文件...");
    
    // 处理事件以更新UI
    QApplication::processEvents();

    try {
        // 创建 PCL 点云对象
        core::PCLPointCloud::Ptr pclCloud(new core::PCLPointCloud);
        
        // 使用加载器加载点云数据
        m_progressBar->setValue(30);
        QApplication::processEvents();
        
        if (!core::PointCloudLoader::loadPointCloud(filename.toStdString(), pclCloud)) {
            m_progressBar->setVisible(false);
            m_statusLabel->setText("错误: 无法加载点云文件");
            
            QMessageBox::warning(this, "加载错误", 
                QString("无法加载点云文件:\n%1\n\n请检查文件路径和格式是否正确。").arg(filename));
            return false;
        }
        
        m_progressBar->setValue(60);
        QApplication::processEvents();
        
        // 创建点云封装对象
        m_currentPointCloud = std::make_shared<core::PointCloud>(pclCloud, filename.toStdString());
        m_currentFileName = filename;
        
        m_progressBar->setValue(80);
        QApplication::processEvents();
        
        // 设置点云到可视化器
        if (!m_visualizerWidget->setPointCloud(m_currentPointCloud)) {
            m_progressBar->setVisible(false);
            m_statusLabel->setText("错误: 无法设置点云到可视化器");
            
            QMessageBox::warning(this, "可视化错误", 
                QString("无法在可视化器中显示点云文件:\n%1").arg(filename));
            return false;
        }
        
        m_progressBar->setValue(100);
        QApplication::processEvents();
        
        // 更新结果面板的点云信息
        m_resultPanel->setPointCloudInfo(m_currentPointCloud);
        
        // 更新窗口标题
        QFileInfo fileInfo(filename);
        setWindowTitle(QString("PCL 点云可视化与坑洞检测系统 - %1").arg(fileInfo.fileName()));
        
        // 更新状态
        m_statusLabel->setText(QString("点云加载成功 - %1 个点").arg(m_currentPointCloud->size()));
        m_progressBar->setVisible(false);
        
        // 导入数据后自动开始深度分析
        QTimer::singleShot(100, [this]() {
            startAutomaticAnalysis();
        });
        
        // 发出信号
        emit pointCloudLoaded(m_currentPointCloud);
        
        // 记住上次打开的目录
        m_lastOpenDirectory = QFileInfo(filename).absolutePath();
        
        return true;
        
    } catch (const std::exception& e) {
        m_progressBar->setVisible(false);
        m_statusLabel->setText("错误: 点云加载异常");
        
        QMessageBox::critical(this, "加载异常", 
            QString("加载点云文件时发生异常:\n%1\n\n错误信息:\n%2").arg(filename).arg(e.what()));
        return false;
    } catch (...) {
        m_progressBar->setVisible(false);
        m_statusLabel->setText("错误: 未知异常");
        
        QMessageBox::critical(this, "未知异常", 
            QString("加载点云文件时发生未知异常:\n%1").arg(filename));
        return false;
    }
}

void MainWindow::onOpenFile() {
    // 设置文件对话框
    QString defaultDir = m_lastOpenDirectory;
    if (defaultDir.isEmpty()) {
        // 默认指向项目的data目录
        QString projectDataDir = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../../data");
        QDir dataDir(projectDataDir);
        if (dataDir.exists()) {
            defaultDir = dataDir.absolutePath();
        } else {
            // 如果data目录不存在，回退到Documents目录
            defaultDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
        }
    }
    
    QString filename = QFileDialog::getOpenFileName(
        this,
        "打开点云文件",
        defaultDir,
        "点云文件 (*.asc *.pcd *.ply *.txt);;ASCII 文件 (*.asc *.txt);;PCD 文件 (*.pcd);;PLY 文件 (*.ply);;所有文件 (*.*)"
    );
    
    if (!filename.isEmpty()) {
        loadPointCloud(filename);
    }
}

void MainWindow::onSaveScreenshot() {
    if (!m_currentPointCloud) {
        QMessageBox::information(this, "提示", "请先加载点云数据");
        return;
    }
    
    QString defaultDir = m_lastOpenDirectory;
    if (defaultDir.isEmpty()) {
        defaultDir = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);
    }
    
    QString defaultName = QString("screenshot_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    QString filename = QFileDialog::getSaveFileName(
        this,
        "保存截图",
        QDir(defaultDir).filePath(defaultName),
        "PNG 图像 (*.png);;JPEG 图像 (*.jpg *.jpeg);;BMP 图像 (*.bmp);;所有文件 (*.*)"
    );
    
    if (!filename.isEmpty()) {
        if (m_visualizerWidget->saveScreenshot(filename)) {
            m_statusLabel->setText(QString("截图已保存: %1").arg(QFileInfo(filename).fileName()));
            QMessageBox::information(this, "保存成功", QString("截图已保存到:\n%1").arg(filename));
        } else {
            QMessageBox::warning(this, "保存失败", QString("无法保存截图到:\n%1").arg(filename));
        }
    }
}

void MainWindow::onAbout() {
    QMessageBox::about(this, "关于",
        "<h3>PCL 点云可视化与坑洞检测系统 v1.0</h3>"
        "<p>基于 PCL (Point Cloud Library) 和 Qt6 开发的点云数据可视化和路面坑洞检测分析系统。</p>"
        "<p><b>主要功能：</b></p>"
        "<ul>"
        "<li>支持多种点云文件格式 (ASC, PCD, PLY)</li>"
        "<li>交互式3D点云可视化</li>"
        "<li>智能坑洞检测和测量</li>"
        "<li>实时参数调节</li>"
        "<li>详细测量结果展示</li>"
        "</ul>"
        "<p><b>技术栈：</b> C++17, Qt6, PCL, VTK</p>"
        "<p><b>开发环境：</b> CMake, Modern C++</p>"
        "<p>Copyright © 2024 PCL Visualization Project</p>"
    );
}

void MainWindow::onExit() {
    close();
}

void MainWindow::onResetCamera() {
    if (m_visualizerWidget) {
        m_visualizerWidget->resetCamera();
        m_statusLabel->setText("相机视角已重置");
    }
}

void MainWindow::onToggleResultPanel() {
    m_resultPanelVisible = !m_resultPanelVisible;
    m_resultPanel->setVisible(m_resultPanelVisible);
    m_toggleResultPanelAction->setText(m_resultPanelVisible ? "隐藏结果面板" : "显示结果面板");
}


void MainWindow::closeEvent(QCloseEvent *event) {
    // 确认退出
    int result = QMessageBox::question(this, "确认退出",
        "确定要退出程序吗？",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
        
    if (result == QMessageBox::Yes) {
        // 保存设置
        writeSettings();
        event->accept();
    } else {
        event->ignore();
    }
}

void MainWindow::updateStatusTime() {
    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    m_timeLabel->setText(currentTime);
}

void MainWindow::onPointCloudUpdated(std::shared_ptr<core::PointCloud> pointCloud) {
    m_currentPointCloud = pointCloud;
    if (pointCloud) {
        m_statusLabel->setText(QString("点云已更新 - %1 个点").arg(pointCloud->size()));
    }
}


void MainWindow::initializeUI() {
    // 创建中央窗口部件
    createCentralWidget();
    
    // 创建菜单栏
    createMenuBar();
    
    // 创建工具栏
    createToolBar();
    
    // 创建状态栏
    createStatusBar();
}

void MainWindow::createMenuBar() {
    // 文件菜单
    m_fileMenu = menuBar()->addMenu("文件(&F)");
    
    m_openAction = new QAction("打开(&O)...", this);
    m_openAction->setShortcut(QKeySequence::Open);
    m_openAction->setStatusTip("打开点云文件");
    m_fileMenu->addAction(m_openAction);
    
    m_fileMenu->addSeparator();
    
    m_saveScreenshotAction = new QAction("保存截图(&S)...", this);
    m_saveScreenshotAction->setShortcut(QKeySequence("Ctrl+S"));
    m_saveScreenshotAction->setStatusTip("保存当前视图截图");
    m_fileMenu->addAction(m_saveScreenshotAction);
    
    m_fileMenu->addSeparator();
    
    m_exitAction = new QAction("退出(&X)", this);
    m_exitAction->setShortcut(QKeySequence::Quit);
    m_exitAction->setStatusTip("退出程序");
    m_fileMenu->addAction(m_exitAction);
    
    // 视图菜单
    m_viewMenu = menuBar()->addMenu("视图(&V)");
    
    m_resetCameraAction = new QAction("重置相机(&R)", this);
    m_resetCameraAction->setShortcut(QKeySequence("Ctrl+R"));
    m_resetCameraAction->setStatusTip("重置相机到默认视角");
    m_viewMenu->addAction(m_resetCameraAction);
    
    m_viewMenu->addSeparator();
    
    m_toggleResultPanelAction = new QAction("隐藏结果面板(&T)", this);
    m_toggleResultPanelAction->setStatusTip("切换结果面板显示/隐藏");
    m_viewMenu->addAction(m_toggleResultPanelAction);
    
    
    // 视图菜单 - 添加主题选项
    m_viewMenu->addSeparator();
    
    // 创建主题子菜单
    QMenu* themeMenu = m_viewMenu->addMenu("主题(&T)");
    
    m_lightThemeAction = new QAction("浅色主题(&L)", this);
    m_lightThemeAction->setCheckable(true);
    m_lightThemeAction->setStatusTip("切换到浅色主题");
    themeMenu->addAction(m_lightThemeAction);
    
    m_darkThemeAction = new QAction("深色主题(&D)", this);
    m_darkThemeAction->setCheckable(true);
    m_darkThemeAction->setStatusTip("切换到深色主题");
    themeMenu->addAction(m_darkThemeAction);
    
    m_highContrastThemeAction = new QAction("高对比度主题(&H)", this);
    m_highContrastThemeAction->setCheckable(true);
    m_highContrastThemeAction->setStatusTip("切换到高对比度主题");
    themeMenu->addAction(m_highContrastThemeAction);
    
    // 创建主题按钮组
    m_themeActionGroup = new QActionGroup(this);
    m_themeActionGroup->addAction(m_lightThemeAction);
    m_themeActionGroup->addAction(m_darkThemeAction);
    m_themeActionGroup->addAction(m_highContrastThemeAction);
    m_lightThemeAction->setChecked(true); // 默认选中浅色主题
    
    themeMenu->addSeparator();
    
    // 添加字体大小选项
    QMenu* fontSizeMenu = themeMenu->addMenu("字体大小(&S)");
    
    m_smallFontAction = new QAction("小字体(&S)", this);
    m_smallFontAction->setCheckable(true);
    fontSizeMenu->addAction(m_smallFontAction);
    
    m_normalFontAction = new QAction("正常字体(&N)", this);
    m_normalFontAction->setCheckable(true);
    m_normalFontAction->setChecked(true);
    fontSizeMenu->addAction(m_normalFontAction);
    
    m_largeFontAction = new QAction("大字体(&L)", this);
    m_largeFontAction->setCheckable(true);
    fontSizeMenu->addAction(m_largeFontAction);
    
    m_extraLargeFontAction = new QAction("特大字体(&X)", this);
    m_extraLargeFontAction->setCheckable(true);
    fontSizeMenu->addAction(m_extraLargeFontAction);
    
    // 创建字体大小按钮组
    m_fontSizeActionGroup = new QActionGroup(this);
    m_fontSizeActionGroup->addAction(m_smallFontAction);
    m_fontSizeActionGroup->addAction(m_normalFontAction);
    m_fontSizeActionGroup->addAction(m_largeFontAction);
    m_fontSizeActionGroup->addAction(m_extraLargeFontAction);
    
    // 帮助菜单
    m_helpMenu = menuBar()->addMenu("帮助(&H)");
    
    m_aboutAction = new QAction("关于(&A)...", this);
    m_aboutAction->setStatusTip("显示程序信息");
    m_helpMenu->addAction(m_aboutAction);
}

void MainWindow::createToolBar() {
    m_mainToolBar = addToolBar("主工具栏");
    m_mainToolBar->setObjectName("MainToolBar");
    
    // 添加常用操作到工具栏
    m_mainToolBar->addAction(m_openAction);
    m_mainToolBar->addAction(m_saveScreenshotAction);
    m_mainToolBar->addSeparator();
    m_mainToolBar->addAction(m_resetCameraAction);
}

void MainWindow::createStatusBar() {
    // 创建状态栏组件
    m_statusLabel = new QLabel("就绪");
    m_timeLabel = new QLabel();
    m_progressBar = new QProgressBar();
    
    // 设置进度条属性
    m_progressBar->setVisible(false);
    m_progressBar->setMaximumWidth(200);
    
    // 添加到状态栏
    statusBar()->addWidget(m_statusLabel, 1); // 拉伸
    statusBar()->addPermanentWidget(m_progressBar);
    statusBar()->addPermanentWidget(m_timeLabel);
}

void MainWindow::createCentralWidget() {
    // 创建中央窗口部件
    m_centralWidget = new QWidget;
    setCentralWidget(m_centralWidget);
    
    // 创建主布局
    m_mainLayout = new QHBoxLayout(m_centralWidget);
    m_mainLayout->setContentsMargins(2, 2, 2, 2);
    m_mainLayout->setSpacing(2);
    
    // 创建水平分割器（左侧：可视化器，右侧：结果面板）
    m_horizontalSplitter = new QSplitter(Qt::Horizontal);
    m_mainLayout->addWidget(m_horizontalSplitter);
    
    // 创建可视化器
    m_visualizerWidget = new VisualizerWidget();
    
    // 创建结果面板
    m_resultPanel = new ResultPanel();
    
    // 设置二栏布局 - 左侧3D视图，右侧结果面板
    m_horizontalSplitter->addWidget(m_visualizerWidget);
    m_horizontalSplitter->addWidget(m_resultPanel);
    
    // 设置分割器比例
    m_horizontalSplitter->setStretchFactor(0, 3);  // 可视化器占3/4
    m_horizontalSplitter->setStretchFactor(1, 1);  // 结果面板占1/4
    
    // 设置最小和最大宽度
    m_visualizerWidget->setMinimumWidth(600);
    m_resultPanel->setMinimumWidth(300);
    m_resultPanel->setMaximumWidth(400);  // 限制结果面板最大宽度，保证3D视图有足够空间
}

void MainWindow::connectSignalsAndSlots() {
    // 菜单动作连接
    connect(m_openAction, &QAction::triggered, this, &MainWindow::onOpenFile);
    connect(m_saveScreenshotAction, &QAction::triggered, this, &MainWindow::onSaveScreenshot);
    connect(m_exitAction, &QAction::triggered, this, &MainWindow::onExit);
    connect(m_resetCameraAction, &QAction::triggered, this, &MainWindow::onResetCamera);
    connect(m_toggleResultPanelAction, &QAction::triggered, this, &MainWindow::onToggleResultPanel);
    connect(m_aboutAction, &QAction::triggered, this, &MainWindow::onAbout);
    
    // 创建内部分析控制Actions（不显示在菜单中）
    m_startAnalysisAction = new QAction(this);
    m_stopAnalysisAction = new QAction(this);
    m_analysisSettingsAction = new QAction(this);
    
    // 分析相关信号连接
    connect(m_startAnalysisAction, &QAction::triggered, this, &MainWindow::onStartAnalysis);
    connect(m_stopAnalysisAction, &QAction::triggered, this, &MainWindow::onStopAnalysis);
    connect(m_analysisSettingsAction, &QAction::triggered, this, &MainWindow::onAnalysisSettings);
    
    // 主题相关信号连接
    connect(m_lightThemeAction, &QAction::triggered, this, &MainWindow::onLightTheme);
    connect(m_darkThemeAction, &QAction::triggered, this, &MainWindow::onDarkTheme);
    connect(m_highContrastThemeAction, &QAction::triggered, this, &MainWindow::onHighContrastTheme);
    
    // 字体大小相关信号连接
    connect(m_smallFontAction, &QAction::triggered, this, &MainWindow::onSmallFont);
    connect(m_normalFontAction, &QAction::triggered, this, &MainWindow::onNormalFont);
    connect(m_largeFontAction, &QAction::triggered, this, &MainWindow::onLargeFont);
    connect(m_extraLargeFontAction, &QAction::triggered, this, &MainWindow::onExtraLargeFont);
    
    // 可视化器信号连接
    connect(m_visualizerWidget, &VisualizerWidget::pointCloudSet, 
            this, [this](bool success) {
                if (success) {
                    m_statusLabel->setText("点云设置成功");
                } else {
                    m_statusLabel->setText("点云设置失败");
                }
            });
    
    
    // 结果面板信号连接
    connect(m_resultPanel, &ResultPanel::analysisRequested,
            this, [this]() {
                m_statusLabel->setText("请求重新分析");
                // TODO: 触发重新分析
            });
}

void MainWindow::applyTheme() {
    // 获取主题管理器实例
    ThemeManager* themeManager = ThemeManager::instance();
    
    // 应用主窗口特定样式
    QString additionalStyles = themeManager->getMainWindowStyleSheet();
    if (!additionalStyles.isEmpty()) {
        setStyleSheet(styleSheet() + additionalStyles);
    }
    
    // 连接主题变化信号
    connect(themeManager, &ThemeManager::themeChanged, 
            this, &MainWindow::onThemeChanged);
    connect(themeManager, &ThemeManager::fontSizeChanged,
            this, &MainWindow::onFontSizeChanged);
    
    // 为所有子控件应用主题
    themeManager->applyThemeToWidget(this);
    
    // 添加主题切换菜单项
    addThemeMenuItems();
}

void MainWindow::readSettings() {
    QSettings settings("PCL_Visualization", "MainWindow");
    
    // 恢复窗口几何
    const QByteArray geometry = settings.value("geometry").toByteArray();
    if (!geometry.isEmpty()) {
        restoreGeometry(geometry);
    }
    
    // 恢复窗口状态
    const QByteArray windowState = settings.value("windowState").toByteArray();
    if (!windowState.isEmpty()) {
        restoreState(windowState);
    }
    
    // 恢复分割器状态
    const QByteArray horizontalSplitterState = settings.value("horizontalSplitter").toByteArray();
    if (!horizontalSplitterState.isEmpty()) {
        m_horizontalSplitter->restoreState(horizontalSplitterState);
    }
    
    // 恢复面板可见性
    m_resultPanelVisible = settings.value("resultPanelVisible", true).toBool();
    
    m_resultPanel->setVisible(m_resultPanelVisible);
    
    m_toggleResultPanelAction->setText(m_resultPanelVisible ? "隐藏结果面板" : "显示结果面板");
    
    // 恢复上次打开的目录
    m_lastOpenDirectory = settings.value("lastOpenDirectory").toString();
}

void MainWindow::writeSettings() {
    QSettings settings("PCL_Visualization", "MainWindow");
    
    // 保存窗口几何和状态
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    
    // 保存分割器状态
    settings.setValue("horizontalSplitter", m_horizontalSplitter->saveState());
    
    // 保存面板可见性
    settings.setValue("resultPanelVisible", m_resultPanelVisible);
    
    // 保存上次打开的目录
    settings.setValue("lastOpenDirectory", m_lastOpenDirectory);
}

void MainWindow::addThemeMenuItems() {
    // 这个函数的实现已经在createMenuBar()中完成
    // 这里可以添加额外的主题相关初始化代码
}

void MainWindow::onThemeChanged() {
    // 主题改变时刷新整个窗口
    update();
    
    // 确保所有子控件都应用新主题
    ThemeManager* themeManager = ThemeManager::instance();
    themeManager->applyThemeToWidget(this);
    
    m_statusLabel->setText("主题已更新");
}

void MainWindow::onFontSizeChanged() {
    // 字体大小改变时刷新整个窗口
    update();
    
    m_statusLabel->setText("字体大小已更新");
}

void MainWindow::onLightTheme() {
    ThemeManager::instance()->setTheme(ThemeManager::Theme::Light);
}

void MainWindow::onDarkTheme() {
    ThemeManager::instance()->setTheme(ThemeManager::Theme::Dark);
}

void MainWindow::onHighContrastTheme() {
    ThemeManager::instance()->setTheme(ThemeManager::Theme::HighContrast);
}

void MainWindow::onSmallFont() {
    ThemeManager::instance()->setFontSize(ThemeManager::FontSize::Small);
}

void MainWindow::onNormalFont() {
    ThemeManager::instance()->setFontSize(ThemeManager::FontSize::Normal);
}

void MainWindow::onLargeFont() {
    ThemeManager::instance()->setFontSize(ThemeManager::FontSize::Large);
}

void MainWindow::onExtraLargeFont() {
    ThemeManager::instance()->setFontSize(ThemeManager::FontSize::ExtraLarge);
}

void MainWindow::setupAnalysisEngine() {
    // 设置分析进度回调
    m_potholeDetector->setProgressCallback(
        [this](const std::string& stage, int progress, const std::string& message) {
            QString qStage = QString::fromStdString(stage);
            QString qMessage = QString::fromStdString(message);
            emit analysisProgressUpdated(qStage, progress, qMessage);
        }
    );
    
    // 使用优化的深度分析参数
    m_analysisParams = analysis::createCentralMaxPotholeParams();
    m_potholeDetector->setAnalysisParams(m_analysisParams);
}

void MainWindow::startAutomaticAnalysis() {
    if (!m_currentPointCloud) {
        m_statusLabel->setText("错误：未加载点云数据");
        return;
    }
    
    if (m_analysisInProgress) {
        return; // 避免重复分析
    }
    
    m_statusLabel->setText("正在进行智能深度分析...");
    m_progressBar->setVisible(true);
    m_progressBar->setValue(0);
    
    // 直接调用分析函数
    onStartAnalysis();
}

void MainWindow::onStartAnalysis() {
    if (!m_currentPointCloud) {
        QMessageBox::warning(this, "分析错误", "请先加载点云文件");
        return;
    }
    
    if (m_analysisInProgress) {
        QMessageBox::information(this, "分析状态", "分析正在进行中，请等待完成");
        return;
    }
    
    // ! 调试日志：开始分析
    qDebug() << "[DEBUG] 开始分析 - 点云大小:" << m_currentPointCloud->size();
    qDebug() << "[DEBUG] 分析参数 - Z阈值百分位:" << m_analysisParams.zThresholdPercentile
             << ", 最小凹坑面积:" << m_analysisParams.minPotholeArea
             << ", 最小凹坑点数:" << m_analysisParams.minPotholePoints;
    
    m_analysisInProgress = true;
    m_startAnalysisAction->setEnabled(false);
    m_stopAnalysisAction->setEnabled(true);
    m_progressBar->setVisible(true);
    m_progressBar->setValue(0);
    m_statusLabel->setText("开始凹坑检测分析...");
    
    // 异步启动分析
    m_analysisFuture = m_potholeDetector->analyzeAsync(m_currentPointCloud);
    
    // 创建定时器检查分析结果
    QTimer* checkTimer = new QTimer(this);
    connect(checkTimer, &QTimer::timeout, [this, checkTimer]() {
        if (m_analysisFuture.valid()) {
            auto status = m_analysisFuture.wait_for(std::chrono::milliseconds(0));
            if (status == std::future_status::ready) {
                try {
                    auto result = m_analysisFuture.get();
                    // ! 调试日志：获取分析结果
                    qDebug() << "[DEBUG] 分析完成 - 成功:" << result.analysisSuccessful
                             << ", 凹坑数量:" << result.potholes.size()
                             << ", 有效凹坑:" << result.validPotholeCount;
                    onAnalysisFinished(result);
                } catch (const std::exception& e) {
                    // ! 调试日志：分析异常
                    qDebug() << "[ERROR] 分析异常:" << e.what();
                    QMessageBox::critical(this, "分析错误", 
                        QString("分析过程中发生异常: %1").arg(e.what()));
                    m_analysisInProgress = false;
                    m_startAnalysisAction->setEnabled(true);
                    m_stopAnalysisAction->setEnabled(false);
                    m_progressBar->setVisible(false);
                }
                checkTimer->deleteLater();
            }
        }
    });
    checkTimer->start(500); // 每500ms检查一次
}

void MainWindow::onStopAnalysis() {
    if (!m_analysisInProgress) {
        return;
    }
    
    m_potholeDetector->cancelAnalysis();
    m_analysisInProgress = false;
    m_startAnalysisAction->setEnabled(true);
    m_stopAnalysisAction->setEnabled(false);
    m_progressBar->setVisible(false);
    m_statusLabel->setText("分析已被用户取消");
}

void MainWindow::onAnalysisSettings() {
    // TODO: 实现分析参数设置对话框
    QMessageBox::information(this, "分析设置", "分析参数设置对话框尚未实现\n当前使用默认参数");
}

void MainWindow::onAnalysisResultUpdated(const analysis::AnalysisResult& result) {
    m_lastAnalysisResult = result;
    
    // 更新结果面板
    if (m_resultPanel) {
        // 创建简化的结果用于结果面板
        ui::PotholeResult panelResult;
        if (!result.potholes.empty()) {
            const auto& firstPothole = result.potholes[0];
            // * 修正：analysis模块现在直接输出mm单位，不需要转换
            panelResult.volume = firstPothole.volume;       // 已经是mm³
            panelResult.area = firstPothole.area;           // 已经是mm²
            panelResult.width = firstPothole.width;         // 已经是mm
            panelResult.length = firstPothole.length;       // 已经是mm
            panelResult.maxDepth = firstPothole.maxDepth;   // 已经是mm
            panelResult.pointCount = firstPothole.pointCount;
            panelResult.isValid = result.analysisSuccessful;
        } else {
            panelResult.isValid = false;
        }
        m_resultPanel->updateResults(panelResult);
    }
    
    emit analysisCompleted(result);
}

void MainWindow::onAnalysisProgressUpdated(const QString& stage, int progress, const QString& message) {
    m_progressBar->setValue(progress);
    m_statusLabel->setText(QString("[%1] %2").arg(stage).arg(message));
}

void MainWindow::onAnalysisFinished(const analysis::AnalysisResult& result) {
    m_analysisInProgress = false;
    m_startAnalysisAction->setEnabled(true);
    m_stopAnalysisAction->setEnabled(false);
    m_progressBar->setVisible(false);
    
    if (result.analysisSuccessful) {
        QString message = QString("分析完成 - 检测到 %1 个凹坑，总体积 %2 m³")
                         .arg(result.validPotholeCount)
                         .arg(result.totalPotholeVolume, 0, 'e', 3);
        m_statusLabel->setText(message);
        
        // 更新结果显示
        onAnalysisResultUpdated(result);
        
        // 在3D视图中高亮显示凹坑（如果支持的话）
        // TODO: 实现凹坑可视化高亮
        
    } else {
        QString errorMsg = result.errorMessage.empty() ? "未知错误" : QString::fromStdString(result.errorMessage);
        m_statusLabel->setText(QString("分析失败: %1").arg(errorMsg));
        QMessageBox::warning(this, "分析失败", errorMsg);
    }
}

} // namespace ui
} // namespace pcl_viz

// Qt MOC 包含
// MOC 文件将由CMake自动生成和包含