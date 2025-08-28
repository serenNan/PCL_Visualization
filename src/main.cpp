#include <iostream>
#include <memory>
#include <string>

// Qt 头文件
#include <QApplication>
#include <QDir>
#include <QStandardPaths>
#include <QTranslator>
#include <QLocale>
#include <QTimer>
#include <QThread>
#include <QMessageBox>
#include <QSplashScreen>
#include <QPixmap>

// 项目头文件
#include "ui/MainWindow.h"
#include "ui/ThemeManager.h"
#include "core/PointCloudLoader.h"
#include "core/PointCloud.h"

/**
 * @brief 显示程序使用方法
 * @param programName 程序名称
 */
void showUsage(const std::string& programName) {
    std::cout << "PCL 点云可视化与坑洞检测系统" << std::endl;
    std::cout << "使用方法: " << programName << " [选项] [点云文件路径]" << std::endl;
    std::cout << "\n选项:" << std::endl;
    std::cout << "  --help, -h     显示帮助信息" << std::endl;
    std::cout << "  --version, -v  显示版本信息" << std::endl;
    std::cout << "\n示例:" << std::endl;
    std::cout << "  " << programName << " ../data/H103v2.asc" << std::endl;
    std::cout << "  " << programName << "  # 启动GUI界面" << std::endl;
    std::cout << "\n支持的文件格式:" << std::endl;
    std::cout << "  - .asc (Geomagic Studio ASCII format)" << std::endl;
    std::cout << "  - .pcd (PCL Point Cloud Data)" << std::endl;
    std::cout << "  - .ply (Polygon File Format)" << std::endl;
    std::cout << "  - .txt (Text format)" << std::endl;
}

/**
 * @brief 显示版本信息
 */
void showVersion() {
    std::cout << "PCL 点云可视化与坑洞检测系统 v1.0.0" << std::endl;
    std::cout << "基于 PCL (Point Cloud Library) 和 Qt6 开发" << std::endl;
    std::cout << "Copyright (c) 2024 PCL Visualization Project" << std::endl;
}

/**
 * @brief 设置应用程序属性和样式
 * @param app Qt应用程序对象
 */
void setupApplication(QApplication& app) {
    // 设置应用程序属性
    app.setApplicationName("PCL点云可视化系统");
    app.setApplicationDisplayName("PCL 点云可视化与坑洞检测系统");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("PCL Visualization Project");
    app.setOrganizationDomain("pcl-visualization.org");
    
    // 设置应用程序样式
    app.setStyle("Fusion");  // 使用现代化的Fusion风格
    
    // 初始化主题管理器
    pcl_viz::ui::ThemeManager* themeManager = pcl_viz::ui::ThemeManager::instance();
    themeManager->initialize(&app);
    
    std::cout << "主题管理器初始化完成，DPI缩放因子: " << themeManager->getDpiScaleFactor() << std::endl;
}

/**
 * @brief 显示启动画面
 * @return 启动画面对象指针
 */
std::unique_ptr<QSplashScreen> showSplashScreen() {
    // TODO: 创建实际的启动画面图片
    QPixmap splashPixmap(400, 300);
    splashPixmap.fill(QColor(70, 130, 180)); // Steel Blue背景
    
    auto splash = std::make_unique<QSplashScreen>(splashPixmap);
    splash->setWindowFlags(Qt::WindowStaysOnTopHint | Qt::SplashScreen);
    
    // 添加文本信息
    splash->showMessage("PCL 点云可视化与坑洞检测系统\n正在初始化...", 
                       Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    splash->show();
    
    return splash;
}

int main(int argc, char** argv) {
    using namespace pcl_viz;
    
    // 创建Qt应用程序
    QApplication app(argc, argv);
    
    // 解析命令行参数
    QString commandLineFile;
    
    for (int i = 1; i < argc; ++i) {
        QString arg = QString::fromLocal8Bit(argv[i]);
        if (arg == "--help" || arg == "-h") {
            showUsage(argv[0]);
            return 0;
        } else if (arg == "--version" || arg == "-v") {
            showVersion();
            return 0;
        } else if (!arg.startsWith("-")) {
            // 这是文件路径
            commandLineFile = arg;
        }
    }
    
    // 设置应用程序属性和样式
    setupApplication(app);
    
    // 显示启动画面
    auto splash = showSplashScreen();
    app.processEvents();
    
    std::cout << "=== PCL 点云可视化与坑洞检测系统 ===" << std::endl;
    std::cout << "正在启动GUI界面..." << std::endl;
    
    try {
        // 等待一小段时间让启动画面显示
        QThread::msleep(1000);
        splash->showMessage("正在初始化主窗口...", 
                           Qt::AlignBottom | Qt::AlignCenter, Qt::white);
        app.processEvents();
        
        // 创建主窗口
        ui::MainWindow mainWindow;
        
        // 确保主题应用到主窗口
        ui::ThemeManager::instance()->applyThemeToWidget(&mainWindow);
        
        // 关闭启动画面
        splash->finish(&mainWindow);
        splash.reset();
        
        // 如果指定了命令行文件，则加载它
        if (!commandLineFile.isEmpty()) {
            std::cout << "正在加载命令行指定的文件: " << commandLineFile.toStdString() << std::endl;
            
            // 延迟加载以确保窗口完全显示
            QTimer::singleShot(500, [&mainWindow, commandLineFile]() {
                if (!mainWindow.loadPointCloud(commandLineFile)) {
                    QMessageBox::warning(&mainWindow, "加载失败", 
                        QString("无法加载指定的点云文件:\n%1\n\n"
                               "请检查文件路径和格式是否正确。").arg(commandLineFile));
                }
            });
        }
        
        // 显示主窗口
        mainWindow.show();
        
        std::cout << "GUI界面启动成功" << std::endl;
        std::cout << "提示: 使用 文件->打开 来加载点云数据" << std::endl;
        
        // 运行Qt事件循环
        int result = app.exec();
        
        std::cout << "程序正常退出" << std::endl;
        return result;
        
    } catch (const std::exception& e) {
        // 关闭启动画面
        splash.reset();
        
        std::cerr << "程序异常: " << e.what() << std::endl;
        
        QMessageBox::critical(nullptr, "程序异常", 
            QString("程序运行时发生异常:\n%1\n\n程序将退出。").arg(e.what()));
        
        return -1;
    } catch (...) {
        // 关闭启动画面
        splash.reset();
        
        std::cerr << "未知异常发生" << std::endl;
        
        QMessageBox::critical(nullptr, "未知异常", 
            "程序运行时发生未知异常。\n\n程序将退出。");
        
        return -1;
    }
}