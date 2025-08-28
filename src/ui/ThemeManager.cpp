#include "ThemeManager.h"

#include <QtWidgets/QWidget>
#include <QtCore/QSettings>
#include <QtCore/QStandardPaths>
#include <QtGui/QScreen>
#include <QtGui/QFontDatabase>
#include <QtWidgets/QStyleFactory>
#include <QtCore/QDebug>

namespace pcl_viz {
namespace ui {

ThemeManager* ThemeManager::s_instance = nullptr;

ThemeManager* ThemeManager::instance() {
    if (!s_instance) {
        s_instance = new ThemeManager();
    }
    return s_instance;
}

ThemeManager::ThemeManager(QObject* parent)
    : QObject(parent)
    , m_application(nullptr)
    , m_currentTheme(Theme::Light)
    , m_currentFontSize(FontSize::Normal)
    , m_dpiScaleFactor(1.0)
    , m_systemFontFamily("Arial")
    , m_monospaceFamily("Consolas")
{
    // 检测系统字体
    QFontDatabase fontDb;
    QStringList families = fontDb.families();
    
    // 寻找合适的系统字体
    if (families.contains("Microsoft YaHei UI")) {
        m_systemFontFamily = "Microsoft YaHei UI";
    } else if (families.contains("Microsoft YaHei")) {
        m_systemFontFamily = "Microsoft YaHei";
    } else if (families.contains("SimHei")) {
        m_systemFontFamily = "SimHei";
    } else if (families.contains("Arial")) {
        m_systemFontFamily = "Arial";
    } else {
        m_systemFontFamily = QApplication::font().family();
    }
    
    // 寻找合适的等宽字体
    if (families.contains("JetBrains Mono")) {
        m_monospaceFamily = "JetBrains Mono";
    } else if (families.contains("Fira Code")) {
        m_monospaceFamily = "Fira Code";
    } else if (families.contains("Consolas")) {
        m_monospaceFamily = "Consolas";
    } else if (families.contains("Courier New")) {
        m_monospaceFamily = "Courier New";
    } else {
        m_monospaceFamily = "monospace";
    }
    
    qDebug() << "ThemeManager: Selected system font:" << m_systemFontFamily;
    qDebug() << "ThemeManager: Selected monospace font:" << m_monospaceFamily;
}

ThemeManager::~ThemeManager() {
    saveThemeSettings();
}

void ThemeManager::initialize(QApplication* app) {
    m_application = app;
    
    // 设置DPI缩放
    setupDpiScaling();
    
    // 加载设置
    loadThemeSettings();
    
    // 初始化字体
    updateFonts();
    
    // 应用主题
    updateApplication();
    
    qDebug() << "ThemeManager: Initialized with DPI scale factor:" << m_dpiScaleFactor;
}

void ThemeManager::setTheme(Theme theme) {
    if (m_currentTheme != theme) {
        m_currentTheme = theme;
        updateApplication();
        emit themeChanged(theme);
    }
}

void ThemeManager::setFontSize(FontSize fontSize) {
    if (m_currentFontSize != fontSize) {
        m_currentFontSize = fontSize;
        updateFonts();
        updateApplication();
        emit fontSizeChanged(fontSize);
    }
}

void ThemeManager::setupDpiScaling() {
    if (!m_application) return;
    
    // 获取主屏幕DPI
    QScreen* screen = m_application->primaryScreen();
    if (screen) {
        double dpi = screen->logicalDotsPerInch();
        m_dpiScaleFactor = dpi / 96.0; // 96 DPI是标准分辨率
        
        // 限制缩放因子范围
        m_dpiScaleFactor = qBound(0.8, m_dpiScaleFactor, 3.0);
        
        qDebug() << "ThemeManager: Screen DPI:" << dpi << "Scale factor:" << m_dpiScaleFactor;
    }
}

void ThemeManager::updateFonts() {
    int baseFontSize = static_cast<int>(m_currentFontSize) * m_dpiScaleFactor;
    
    // 基础字体 - 用于一般文本
    m_baseFont = QFont(m_systemFontFamily, baseFontSize);
    m_baseFont.setStyleHint(QFont::SansSerif);
    m_baseFont.setWeight(QFont::Normal);
    
    // 标题字体 - 用于标题和组框
    m_titleFont = QFont(m_systemFontFamily, baseFontSize + 1);
    m_titleFont.setWeight(QFont::Bold);
    
    // 数据字体 - 用于数值显示
    m_dataFont = QFont(m_systemFontFamily, baseFontSize);
    m_dataFont.setWeight(QFont::Medium);
    
    // 代码字体 - 用于等宽文本
    m_codeFont = QFont(m_monospaceFamily, baseFontSize - 1);
    m_codeFont.setStyleHint(QFont::Monospace);
    
    qDebug() << "ThemeManager: Updated fonts with base size:" << baseFontSize;
}

void ThemeManager::updateApplication() {
    if (!m_application) return;
    
    // 设置应用程序字体
    m_application->setFont(m_baseFont);
    
    // 应用样式表
    QString styleSheet = getApplicationStyleSheet();
    m_application->setStyleSheet(styleSheet);
    
    qDebug() << "ThemeManager: Applied" << (m_currentTheme == Theme::Light ? "Light" : 
                                          m_currentTheme == Theme::Dark ? "Dark" : "HighContrast") << "theme";
}

QString ThemeManager::getApplicationStyleSheet() const {
    switch (m_currentTheme) {
        case Theme::Dark:
            return getDarkThemeStyleSheet();
        case Theme::HighContrast:
            return getHighContrastThemeStyleSheet();
        case Theme::Light:
        default:
            return getLightThemeStyleSheet();
    }
}

QString ThemeManager::getLightThemeStyleSheet() const {
    return QString(R"(
/* === 基础样式 === */
QWidget {
    background-color: #ffffff;
    color: #2c2c2c;
    font-family: "%1";
    font-size: %2px;
}

QMainWindow {
    background-color: #f5f5f5;
}

/* === 菜单栏样式 === */
QMenuBar {
    background-color: #ffffff;
    border-bottom: 1px solid #d0d0d0;
    color: #2c2c2c;
    padding: 2px;
}

QMenuBar::item {
    background-color: transparent;
    padding: 6px 12px;
    margin: 0px;
    color: #2c2c2c;
    font-weight: 500;
}

QMenuBar::item:selected {
    background-color: #e3f2fd;
    color: #1976d2;
}

QMenuBar::item:pressed {
    background-color: #bbdefb;
}

/* === 工具栏样式 === */
QToolBar {
    background-color: #ffffff;
    border: 1px solid #d0d0d0;
    color: #2c2c2c;
    spacing: 3px;
    padding: 2px;
}

QToolButton {
    background-color: transparent;
    border: 1px solid transparent;
    border-radius: 3px;
    padding: 4px;
    margin: 1px;
    color: #2c2c2c;
}

QToolButton:hover {
    background-color: #e3f2fd;
    border-color: #90caf9;
}

QToolButton:pressed {
    background-color: #bbdefb;
    border-color: #64b5f6;
}

/* === 状态栏样式 === */
QStatusBar {
    background-color: #ffffff;
    border-top: 1px solid #d0d0d0;
    color: #2c2c2c;
    font-size: %3px;
}

QStatusBar QLabel {
    color: #2c2c2c;
    padding: 2px 4px;
}

/* === 分割器样式 === */
QSplitter::handle {
    background-color: #d0d0d0;
}

QSplitter::handle:horizontal {
    width: 4px;
    background-color: #bdbdbd;
}

QSplitter::handle:vertical {
    height: 4px;
    background-color: #bdbdbd;
}

QSplitter::handle:hover {
    background-color: #1976d2;
}

/* === 组框样式 === */
QGroupBox {
    font-weight: bold;
    border: 2px solid #c0c0c0;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 8px;
    background-color: #ffffff;
    color: #2c2c2c;
    font-size: %4px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px 0 8px;
    color: #1976d2;
    font-weight: bold;
}

/* === 输入控件样式 === */
QLineEdit {
    border: 2px solid #e0e0e0;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #ffffff;
    color: #2c2c2c;
    font-size: %5px;
    selection-background-color: #1976d2;
    selection-color: #ffffff;
}

QLineEdit:focus {
    border-color: #1976d2;
}

QLineEdit:read-only {
    background-color: #f8f9fa;
    color: #495057;
    border-color: #dee2e6;
}

QSpinBox, QDoubleSpinBox {
    border: 2px solid #e0e0e0;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #ffffff;
    color: #2c2c2c;
    font-size: %6px;
    min-height: 22px;
}

QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #1976d2;
}

QSpinBox::up-button, QDoubleSpinBox::up-button {
    background-color: #f8f9fa;
    border-left: 1px solid #e0e0e0;
    border-bottom: 1px solid #e0e0e0;
    border-top-right-radius: 3px;
}

QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #f8f9fa;
    border-left: 1px solid #e0e0e0;
    border-bottom-right-radius: 3px;
}

QComboBox {
    border: 2px solid #e0e0e0;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #ffffff;
    color: #2c2c2c;
    font-size: %7px;
    min-height: 22px;
}

QComboBox:focus {
    border-color: #1976d2;
}

QComboBox::drop-down {
    border: none;
    background-color: #f8f9fa;
    width: 20px;
}

QComboBox::down-arrow {
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 6px solid #666666;
}

/* === 按钮样式 === */
QPushButton {
    background-color: #ffffff;
    border: 2px solid #1976d2;
    border-radius: 4px;
    padding: 6px 16px;
    color: #1976d2;
    font-size: %8px;
    font-weight: 500;
    min-height: 24px;
}

QPushButton:hover {
    background-color: #e3f2fd;
    border-color: #1565c0;
    color: #1565c0;
}

QPushButton:pressed {
    background-color: #bbdefb;
    border-color: #0d47a1;
    color: #0d47a1;
}

QPushButton:disabled {
    background-color: #f5f5f5;
    border-color: #bdbdbd;
    color: #9e9e9e;
}

/* 特殊按钮样式 */
QPushButton[class="success"] {
    background-color: #4caf50;
    border-color: #4caf50;
    color: #ffffff;
}

QPushButton[class="success"]:hover {
    background-color: #43a047;
    border-color: #43a047;
}

QPushButton[class="danger"] {
    background-color: #f44336;
    border-color: #f44336;
    color: #ffffff;
}

QPushButton[class="danger"]:hover {
    background-color: #e53935;
    border-color: #e53935;
}

/* === 复选框样式 === */
QCheckBox {
    color: #2c2c2c;
    font-size: %9px;
    spacing: 8px;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 2px solid #1976d2;
    border-radius: 3px;
    background-color: #ffffff;
}

QCheckBox::indicator:checked {
    background-color: #1976d2;
    image: none;
}

QCheckBox::indicator:hover {
    border-color: #1565c0;
}

/* === 进度条样式 === */
QProgressBar {
    border: 2px solid #e0e0e0;
    border-radius: 4px;
    background-color: #ffffff;
    color: #2c2c2c;
    text-align: center;
    font-size: %10px;
    font-weight: bold;
}

QProgressBar::chunk {
    background-color: #1976d2;
    border-radius: 2px;
}

/* === 标签样式 === */
QLabel {
    color: #2c2c2c;
    font-size: %11px;
}

QLabel[class="title"] {
    font-size: %12px;
    font-weight: bold;
    color: #1976d2;
}

QLabel[class="data"] {
    font-size: %13px;
    font-weight: 500;
    color: #2c2c2c;
}

QLabel[class="info"] {
    font-size: %14px;
    color: #666666;
}

/* === 结果面板特殊样式 === */
QGroupBox#measurementGroup[class="valid-result"] {
    border-color: #4caf50;
    background-color: #f8fff8;
}

QGroupBox#measurementGroup[class="valid-result"]::title {
    color: #2e7d32;
}

QGroupBox#measurementGroup[class="invalid-result"] {
    border-color: #f44336;
    background-color: #fff8f8;
}

QGroupBox#measurementGroup[class="invalid-result"]::title {
    color: #c62828;
}

/* === 工具提示样式 === */
QToolTip {
    background-color: #424242;
    color: #ffffff;
    border: 1px solid #616161;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: %15px;
}
)")
    .arg(m_systemFontFamily)
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 基础字体大小
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 状态栏字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 组框标题
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 输入框字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 数字输入框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 下拉框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 按钮字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 复选框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 进度条
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 2)  // 标题标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 数据标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 信息标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1); // 工具提示
}

QString ThemeManager::getDarkThemeStyleSheet() const {
    return QString(R"(
/* === 深色主题基础样式 === */
QWidget {
    background-color: #2b2b2b;
    color: #ffffff;
    font-family: "%1";
    font-size: %2px;
}

QMainWindow {
    background-color: #1e1e1e;
}

/* === 菜单栏样式 === */
QMenuBar {
    background-color: #3c3c3c;
    border-bottom: 1px solid #555555;
    color: #ffffff;
    padding: 2px;
}

QMenuBar::item {
    background-color: transparent;
    padding: 6px 12px;
    margin: 0px;
    color: #ffffff;
    font-weight: 500;
}

QMenuBar::item:selected {
    background-color: #404040;
    color: #64b5f6;
}

QMenuBar::item:pressed {
    background-color: #505050;
}

/* === 工具栏样式 === */
QToolBar {
    background-color: #3c3c3c;
    border: 1px solid #555555;
    color: #ffffff;
    spacing: 3px;
    padding: 2px;
}

QToolButton {
    background-color: transparent;
    border: 1px solid transparent;
    border-radius: 3px;
    padding: 4px;
    margin: 1px;
    color: #ffffff;
}

QToolButton:hover {
    background-color: #404040;
    border-color: #666666;
}

QToolButton:pressed {
    background-color: #505050;
    border-color: #777777;
}

/* === 状态栏样式 === */
QStatusBar {
    background-color: #3c3c3c;
    border-top: 1px solid #555555;
    color: #ffffff;
    font-size: %3px;
}

QStatusBar QLabel {
    color: #ffffff;
    padding: 2px 4px;
}

/* === 分割器样式 === */
QSplitter::handle {
    background-color: #555555;
}

QSplitter::handle:horizontal {
    width: 4px;
    background-color: #666666;
}

QSplitter::handle:vertical {
    height: 4px;
    background-color: #666666;
}

QSplitter::handle:hover {
    background-color: #64b5f6;
}

/* === 组框样式 === */
QGroupBox {
    font-weight: bold;
    border: 2px solid #555555;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 8px;
    background-color: #2b2b2b;
    color: #ffffff;
    font-size: %4px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px 0 8px;
    color: #64b5f6;
    font-weight: bold;
}

/* === 输入控件样式 === */
QLineEdit {
    border: 2px solid #555555;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #3c3c3c;
    color: #ffffff;
    font-size: %5px;
    selection-background-color: #64b5f6;
    selection-color: #000000;
}

QLineEdit:focus {
    border-color: #64b5f6;
}

QLineEdit:read-only {
    background-color: #404040;
    color: #cccccc;
    border-color: #666666;
}

QSpinBox, QDoubleSpinBox {
    border: 2px solid #555555;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #3c3c3c;
    color: #ffffff;
    font-size: %6px;
    min-height: 22px;
}

QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #64b5f6;
}

QComboBox {
    border: 2px solid #555555;
    border-radius: 4px;
    padding: 4px 8px;
    background-color: #3c3c3c;
    color: #ffffff;
    font-size: %7px;
    min-height: 22px;
}

QComboBox:focus {
    border-color: #64b5f6;
}

/* === 按钮样式 === */
QPushButton {
    background-color: #3c3c3c;
    border: 2px solid #64b5f6;
    border-radius: 4px;
    padding: 6px 16px;
    color: #64b5f6;
    font-size: %8px;
    font-weight: 500;
    min-height: 24px;
}

QPushButton:hover {
    background-color: #404040;
    border-color: #42a5f5;
    color: #42a5f5;
}

QPushButton:pressed {
    background-color: #505050;
    border-color: #2196f3;
    color: #2196f3;
}

QPushButton:disabled {
    background-color: #2b2b2b;
    border-color: #666666;
    color: #888888;
}

/* 特殊按钮样式 */
QPushButton[class="success"] {
    background-color: #4caf50;
    border-color: #4caf50;
    color: #ffffff;
}

QPushButton[class="success"]:hover {
    background-color: #43a047;
    border-color: #43a047;
}

QPushButton[class="danger"] {
    background-color: #f44336;
    border-color: #f44336;
    color: #ffffff;
}

QPushButton[class="danger"]:hover {
    background-color: #e53935;
    border-color: #e53935;
}

/* === 复选框样式 === */
QCheckBox {
    color: #ffffff;
    font-size: %9px;
    spacing: 8px;
}

QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 2px solid #64b5f6;
    border-radius: 3px;
    background-color: #3c3c3c;
}

QCheckBox::indicator:checked {
    background-color: #64b5f6;
}

QCheckBox::indicator:hover {
    border-color: #42a5f5;
}

/* === 进度条样式 === */
QProgressBar {
    border: 2px solid #555555;
    border-radius: 4px;
    background-color: #3c3c3c;
    color: #ffffff;
    text-align: center;
    font-size: %10px;
    font-weight: bold;
}

QProgressBar::chunk {
    background-color: #64b5f6;
    border-radius: 2px;
}

/* === 标签样式 === */
QLabel {
    color: #ffffff;
    font-size: %11px;
}

QLabel[class="title"] {
    font-size: %12px;
    font-weight: bold;
    color: #64b5f6;
}

QLabel[class="data"] {
    font-size: %13px;
    font-weight: 500;
    color: #ffffff;
}

QLabel[class="info"] {
    font-size: %14px;
    color: #cccccc;
}

/* === 结果面板特殊样式 === */
QGroupBox#measurementGroup[class="valid-result"] {
    border-color: #4caf50;
    background-color: #1b3d1b;
}

QGroupBox#measurementGroup[class="valid-result"]::title {
    color: #81c784;
}

QGroupBox#measurementGroup[class="invalid-result"] {
    border-color: #f44336;
    background-color: #3d1b1b;
}

QGroupBox#measurementGroup[class="invalid-result"]::title {
    color: #e57373;
}

/* === 工具提示样式 === */
QToolTip {
    background-color: #616161;
    color: #ffffff;
    border: 1px solid #757575;
    border-radius: 4px;
    padding: 4px 8px;
    font-size: %15px;
}
)")
    .arg(m_systemFontFamily)
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 基础字体大小
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 状态栏字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 组框标题
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 输入框字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 数字输入框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 下拉框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 按钮字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 复选框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 进度条
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 2)  // 标题标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 数据标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1)  // 信息标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor - 1); // 工具提示
}

QString ThemeManager::getHighContrastThemeStyleSheet() const {
    return QString(R"(
/* === 高对比度主题基础样式 === */
QWidget {
    background-color: #000000;
    color: #ffffff;
    font-family: "%1";
    font-size: %2px;
}

QMainWindow {
    background-color: #000000;
}

/* === 菜单栏样式 === */
QMenuBar {
    background-color: #000000;
    border-bottom: 2px solid #ffffff;
    color: #ffffff;
    padding: 2px;
}

QMenuBar::item {
    background-color: transparent;
    padding: 6px 12px;
    margin: 0px;
    color: #ffffff;
    font-weight: bold;
}

QMenuBar::item:selected {
    background-color: #ffffff;
    color: #000000;
}

QMenuBar::item:pressed {
    background-color: #cccccc;
    color: #000000;
}

/* === 工具栏样式 === */
QToolBar {
    background-color: #000000;
    border: 2px solid #ffffff;
    color: #ffffff;
    spacing: 3px;
    padding: 2px;
}

QToolButton {
    background-color: transparent;
    border: 2px solid #ffffff;
    border-radius: 3px;
    padding: 4px;
    margin: 1px;
    color: #ffffff;
    font-weight: bold;
}

QToolButton:hover {
    background-color: #ffffff;
    color: #000000;
}

QToolButton:pressed {
    background-color: #cccccc;
    color: #000000;
}

/* === 状态栏样式 === */
QStatusBar {
    background-color: #000000;
    border-top: 2px solid #ffffff;
    color: #ffffff;
    font-size: %3px;
    font-weight: bold;
}

QStatusBar QLabel {
    color: #ffffff;
    padding: 2px 4px;
    font-weight: bold;
}

/* === 分割器样式 === */
QSplitter::handle {
    background-color: #ffffff;
}

QSplitter::handle:horizontal {
    width: 6px;
    background-color: #ffffff;
}

QSplitter::handle:vertical {
    height: 6px;
    background-color: #ffffff;
}

/* === 组框样式 === */
QGroupBox {
    font-weight: bold;
    border: 3px solid #ffffff;
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 8px;
    background-color: #000000;
    color: #ffffff;
    font-size: %4px;
}

QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 8px 0 8px;
    color: #ffffff;
    font-weight: bold;
    background-color: #000000;
}

/* === 输入控件样式 === */
QLineEdit {
    border: 3px solid #ffffff;
    border-radius: 4px;
    padding: 6px 10px;
    background-color: #000000;
    color: #ffffff;
    font-size: %5px;
    font-weight: bold;
    selection-background-color: #ffffff;
    selection-color: #000000;
}

QLineEdit:focus {
    border-color: #ffff00;
    background-color: #333333;
}

QLineEdit:read-only {
    background-color: #333333;
    color: #ffffff;
    border-color: #ffffff;
}

QSpinBox, QDoubleSpinBox {
    border: 3px solid #ffffff;
    border-radius: 4px;
    padding: 6px 10px;
    background-color: #000000;
    color: #ffffff;
    font-size: %6px;
    font-weight: bold;
    min-height: 26px;
}

QSpinBox:focus, QDoubleSpinBox:focus {
    border-color: #ffff00;
    background-color: #333333;
}

QComboBox {
    border: 3px solid #ffffff;
    border-radius: 4px;
    padding: 6px 10px;
    background-color: #000000;
    color: #ffffff;
    font-size: %7px;
    font-weight: bold;
    min-height: 26px;
}

QComboBox:focus {
    border-color: #ffff00;
    background-color: #333333;
}

/* === 按钮样式 === */
QPushButton {
    background-color: #000000;
    border: 3px solid #ffffff;
    border-radius: 4px;
    padding: 8px 18px;
    color: #ffffff;
    font-size: %8px;
    font-weight: bold;
    min-height: 28px;
}

QPushButton:hover {
    background-color: #ffffff;
    color: #000000;
}

QPushButton:pressed {
    background-color: #cccccc;
    color: #000000;
}

QPushButton:disabled {
    background-color: #333333;
    border-color: #666666;
    color: #666666;
}

/* 特殊按钮样式 */
QPushButton[class="success"] {
    background-color: #00ff00;
    border-color: #00ff00;
    color: #000000;
    font-weight: bold;
}

QPushButton[class="success"]:hover {
    background-color: #ccffcc;
    border-color: #00ff00;
    color: #000000;
}

QPushButton[class="danger"] {
    background-color: #ff0000;
    border-color: #ff0000;
    color: #ffffff;
    font-weight: bold;
}

QPushButton[class="danger"]:hover {
    background-color: #ffcccc;
    border-color: #ff0000;
    color: #000000;
}

/* === 复选框样式 === */
QCheckBox {
    color: #ffffff;
    font-size: %9px;
    font-weight: bold;
    spacing: 10px;
}

QCheckBox::indicator {
    width: 20px;
    height: 20px;
    border: 3px solid #ffffff;
    border-radius: 3px;
    background-color: #000000;
}

QCheckBox::indicator:checked {
    background-color: #ffffff;
}

QCheckBox::indicator:hover {
    border-color: #ffff00;
}

/* === 进度条样式 === */
QProgressBar {
    border: 3px solid #ffffff;
    border-radius: 4px;
    background-color: #000000;
    color: #ffffff;
    text-align: center;
    font-size: %10px;
    font-weight: bold;
}

QProgressBar::chunk {
    background-color: #ffffff;
    border-radius: 2px;
}

/* === 标签样式 === */
QLabel {
    color: #ffffff;
    font-size: %11px;
    font-weight: bold;
}

QLabel[class="title"] {
    font-size: %12px;
    font-weight: bold;
    color: #ffffff;
}

QLabel[class="data"] {
    font-size: %13px;
    font-weight: bold;
    color: #ffffff;
}

QLabel[class="info"] {
    font-size: %14px;
    font-weight: bold;
    color: #ffffff;
}

/* === 结果面板特殊样式 === */
QGroupBox#measurementGroup[class="valid-result"] {
    border-color: #00ff00;
    background-color: #003300;
}

QGroupBox#measurementGroup[class="valid-result"]::title {
    color: #00ff00;
}

QGroupBox#measurementGroup[class="invalid-result"] {
    border-color: #ff0000;
    background-color: #330000;
}

QGroupBox#measurementGroup[class="invalid-result"]::title {
    color: #ff0000;
}

/* === 工具提示样式 === */
QToolTip {
    background-color: #ffffff;
    color: #000000;
    border: 2px solid #000000;
    border-radius: 4px;
    padding: 6px 10px;
    font-size: %15px;
    font-weight: bold;
}
)")
    .arg(m_systemFontFamily)
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 基础字体大小（高对比度使用稍大字体）
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 状态栏字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 2)  // 组框标题
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 输入框字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 数字输入框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 下拉框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 按钮字体
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 复选框
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor)      // 进度条
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 3)  // 标题标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 2)  // 数据标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor + 1)  // 信息标签
    .arg(static_cast<int>(m_currentFontSize) * m_dpiScaleFactor);     // 工具提示
}

QFont ThemeManager::getBaseFont() const {
    return m_baseFont;
}

QFont ThemeManager::getTitleFont() const {
    return m_titleFont;
}

QFont ThemeManager::getDataFont() const {
    return m_dataFont;
}

QFont ThemeManager::getCodeFont() const {
    return m_codeFont;
}

void ThemeManager::applyThemeToWidget(QWidget* widget) {
    if (!widget) return;
    
    // 应用基础字体
    widget->setFont(m_baseFont);
    
    // 如果需要，可以为特定类型的widget应用特殊样式
    // 这里可以根据widget的类型进行定制
}

void ThemeManager::loadThemeSettings() {
    QSettings settings("PCL_Visualization", "Theme");
    
    int themeValue = settings.value("theme", static_cast<int>(Theme::Light)).toInt();
    m_currentTheme = static_cast<Theme>(themeValue);
    
    int fontSizeValue = settings.value("fontSize", static_cast<int>(FontSize::Normal)).toInt();
    m_currentFontSize = static_cast<FontSize>(fontSizeValue);
}

void ThemeManager::saveThemeSettings() {
    QSettings settings("PCL_Visualization", "Theme");
    
    settings.setValue("theme", static_cast<int>(m_currentTheme));
    settings.setValue("fontSize", static_cast<int>(m_currentFontSize));
}

QString ThemeManager::getMainWindowStyleSheet() const {
    // 主窗口特定的额外样式可以在这里添加
    return QString();
}

QString ThemeManager::getResultPanelStyleSheet() const {
    // 结果面板特定的额外样式可以在这里添加
    return QString();
}


QString ThemeManager::getVisualizerStyleSheet() const {
    // 可视化器特定的额外样式可以在这里添加
    return QString();
}

} // namespace ui  
} // namespace pcl_viz

#include "ThemeManager.moc"