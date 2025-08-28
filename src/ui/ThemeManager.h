#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtGui/QFont>
#include <QtWidgets/QApplication>

namespace pcl_viz {
namespace ui {

/**
 * @brief 主题管理器类，负责统一管理应用程序的UI主题和样式
 * 
 * 解决文字可见性问题，提供DPI感知和字体优化功能
 */
class ThemeManager : public QObject {
    Q_OBJECT

public:
    /**
     * @brief 主题类型枚举
     */
    enum class Theme {
        Light,      // 浅色主题（默认）
        Dark,       // 深色主题
        HighContrast // 高对比度主题
    };

    /**
     * @brief 字体大小级别
     */
    enum class FontSize {
        Small = 8,   // 小字体
        Normal = 10, // 正常字体
        Large = 12,  // 大字体
        ExtraLarge = 14 // 特大字体
    };

    /**
     * @brief 获取单例实例
     */
    static ThemeManager* instance();

    /**
     * @brief 析构函数
     */
    ~ThemeManager() override;

    /**
     * @brief 初始化主题管理器
     * @param app Qt应用程序实例
     */
    void initialize(QApplication* app);

    /**
     * @brief 设置当前主题
     * @param theme 主题类型
     */
    void setTheme(Theme theme);

    /**
     * @brief 获取当前主题
     * @return 当前主题类型
     */
    Theme currentTheme() const { return m_currentTheme; }

    /**
     * @brief 设置字体大小
     * @param fontSize 字体大小级别
     */
    void setFontSize(FontSize fontSize);

    /**
     * @brief 获取当前字体大小
     * @return 当前字体大小级别
     */
    FontSize currentFontSize() const { return m_currentFontSize; }

    /**
     * @brief 获取应用程序样式表
     * @return 完整的应用程序样式表字符串
     */
    QString getApplicationStyleSheet() const;

    /**
     * @brief 获取主窗口样式表
     * @return 主窗口样式表
     */
    QString getMainWindowStyleSheet() const;

    /**
     * @brief 获取结果面板样式表
     * @return 结果面板样式表
     */
    QString getResultPanelStyleSheet() const;


    /**
     * @brief 获取可视化器样式表
     * @return 可视化器样式表
     */
    QString getVisualizerStyleSheet() const;

    /**
     * @brief 获取基础字体
     * @return 基础字体对象
     */
    QFont getBaseFont() const;

    /**
     * @brief 获取标题字体
     * @return 标题字体对象
     */
    QFont getTitleFont() const;

    /**
     * @brief 获取数据字体
     * @return 数据显示字体对象
     */
    QFont getDataFont() const;

    /**
     * @brief 获取代码字体
     * @return 等宽代码字体对象
     */
    QFont getCodeFont() const;

    /**
     * @brief 自动检测和设置DPI缩放
     */
    void setupDpiScaling();

    /**
     * @brief 获取当前DPI缩放因子
     * @return DPI缩放因子
     */
    double getDpiScaleFactor() const { return m_dpiScaleFactor; }

    /**
     * @brief 应用主题到特定窗口部件
     * @param widget 目标窗口部件
     */
    void applyThemeToWidget(QWidget* widget);

signals:
    /**
     * @brief 主题改变信号
     * @param theme 新的主题类型
     */
    void themeChanged(Theme theme);

    /**
     * @brief 字体大小改变信号
     * @param fontSize 新的字体大小级别
     */
    void fontSizeChanged(FontSize fontSize);

private:
    explicit ThemeManager(QObject* parent = nullptr);

    // * 私有成员函数
    void loadThemeSettings();
    void saveThemeSettings();
    void updateFonts();
    void updateApplication();
    
    QString getLightThemeStyleSheet() const;
    QString getDarkThemeStyleSheet() const;
    QString getHighContrastThemeStyleSheet() const;
    
    QString getBaseColors() const;
    QString getCommonStyles() const;
    QString getFontStyles() const;

    // * 私有成员变量
    static ThemeManager* s_instance;
    
    QApplication* m_application;
    Theme m_currentTheme;
    FontSize m_currentFontSize;
    double m_dpiScaleFactor;
    
    QFont m_baseFont;
    QFont m_titleFont;
    QFont m_dataFont;
    QFont m_codeFont;
    
    QString m_systemFontFamily;
    QString m_monospaceFamily;
};

} // namespace ui
} // namespace pcl_viz