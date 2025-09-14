#include "ResultPanel.h"
#include "ThemeManager.h"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtCore/QDateTime>
#include <QtCore/QStandardPaths>
#include <QtCore/QDir>
#include <QtCore/QDebug>
#include <QtCore/QTextStream>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtGui/QClipboard>
#include <QtGui/QFont>

namespace pcl_viz {
namespace ui {

ResultPanel::ResultPanel(QWidget *parent)
    : QWidget(parent)
    , m_mainLayout(nullptr)
    , m_measurementGroup(nullptr)
    , m_volumeEdit(nullptr)
    , m_areaEdit(nullptr)
    , m_widthEdit(nullptr)
    , m_lengthEdit(nullptr)
    , m_maxDepthEdit(nullptr)
    , m_pointCloudGroup(nullptr)
    , m_fileNameLabel(nullptr)
    , m_pointCountLabel(nullptr)
    , m_boundsLabel(nullptr)
    , m_statusGroup(nullptr)
    , m_analysisStatusLabel(nullptr)
    , m_analysisProgressBar(nullptr)
    , m_timestampLabel(nullptr)
    , m_exportButton(nullptr)
    , m_copyButton(nullptr)
    , m_refreshButton(nullptr)
    , m_resetButton(nullptr)
    , m_currentPointCloud(nullptr)
    , m_timeUpdateTimer(nullptr)
    , m_analyzing(false)
    , m_resultsValid(false)
{
    setObjectName("ResultPanel");
    setMinimumWidth(320);
    setMaximumWidth(400);
    
    // 初始化UI
    initializeUI();
    
    // 设置样式
    setupStyles();
    
    // 启动时间更新定时器
    m_timeUpdateTimer = new QTimer(this);
    connect(m_timeUpdateTimer, &QTimer::timeout, this, &ResultPanel::updateTimeDisplay);
    m_timeUpdateTimer->start(1000); // 每秒更新一次
    
    // 初始状态
    clearResults();
}

ResultPanel::~ResultPanel() = default;

void ResultPanel::updateResults(const PotholeResult& result) {
    
    m_currentResult = result;
    
    // 更新所有结果显示
    updateResultDisplay(m_volumeEdit, result.volume, "mm³");
    updateResultDisplay(m_areaEdit, result.area, "mm²");
    updateResultDisplay(m_widthEdit, result.width, "mm");
    updateResultDisplay(m_lengthEdit, result.length, "mm");
    updateResultDisplay(m_maxDepthEdit, result.maxDepth, "mm", 5);
    
    // 更新有效性
    setResultsValid(result.isValid);
    
    // 更新时间戳
    updateTimeDisplay();
}

void ResultPanel::clearResults() {
    // 清空所有显示
    m_volumeEdit->clear();
    m_areaEdit->clear();
    m_widthEdit->clear();
    m_lengthEdit->clear();
    m_maxDepthEdit->clear();
    
    // 重置结果对象
    m_currentResult = PotholeResult();
    
    // 设置为无效状态
    setResultsValid(false);
    
    // 更新状态
    m_analysisStatusLabel->setText("未进行分析");
    m_analysisProgressBar->setValue(0);
}

void ResultPanel::setPointCloudInfo(std::shared_ptr<core::PointCloud> pointCloud) {
    m_currentPointCloud = pointCloud;
    
    if (!pointCloud) {
        m_fileNameLabel->setText("文件名: 未加载");
        m_pointCountLabel->setText("点数量: 0");
        m_boundsLabel->setText("边界: 无");
        return;
    }
    
    // 更新文件名
    QString fileName = QString::fromStdString(pointCloud->getFilename());
    QFileInfo fileInfo(fileName);
    m_fileNameLabel->setText(QString("文件名: %1").arg(fileInfo.fileName()));
    
    // 更新点数量
    m_pointCountLabel->setText(QString("点数量: %1").arg(pointCloud->size()));
    
    // 更新边界信息
    auto bounds = pointCloud->getBounds();
    QString boundsText = QString("X: [%.2f, %.2f]\nY: [%.2f, %.2f]\nZ: [%.2f, %.2f]")
        .arg(bounds.minPoint.x).arg(bounds.maxPoint.x)
        .arg(bounds.minPoint.y).arg(bounds.maxPoint.y)
        .arg(bounds.minPoint.z).arg(bounds.maxPoint.z);
    m_boundsLabel->setText(boundsText);
}

void ResultPanel::setAnalysisStatus(bool analyzing, int progress) {
    m_analyzing = analyzing;
    
    if (analyzing) {
        m_analysisStatusLabel->setText("正在分析中...");
        m_analysisProgressBar->setValue(progress);
        m_analysisProgressBar->setVisible(true);
        
        // 禁用操作按钮
        m_exportButton->setEnabled(false);
        m_refreshButton->setEnabled(false);
    } else {
        m_analysisStatusLabel->setText("分析完成");
        m_analysisProgressBar->setVisible(false);
        
        // 启用操作按钮
        m_exportButton->setEnabled(m_resultsValid);
        m_refreshButton->setEnabled(true);
    }
}

void ResultPanel::setResultsValid(bool valid) {
    m_resultsValid = valid;
    
    // 设置控件启用状态
    m_exportButton->setEnabled(valid && !m_analyzing);
    m_copyButton->setEnabled(valid);
    
    // 设置样式表以指示有效性
    QString styleClass = valid ? "valid-result" : "invalid-result";
    m_measurementGroup->setProperty("class", styleClass);
    m_measurementGroup->style()->unpolish(m_measurementGroup);
    m_measurementGroup->style()->polish(m_measurementGroup);
}

void ResultPanel::onExportResults() {
    if (!m_resultsValid || !m_currentPointCloud) {
        QMessageBox::information(this, "提示", "没有有效的结果可以导出");
        return;
    }
    
    QString defaultDir = m_lastExportPath;
    if (defaultDir.isEmpty()) {
        defaultDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }
    
    QString defaultName = QString("pothole_result_%1.json")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    QString filename = QFileDialog::getSaveFileName(
        this,
        "导出分析结果",
        QDir(defaultDir).filePath(defaultName),
        "JSON 文件 (*.json);;CSV 文件 (*.csv);;文本文件 (*.txt);;所有文件 (*.*)"
    );
    
    if (filename.isEmpty()) {
        return;
    }
    
    bool success = false;
    QFileInfo fileInfo(filename);
    QString extension = fileInfo.suffix().toLower();
    
    if (extension == "json") {
        success = exportAsJson(filename);
    } else if (extension == "csv") {
        success = exportAsCsv(filename);
    } else {
        success = exportAsText(filename);
    }
    
    if (success) {
        m_lastExportPath = fileInfo.absolutePath();
        QMessageBox::information(this, "导出成功", 
            QString("结果已成功导出到:\n%1").arg(filename));
        emit resultsExported(filename, true);
    } else {
        QMessageBox::warning(this, "导出失败", 
            QString("无法导出结果到:\n%1").arg(filename));
        emit resultsExported(filename, false);
    }
}

void ResultPanel::onCopyToClipboard() {
    if (!m_resultsValid) {
        QMessageBox::information(this, "提示", "没有有效的结果可以复制");
        return;
    }
    
    // 构建文本内容
    QString content = QString(
        "PCL 坑洞检测分析结果\n"
        "生成时间: %1\n"
        "文件名: %2\n\n"
        "测量结果:\n"
        "最大深度: %3 mm\n"
        "体积: %4 mm³\n"
        "面积: %5 mm²\n"
        "宽度: %6 mm\n"
        "长度: %7 mm\n"
        "点云数量: %8"
    ).arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
     .arg(m_currentPointCloud ? QString::fromStdString(m_currentPointCloud->getFilename()) : "无")
     .arg(m_currentResult.maxDepth, 0, 'f', 2)
     .arg(m_currentResult.volume, 0, 'f', 2)
     .arg(m_currentResult.area, 0, 'f', 2)
     .arg(m_currentResult.width, 0, 'f', 2)
     .arg(m_currentResult.length, 0, 'f', 2)
     .arg(m_currentResult.pointCount);
    
    // 复制到剪贴板
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(content);
    
    QMessageBox::information(this, "复制成功", "结果已复制到剪贴板");
}

void ResultPanel::onRefreshDisplay() {
    if (m_currentPointCloud) {
        setPointCloudInfo(m_currentPointCloud);
    }
    updateTimeDisplay();
}

void ResultPanel::onResetResults() {
    int result = QMessageBox::question(this, "确认重置",
        "确定要重置所有结果吗？",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
        
    if (result == QMessageBox::Yes) {
        clearResults();
    }
}

void ResultPanel::updateTimeDisplay() {
    QString currentTime = QDateTime::currentDateTime().toString("hh:mm:ss");
    m_timestampLabel->setText(QString("更新时间: %1").arg(currentTime));
}

void ResultPanel::initializeUI() {
    m_mainLayout = new QVBoxLayout(this);
    m_mainLayout->setContentsMargins(8, 8, 8, 8);
    m_mainLayout->setSpacing(6);
    
    // 创建各个组
    m_measurementGroup = createMeasurementGroup();
    m_pointCloudGroup = createPointCloudInfoGroup();
    m_statusGroup = createAnalysisStatusGroup();
    
    // 添加到主布局
    m_mainLayout->addWidget(m_measurementGroup);
    m_mainLayout->addWidget(m_pointCloudGroup);
    m_mainLayout->addWidget(m_statusGroup);
    
    // 添加操作按钮
    QHBoxLayout* buttonLayout = createActionButtons();
    m_mainLayout->addLayout(buttonLayout);
    
    // 添加弹性空间
    m_mainLayout->addStretch();
}

QGroupBox* ResultPanel::createMeasurementGroup() {
    QGroupBox* group = new QGroupBox("测量结果");
    group->setObjectName("measurementGroup");
    
    QGridLayout* layout = new QGridLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    // 创建结果显示项
    createResultItem(layout, 0, "最大深度:", m_maxDepthEdit, "mm");
    createResultItem(layout, 1, "体积:", m_volumeEdit, "mm³");
    createResultItem(layout, 2, "面积:", m_areaEdit, "mm²");
    createResultItem(layout, 3, "宽度:", m_widthEdit, "mm");
    createResultItem(layout, 4, "长度:", m_lengthEdit, "mm");
    
    return group;
}

QGroupBox* ResultPanel::createPointCloudInfoGroup() {
    QGroupBox* group = new QGroupBox("点云信息");
    group->setObjectName("pointCloudGroup");
    
    QVBoxLayout* layout = new QVBoxLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    m_fileNameLabel = new QLabel("文件名: 未加载");
    m_fileNameLabel->setWordWrap(true);
    
    m_pointCountLabel = new QLabel("点数量: 0");
    
    m_boundsLabel = new QLabel("边界: 无");
    m_boundsLabel->setWordWrap(true);
    m_boundsLabel->setAlignment(Qt::AlignTop);
    
    // 设置字体
    QFont infoFont;
    infoFont.setPointSize(8);
    m_fileNameLabel->setFont(infoFont);
    m_pointCountLabel->setFont(infoFont);
    m_boundsLabel->setFont(infoFont);
    
    layout->addWidget(m_fileNameLabel);
    layout->addWidget(m_pointCountLabel);
    layout->addWidget(m_boundsLabel);
    
    return group;
}

QGroupBox* ResultPanel::createAnalysisStatusGroup() {
    QGroupBox* group = new QGroupBox("分析状态");
    group->setObjectName("statusGroup");
    
    QVBoxLayout* layout = new QVBoxLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    m_analysisStatusLabel = new QLabel("未进行分析");
    
    m_analysisProgressBar = new QProgressBar();
    m_analysisProgressBar->setVisible(false);
    m_analysisProgressBar->setMinimum(0);
    m_analysisProgressBar->setMaximum(100);
    
    m_timestampLabel = new QLabel("更新时间: --:--:--");
    
    // 设置字体
    QFont statusFont;
    statusFont.setPointSize(8);
    m_analysisStatusLabel->setFont(statusFont);
    m_timestampLabel->setFont(statusFont);
    
    layout->addWidget(m_analysisStatusLabel);
    layout->addWidget(m_analysisProgressBar);
    layout->addWidget(m_timestampLabel);
    
    return group;
}

QHBoxLayout* ResultPanel::createActionButtons() {
    QHBoxLayout* layout = new QHBoxLayout();
    layout->setSpacing(4);
    
    // 创建按钮
    m_exportButton = new QPushButton("导出");
    m_copyButton = new QPushButton("复制");
    m_refreshButton = new QPushButton("刷新");
    m_resetButton = new QPushButton("重置");
    
    // 设置按钮样式
    QString buttonStyle = 
        "QPushButton {"
        "    min-height: 24px;"
        "    padding: 2px 8px;"
        "    font-size: 11px;"
        "}"
        "QPushButton:disabled {"
        "    color: #888888;"
        "}";
    
    m_exportButton->setStyleSheet(buttonStyle);
    m_copyButton->setStyleSheet(buttonStyle);
    m_refreshButton->setStyleSheet(buttonStyle);
    m_resetButton->setStyleSheet(buttonStyle);
    
    // 设置工具提示
    m_exportButton->setToolTip("导出分析结果到文件");
    m_copyButton->setToolTip("复制结果到剪贴板");
    m_refreshButton->setToolTip("刷新显示");
    m_resetButton->setToolTip("重置所有结果");
    
    // 连接信号
    connect(m_exportButton, &QPushButton::clicked, this, &ResultPanel::onExportResults);
    connect(m_copyButton, &QPushButton::clicked, this, &ResultPanel::onCopyToClipboard);
    connect(m_refreshButton, &QPushButton::clicked, this, &ResultPanel::onRefreshDisplay);
    connect(m_resetButton, &QPushButton::clicked, this, &ResultPanel::onResetResults);
    
    // 初始状态
    m_exportButton->setEnabled(false);
    m_copyButton->setEnabled(false);
    
    layout->addWidget(m_exportButton);
    layout->addWidget(m_copyButton);
    layout->addWidget(m_refreshButton);
    layout->addWidget(m_resetButton);
    
    return layout;
}

QString ResultPanel::formatValueWithUnit(double value, const QString& unit, int precision) const {
    if (std::isnan(value) || std::isinf(value)) {
        return "N/A";
    }
    return QString("%1 %2").arg(value, 0, 'f', precision).arg(unit);
}

void ResultPanel::createResultItem(QGridLayout* layout, int row, const QString& labelText, 
                                  QLineEdit*& valueEdit, const QString& unit) {
    // 创建标签
    QLabel* label = new QLabel(labelText);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    // 创建值编辑框
    valueEdit = new QLineEdit();
    valueEdit->setReadOnly(true);
    valueEdit->setAlignment(Qt::AlignRight);
    valueEdit->setMaximumHeight(24);
    
    // 创建单位标签
    QLabel* unitLabel = nullptr;
    if (!unit.isEmpty()) {
        unitLabel = new QLabel(unit);
        unitLabel->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    }
    
    // 设置字体
    QFont valueFont;
    valueFont.setPointSize(9);
    valueFont.setBold(true);
    valueEdit->setFont(valueFont);
    
    QFont labelFont;
    labelFont.setPointSize(8);
    label->setFont(labelFont);
    if (unitLabel) {
        unitLabel->setFont(labelFont);
    }
    
    // 添加到布局
    layout->addWidget(label, row, 0);
    layout->addWidget(valueEdit, row, 1);
    if (unitLabel) {
        layout->addWidget(unitLabel, row, 2);
    }
    
    // 设置列比例
    layout->setColumnStretch(0, 2);  // 标签列
    layout->setColumnStretch(1, 3);  // 值列
    layout->setColumnStretch(2, 1);  // 单位列
}

void ResultPanel::updateResultDisplay(QLineEdit* edit, double value, const QString& unit, int precision) {
    Q_UNUSED(unit)  // 单位在布局中已设置，此处不使用
    if (!edit) {
        return;
    }
    
    if (std::isnan(value) || std::isinf(value) || value == 0.0) {
        edit->setText("--");
        edit->setStyleSheet("QLineEdit { color: #888888; }");
    } else {
        QString displayText = QString::number(value, 'f', precision);
        edit->setText(displayText);
        edit->setStyleSheet("QLineEdit { color: #000000; font-weight: bold; }");
    }
}

void ResultPanel::setupStyles() {
    // 使用主题管理器应用样式
    ThemeManager* themeManager = ThemeManager::instance();
    themeManager->applyThemeToWidget(this);
    
    // 应用组件特定的额外样式
    QString additionalStyles = themeManager->getResultPanelStyleSheet();
    if (!additionalStyles.isEmpty()) {
        setStyleSheet(styleSheet() + additionalStyles);
    }
    
    // 连接主题变化信号
    connect(themeManager, &ThemeManager::themeChanged, 
            this, [this]() {
                ThemeManager* tm = ThemeManager::instance();
                tm->applyThemeToWidget(this);
                QString additionalStyles = tm->getResultPanelStyleSheet();
                if (!additionalStyles.isEmpty()) {
                    setStyleSheet(styleSheet() + additionalStyles);
                }
            });
}

bool ResultPanel::exportAsJson(const QString& filename) {
    QJsonObject json;
    
    // 基本信息
    json["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate);
    json["filename"] = m_currentPointCloud ? QString::fromStdString(m_currentPointCloud->getFilename()) : "";
    json["pointCount"] = m_currentPointCloud ? static_cast<int>(m_currentPointCloud->size()) : 0;
    
    // 测量结果
    QJsonObject measurements;
    measurements["maxDepth_mm"] = m_currentResult.maxDepth;
    measurements["volume_mm3"] = m_currentResult.volume;
    measurements["area_mm2"] = m_currentResult.area;
    measurements["width_mm"] = m_currentResult.width;
    measurements["length_mm"] = m_currentResult.length;
    measurements["isValid"] = m_currentResult.isValid;
    json["measurements"] = measurements;
    
    // 写入文件
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        return false;
    }
    
    QJsonDocument doc(json);
    file.write(doc.toJson());
    return true;
}

bool ResultPanel::exportAsCsv(const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream stream(&file);
    
    // 写入表头
    stream << "Parameter,Value,Unit\n";
    
    // 写入数据
    stream << "Timestamp," << QDateTime::currentDateTime().toString(Qt::ISODate) << ",\n";
    stream << "Filename," << (m_currentPointCloud ? QString::fromStdString(m_currentPointCloud->getFilename()) : "") << ",\n";
    stream << "Point Count," << (m_currentPointCloud ? m_currentPointCloud->size() : 0) << ",\n";
    stream << "Max Depth," << QString::number(m_currentResult.maxDepth, 'f', 5) << ",mm\n";
    stream << "Volume," << m_currentResult.volume << ",mm³\n";
    stream << "Area," << m_currentResult.area << ",mm²\n";
    stream << "Width," << m_currentResult.width << ",mm\n";
    stream << "Length," << m_currentResult.length << ",mm\n";
    stream << "Valid," << (m_currentResult.isValid ? "true" : "false") << ",\n";
    
    return true;
}

bool ResultPanel::exportAsText(const QString& filename) {
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    
    QTextStream stream(&file);
    
    stream << "PCL 坑洞检测分析结果报告\n";
    stream << "================================\n\n";
    stream << "生成时间: " << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << "\n";
    stream << "文件名: " << (m_currentPointCloud ? QString::fromStdString(m_currentPointCloud->getFilename()) : "无") << "\n";
    stream << "点云数量: " << (m_currentPointCloud ? m_currentPointCloud->size() : 0) << "\n\n";
    
    stream << "测量结果:\n";
    stream << "---------\n";
    stream << "最大深度: " << QString::number(m_currentResult.maxDepth, 'f', 5) << " mm\n";
    stream << "体积: " << m_currentResult.volume << " mm³\n";
    stream << "面积: " << m_currentResult.area << " mm²\n";
    stream << "宽度: " << m_currentResult.width << " mm\n";
    stream << "长度: " << m_currentResult.length << " mm\n";
    stream << "结果有效性: " << (m_currentResult.isValid ? "有效" : "无效") << "\n\n";
    
    if (m_currentPointCloud) {
        auto bounds = m_currentPointCloud->getBounds();
        stream << "点云边界信息:\n";
        stream << "-------------\n";
        stream << QString("X: [%1, %2]\n").arg(bounds.minPoint.x).arg(bounds.maxPoint.x);
        stream << QString("Y: [%1, %2]\n").arg(bounds.minPoint.y).arg(bounds.maxPoint.y);
        stream << QString("Z: [%1, %2]\n").arg(bounds.minPoint.z).arg(bounds.maxPoint.z);
    }
    
    return true;
}

} // namespace ui
} // namespace pcl_viz

// Qt MOC 包含
// MOC 文件将由CMake自动生成和包含