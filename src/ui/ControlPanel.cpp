#include "ControlPanel.h"
#include "ThemeManager.h"

#include <QtWidgets/QApplication>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>
#include <QtCore/QStandardPaths>
#include <QtCore/QDir>
#include <QtCore/QSettings>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>
#include <QtGui/QFont>

namespace pcl_viz {
namespace ui {

ControlPanel::ControlPanel(QWidget *parent)
    : QWidget(parent)
    , m_mainLayout(nullptr)
    , m_visualizationGroup(nullptr)
    , m_pointSizeSpinBox(nullptr)
    , m_colorModeComboBox(nullptr)
    , m_darkBackgroundCheckBox(nullptr)
    , m_coordinateSystemCheckBox(nullptr)
    , m_preprocessingGroup(nullptr)
    , m_voxelSizeSpinBox(nullptr)
    , m_statisticalFilterCheckBox(nullptr)
    , m_filterKSpinBox(nullptr)
    , m_filterStddevSpinBox(nullptr)
    , m_detectionGroup(nullptr)
    , m_searchRadiusSpinBox(nullptr)
    , m_polynomialOrderSpinBox(nullptr)
    , m_depthThresholdSpinBox(nullptr)
    , m_areaThresholdSpinBox(nullptr)
    , m_volumeThresholdSpinBox(nullptr)
    , m_slopeAngleSpinBox(nullptr)
    , m_clusteringGroup(nullptr)
    , m_clusterToleranceSpinBox(nullptr)
    , m_minClusterSizeSpinBox(nullptr)
    , m_maxClusterSizeSpinBox(nullptr)
    , m_startAnalysisButton(nullptr)
    , m_stopAnalysisButton(nullptr)
    , m_resetButton(nullptr)
    , m_saveConfigButton(nullptr)
    , m_loadConfigButton(nullptr)
    , m_analysisRunning(false)
    , m_updatingControls(false)
    , m_parameterUpdateTimer(nullptr)
{
    setObjectName("ControlPanel");
    setMinimumWidth(320);
    setMaximumWidth(400);
    
    // 初始化配置
    m_vizConfig.pointSize = 2.0f;
    m_vizConfig.colorMode = visualization::VisualizationConfig::ColorMode::HEIGHT_Z;
    m_vizConfig.backgroundColor[0] = 0.0;
    m_vizConfig.backgroundColor[1] = 0.0;
    m_vizConfig.backgroundColor[2] = 0.0;
    m_vizConfig.showCoordinateSystem = true;
    m_vizConfig.coordinateSystemScale = 1.0f;
    
    // 初始化检测配置为默认值
    m_detectionConfig = PotholeDetectionConfig();
    
    // 创建参数更新延迟定时器
    m_parameterUpdateTimer = new QTimer(this);
    m_parameterUpdateTimer->setSingleShot(true);
    connect(m_parameterUpdateTimer, &QTimer::timeout, 
            this, &ControlPanel::onParameterChangeDelayed);
    
    // 初始化UI
    initializeUI();
    
    // 连接信号和槽
    connectSignalsAndSlots();
    
    // 设置样式
    setupStyles();
    
    // 更新控件值
    updateControlsWithoutSignals();
}

ControlPanel::~ControlPanel() = default;

visualization::VisualizationConfig ControlPanel::getVisualizationConfig() const {
    return m_vizConfig;
}

void ControlPanel::setVisualizationConfig(const visualization::VisualizationConfig& config) {
    m_vizConfig = config;
    updateControlsWithoutSignals();
}

PotholeDetectionConfig ControlPanel::getDetectionConfig() const {
    return m_detectionConfig;
}

void ControlPanel::setDetectionConfig(const PotholeDetectionConfig& config) {
    m_detectionConfig = config;
    updateControlsWithoutSignals();
}

void ControlPanel::resetToDefaults() {
    // 重置可视化配置
    m_vizConfig.pointSize = 2.0f;
    m_vizConfig.colorMode = visualization::VisualizationConfig::ColorMode::HEIGHT_Z;
    m_vizConfig.backgroundColor[0] = 0.0;
    m_vizConfig.backgroundColor[1] = 0.0;
    m_vizConfig.backgroundColor[2] = 0.0;
    m_vizConfig.showCoordinateSystem = true;
    m_vizConfig.coordinateSystemScale = 1.0f;
    
    // 重置检测配置
    m_detectionConfig = PotholeDetectionConfig();
    
    // 更新控件
    updateControlsWithoutSignals();
    
    // 发出信号
    emit visualizationConfigChanged(m_vizConfig);
    emit detectionConfigChanged(m_detectionConfig);
    emit parametersReset();
}

void ControlPanel::setControlsEnabled(bool enabled) {
    // 可视化控件
    m_pointSizeSpinBox->setEnabled(enabled);
    m_colorModeComboBox->setEnabled(enabled);
    m_darkBackgroundCheckBox->setEnabled(enabled);
    m_coordinateSystemCheckBox->setEnabled(enabled);
    
    // 检测参数控件
    m_voxelSizeSpinBox->setEnabled(enabled);
    m_statisticalFilterCheckBox->setEnabled(enabled);
    m_filterKSpinBox->setEnabled(enabled);
    m_filterStddevSpinBox->setEnabled(enabled);
    m_searchRadiusSpinBox->setEnabled(enabled);
    m_polynomialOrderSpinBox->setEnabled(enabled);
    m_depthThresholdSpinBox->setEnabled(enabled);
    m_areaThresholdSpinBox->setEnabled(enabled);
    m_volumeThresholdSpinBox->setEnabled(enabled);
    m_slopeAngleSpinBox->setEnabled(enabled);
    m_clusterToleranceSpinBox->setEnabled(enabled);
    m_minClusterSizeSpinBox->setEnabled(enabled);
    m_maxClusterSizeSpinBox->setEnabled(enabled);
    
    // 操作按钮
    m_startAnalysisButton->setEnabled(enabled && !m_analysisRunning);
    m_resetButton->setEnabled(enabled);
    m_saveConfigButton->setEnabled(enabled);
    m_loadConfigButton->setEnabled(enabled);
}

void ControlPanel::onStartAnalysis() {
    if (m_analysisRunning) {
        return;
    }
    
    m_analysisRunning = true;
    m_startAnalysisButton->setEnabled(false);
    m_stopAnalysisButton->setEnabled(true);
    
    emit analysisRequested(m_detectionConfig);
}

void ControlPanel::onStopAnalysis() {
    if (!m_analysisRunning) {
        return;
    }
    
    m_analysisRunning = false;
    m_startAnalysisButton->setEnabled(true);
    m_stopAnalysisButton->setEnabled(false);
    
    emit analysisStopRequested();
}

void ControlPanel::onResetParameters() {
    int result = QMessageBox::question(this, "确认重置",
        "确定要重置所有参数到默认值吗？",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
        
    if (result == QMessageBox::Yes) {
        resetToDefaults();
    }
}

void ControlPanel::onSaveConfig() {
    QString defaultDir = m_lastConfigPath;
    if (defaultDir.isEmpty()) {
        defaultDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }
    
    QString filename = QFileDialog::getSaveFileName(
        this,
        "保存配置文件",
        QDir(defaultDir).filePath("pcl_config.json"),
        "JSON 配置文件 (*.json);;所有文件 (*.*)"
    );
    
    if (filename.isEmpty()) {
        return;
    }
    
    // 创建配置JSON对象
    QJsonObject config;
    
    // 可视化配置
    QJsonObject visualization;
    visualization["pointSize"] = m_vizConfig.pointSize;
    visualization["colorMode"] = static_cast<int>(m_vizConfig.colorMode);
    visualization["backgroundR"] = m_vizConfig.backgroundColor[0];
    visualization["backgroundG"] = m_vizConfig.backgroundColor[1];
    visualization["backgroundB"] = m_vizConfig.backgroundColor[2];
    visualization["showCoordinateSystem"] = m_vizConfig.showCoordinateSystem;
    visualization["coordinateSystemScale"] = m_vizConfig.coordinateSystemScale;
    config["visualization"] = visualization;
    
    // 检测配置
    QJsonObject detection;
    detection["downsampleVoxelSize"] = m_detectionConfig.downsampleVoxelSize;
    detection["enableStatisticalFilter"] = m_detectionConfig.enableStatisticalFilter;
    detection["statisticalFilterK"] = m_detectionConfig.statisticalFilterK;
    detection["statisticalFilterStddev"] = m_detectionConfig.statisticalFilterStddev;
    detection["searchRadius"] = m_detectionConfig.searchRadius;
    detection["polynomialOrder"] = m_detectionConfig.polynomialOrder;
    detection["depthThreshold"] = m_detectionConfig.depthThreshold;
    detection["areaThreshold"] = m_detectionConfig.areaThreshold;
    detection["volumeThreshold"] = m_detectionConfig.volumeThreshold;
    detection["slopeAngle"] = m_detectionConfig.slopeAngle;
    detection["clusterTolerance"] = m_detectionConfig.clusterTolerance;
    detection["minClusterSize"] = m_detectionConfig.minClusterSize;
    detection["maxClusterSize"] = m_detectionConfig.maxClusterSize;
    config["detection"] = detection;
    
    // 写入文件
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly)) {
        QJsonDocument doc(config);
        file.write(doc.toJson());
        m_lastConfigPath = QFileInfo(filename).absolutePath();
        QMessageBox::information(this, "保存成功", 
            QString("配置已保存到:\n%1").arg(filename));
    } else {
        QMessageBox::warning(this, "保存失败", 
            QString("无法保存配置到:\n%1").arg(filename));
    }
}

void ControlPanel::onLoadConfig() {
    QString defaultDir = m_lastConfigPath;
    if (defaultDir.isEmpty()) {
        defaultDir = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    }
    
    QString filename = QFileDialog::getOpenFileName(
        this,
        "加载配置文件",
        defaultDir,
        "JSON 配置文件 (*.json);;所有文件 (*.*)"
    );
    
    if (filename.isEmpty()) {
        return;
    }
    
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, "加载失败", 
            QString("无法打开配置文件:\n%1").arg(filename));
        return;
    }
    
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &error);
    if (error.error != QJsonParseError::NoError) {
        QMessageBox::warning(this, "解析错误", 
            QString("配置文件格式错误:\n%1").arg(error.errorString()));
        return;
    }
    
    QJsonObject config = doc.object();
    
    // 读取可视化配置
    if (config.contains("visualization")) {
        QJsonObject visualization = config["visualization"].toObject();
        m_vizConfig.pointSize = visualization["pointSize"].toDouble(2.0);
        m_vizConfig.colorMode = static_cast<visualization::VisualizationConfig::ColorMode>(
            visualization["colorMode"].toInt(0));
        m_vizConfig.backgroundColor[0] = visualization["backgroundR"].toDouble(0.0);
        m_vizConfig.backgroundColor[1] = visualization["backgroundG"].toDouble(0.0);
        m_vizConfig.backgroundColor[2] = visualization["backgroundB"].toDouble(0.0);
        m_vizConfig.showCoordinateSystem = visualization["showCoordinateSystem"].toBool(true);
        m_vizConfig.coordinateSystemScale = visualization["coordinateSystemScale"].toDouble(1.0);
    }
    
    // 读取检测配置
    if (config.contains("detection")) {
        QJsonObject detection = config["detection"].toObject();
        m_detectionConfig.downsampleVoxelSize = detection["downsampleVoxelSize"].toDouble(1.0);
        m_detectionConfig.enableStatisticalFilter = detection["enableStatisticalFilter"].toBool(true);
        m_detectionConfig.statisticalFilterK = detection["statisticalFilterK"].toInt(50);
        m_detectionConfig.statisticalFilterStddev = detection["statisticalFilterStddev"].toDouble(1.0);
        m_detectionConfig.searchRadius = detection["searchRadius"].toDouble(5.0);
        m_detectionConfig.polynomialOrder = detection["polynomialOrder"].toInt(2);
        m_detectionConfig.depthThreshold = detection["depthThreshold"].toDouble(2.0);
        m_detectionConfig.areaThreshold = detection["areaThreshold"].toDouble(100.0);
        m_detectionConfig.volumeThreshold = detection["volumeThreshold"].toDouble(50.0);
        m_detectionConfig.slopeAngle = detection["slopeAngle"].toDouble(15.0);
        m_detectionConfig.clusterTolerance = detection["clusterTolerance"].toDouble(2.0);
        m_detectionConfig.minClusterSize = detection["minClusterSize"].toInt(10);
        m_detectionConfig.maxClusterSize = detection["maxClusterSize"].toInt(10000);
    }
    
    // 更新控件
    updateControlsWithoutSignals();
    
    // 发出信号
    emit visualizationConfigChanged(m_vizConfig);
    emit detectionConfigChanged(m_detectionConfig);
    
    m_lastConfigPath = QFileInfo(filename).absolutePath();
    QMessageBox::information(this, "加载成功", 
        QString("配置已从以下文件加载:\n%1").arg(filename));
}

void ControlPanel::onPointSizeChanged(double value) {
    if (m_updatingControls) return;
    
    m_vizConfig.pointSize = static_cast<float>(value);
    emit visualizationConfigChanged(m_vizConfig);
}

void ControlPanel::onColorModeChanged(int index) {
    if (m_updatingControls) return;
    
    m_vizConfig.colorMode = static_cast<visualization::VisualizationConfig::ColorMode>(index);
    emit visualizationConfigChanged(m_vizConfig);
}

void ControlPanel::onBackgroundColorChanged(bool checked) {
    if (m_updatingControls) return;
    
    if (checked) {
        // 暗色背景
        m_vizConfig.backgroundColor[0] = 0.2;
        m_vizConfig.backgroundColor[1] = 0.2;
        m_vizConfig.backgroundColor[2] = 0.2;
    } else {
        // 黑色背景
        m_vizConfig.backgroundColor[0] = 0.0;
        m_vizConfig.backgroundColor[1] = 0.0;
        m_vizConfig.backgroundColor[2] = 0.0;
    }
    
    emit visualizationConfigChanged(m_vizConfig);
}

void ControlPanel::onCoordinateSystemToggled(bool checked) {
    if (m_updatingControls) return;
    
    m_vizConfig.showCoordinateSystem = checked;
    emit visualizationConfigChanged(m_vizConfig);
}

void ControlPanel::onDetectionParameterChanged() {
    if (m_updatingControls) return;
    
    // 更新检测配置
    m_detectionConfig.downsampleVoxelSize = m_voxelSizeSpinBox->value();
    m_detectionConfig.enableStatisticalFilter = m_statisticalFilterCheckBox->isChecked();
    m_detectionConfig.statisticalFilterK = m_filterKSpinBox->value();
    m_detectionConfig.statisticalFilterStddev = m_filterStddevSpinBox->value();
    m_detectionConfig.searchRadius = m_searchRadiusSpinBox->value();
    m_detectionConfig.polynomialOrder = m_polynomialOrderSpinBox->value();
    m_detectionConfig.depthThreshold = m_depthThresholdSpinBox->value();
    m_detectionConfig.areaThreshold = m_areaThresholdSpinBox->value();
    m_detectionConfig.volumeThreshold = m_volumeThresholdSpinBox->value();
    m_detectionConfig.slopeAngle = m_slopeAngleSpinBox->value();
    m_detectionConfig.clusterTolerance = m_clusterToleranceSpinBox->value();
    m_detectionConfig.minClusterSize = m_minClusterSizeSpinBox->value();
    m_detectionConfig.maxClusterSize = m_maxClusterSizeSpinBox->value();
    
    // 延迟发送信号以避免过多的更新
    m_parameterUpdateTimer->start(500); // 500ms延迟
}

void ControlPanel::onParameterChangeDelayed() {
    emit detectionConfigChanged(m_detectionConfig);
}

void ControlPanel::initializeUI() {
    m_mainLayout = new QVBoxLayout(this);
    m_mainLayout->setContentsMargins(8, 8, 8, 8);
    m_mainLayout->setSpacing(6);
    
    // 创建各个组
    m_visualizationGroup = createVisualizationGroup();
    m_preprocessingGroup = createPreprocessingGroup();
    m_detectionGroup = createDetectionGroup();
    m_clusteringGroup = createClusteringGroup();
    
    // 添加到主布局
    m_mainLayout->addWidget(m_visualizationGroup);
    m_mainLayout->addWidget(m_preprocessingGroup);
    m_mainLayout->addWidget(m_detectionGroup);
    m_mainLayout->addWidget(m_clusteringGroup);
    
    // 添加操作按钮
    QVBoxLayout* buttonLayout = createActionButtons();
    m_mainLayout->addLayout(buttonLayout);
    
    // 添加弹性空间
    m_mainLayout->addStretch();
}

QGroupBox* ControlPanel::createVisualizationGroup() {
    QGroupBox* group = new QGroupBox("可视化设置");
    group->setObjectName("visualizationGroup");
    
    QGridLayout* layout = new QGridLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    int row = 0;
    
    // 点大小
    QLabel* pointSizeLabel = new QLabel("点大小:");
    m_pointSizeSpinBox = new QDoubleSpinBox();
    m_pointSizeSpinBox->setRange(0.1, 10.0);
    m_pointSizeSpinBox->setDecimals(1);
    m_pointSizeSpinBox->setSingleStep(0.1);
    m_pointSizeSpinBox->setValue(2.0);
    layout->addWidget(pointSizeLabel, row, 0);
    layout->addWidget(m_pointSizeSpinBox, row, 1);
    row++;
    
    // 颜色模式
    QLabel* colorModeLabel = new QLabel("颜色模式:");
    m_colorModeComboBox = new QComboBox();
    m_colorModeComboBox->addItems({"高度着色", "统一颜色", "法向量X", "法向量Y", "法向量Z"});
    layout->addWidget(colorModeLabel, row, 0);
    layout->addWidget(m_colorModeComboBox, row, 1);
    row++;
    
    // 背景颜色
    m_darkBackgroundCheckBox = new QCheckBox("暗色背景");
    layout->addWidget(m_darkBackgroundCheckBox, row, 0, 1, 2);
    row++;
    
    // 坐标系
    m_coordinateSystemCheckBox = new QCheckBox("显示坐标系");
    m_coordinateSystemCheckBox->setChecked(true);
    layout->addWidget(m_coordinateSystemCheckBox, row, 0, 1, 2);
    
    return group;
}

QGroupBox* ControlPanel::createPreprocessingGroup() {
    QGroupBox* group = new QGroupBox("预处理参数");
    group->setObjectName("preprocessingGroup");
    
    QGridLayout* layout = new QGridLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    int row = 0;
    
    // 下采样体素大小
    m_voxelSizeSpinBox = createParameterControl(layout, row++, "体素大小 (mm):",
                                               0.1, 10.0, 1.0, 0.1, 1);
    
    // 统计滤波
    m_statisticalFilterCheckBox = new QCheckBox("启用统计滤波");
    m_statisticalFilterCheckBox->setChecked(true);
    layout->addWidget(m_statisticalFilterCheckBox, row++, 0, 1, 2);
    
    // 统计滤波参数
    m_filterKSpinBox = createIntParameterControl(layout, row++, "邻居数:", 10, 200, 50);
    m_filterStddevSpinBox = createParameterControl(layout, row++, "标准差倍数:", 
                                                  0.1, 5.0, 1.0, 0.1, 1);
    
    return group;
}

QGroupBox* ControlPanel::createDetectionGroup() {
    QGroupBox* group = new QGroupBox("检测参数");
    group->setObjectName("detectionGroup");
    
    QGridLayout* layout = new QGridLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    int row = 0;
    
    // 搜索参数
    m_searchRadiusSpinBox = createParameterControl(layout, row++, "搜索半径 (mm):",
                                                  1.0, 20.0, 5.0, 0.5, 1);
    m_polynomialOrderSpinBox = createIntParameterControl(layout, row++, "多项式阶数:", 1, 5, 2);
    
    // 阈值参数
    m_depthThresholdSpinBox = createParameterControl(layout, row++, "深度阈值 (mm):",
                                                    0.1, 10.0, 2.0, 0.1, 1);
    m_areaThresholdSpinBox = createParameterControl(layout, row++, "面积阈值 (mm²):",
                                                   1.0, 1000.0, 100.0, 1.0, 0);
    m_volumeThresholdSpinBox = createParameterControl(layout, row++, "体积阈值 (mm³):",
                                                     1.0, 1000.0, 50.0, 1.0, 0);
    m_slopeAngleSpinBox = createParameterControl(layout, row++, "坡度角度 (度):",
                                                1.0, 45.0, 15.0, 1.0, 0);
    
    return group;
}

QGroupBox* ControlPanel::createClusteringGroup() {
    QGroupBox* group = new QGroupBox("聚类参数");
    group->setObjectName("clusteringGroup");
    
    QGridLayout* layout = new QGridLayout(group);
    layout->setSpacing(4);
    layout->setContentsMargins(8, 12, 8, 8);
    
    int row = 0;
    
    // 聚类参数
    m_clusterToleranceSpinBox = createParameterControl(layout, row++, "聚类容差 (mm):",
                                                      0.5, 10.0, 2.0, 0.1, 1);
    m_minClusterSizeSpinBox = createIntParameterControl(layout, row++, "最小聚类:", 5, 1000, 10);
    m_maxClusterSizeSpinBox = createIntParameterControl(layout, row++, "最大聚类:", 100, 50000, 10000);
    
    return group;
}

QVBoxLayout* ControlPanel::createActionButtons() {
    QVBoxLayout* layout = new QVBoxLayout();
    layout->setSpacing(4);
    
    // 分析控制按钮
    m_startAnalysisButton = new QPushButton("开始分析");
    m_stopAnalysisButton = new QPushButton("停止分析");
    
    // 配置管理按钮
    QHBoxLayout* configLayout = new QHBoxLayout();
    m_resetButton = new QPushButton("重置");
    m_saveConfigButton = new QPushButton("保存配置");
    m_loadConfigButton = new QPushButton("加载配置");
    
    configLayout->addWidget(m_resetButton);
    configLayout->addWidget(m_saveConfigButton);
    configLayout->addWidget(m_loadConfigButton);
    
    // 设置按钮样式
    QString buttonStyle = 
        "QPushButton {"
        "    min-height: 28px;"
        "    padding: 4px 12px;"
        "    font-size: 11px;"
        "    font-weight: bold;"
        "}"
        "QPushButton:disabled {"
        "    color: #888888;"
        "}";
    
    m_startAnalysisButton->setStyleSheet(buttonStyle + "QPushButton { background-color: #4CAF50; color: white; }");
    m_stopAnalysisButton->setStyleSheet(buttonStyle + "QPushButton { background-color: #f44336; color: white; }");
    m_resetButton->setStyleSheet(buttonStyle);
    m_saveConfigButton->setStyleSheet(buttonStyle);
    m_loadConfigButton->setStyleSheet(buttonStyle);
    
    // 设置工具提示
    m_startAnalysisButton->setToolTip("开始坑洞检测分析");
    m_stopAnalysisButton->setToolTip("停止当前分析");
    m_resetButton->setToolTip("重置所有参数到默认值");
    m_saveConfigButton->setToolTip("保存当前参数配置到文件");
    m_loadConfigButton->setToolTip("从文件加载参数配置");
    
    // 初始状态
    m_stopAnalysisButton->setEnabled(false);
    
    layout->addWidget(m_startAnalysisButton);
    layout->addWidget(m_stopAnalysisButton);
    layout->addLayout(configLayout);
    
    return layout;
}

QDoubleSpinBox* ControlPanel::createParameterControl(QGridLayout* layout, int row,
                                                    const QString& labelText,
                                                    double minValue, double maxValue,
                                                    double defaultValue, double stepSize,
                                                    int decimals) {
    QLabel* label = new QLabel(labelText);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    QDoubleSpinBox* spinBox = new QDoubleSpinBox();
    spinBox->setRange(minValue, maxValue);
    spinBox->setDecimals(decimals);
    spinBox->setSingleStep(stepSize);
    spinBox->setValue(defaultValue);
    spinBox->setMaximumHeight(24);
    
    // 设置字体
    QFont labelFont;
    labelFont.setPointSize(8);
    label->setFont(labelFont);
    
    layout->addWidget(label, row, 0);
    layout->addWidget(spinBox, row, 1);
    
    return spinBox;
}

QSpinBox* ControlPanel::createIntParameterControl(QGridLayout* layout, int row,
                                                 const QString& labelText,
                                                 int minValue, int maxValue,
                                                 int defaultValue) {
    QLabel* label = new QLabel(labelText);
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    
    QSpinBox* spinBox = new QSpinBox();
    spinBox->setRange(minValue, maxValue);
    spinBox->setValue(defaultValue);
    spinBox->setMaximumHeight(24);
    
    // 设置字体
    QFont labelFont;
    labelFont.setPointSize(8);
    label->setFont(labelFont);
    
    layout->addWidget(label, row, 0);
    layout->addWidget(spinBox, row, 1);
    
    return spinBox;
}

void ControlPanel::connectSignalsAndSlots() {
    // 可视化参数信号连接
    connect(m_pointSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onPointSizeChanged);
    connect(m_colorModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &ControlPanel::onColorModeChanged);
    connect(m_darkBackgroundCheckBox, &QCheckBox::toggled,
            this, &ControlPanel::onBackgroundColorChanged);
    connect(m_coordinateSystemCheckBox, &QCheckBox::toggled,
            this, &ControlPanel::onCoordinateSystemToggled);
    
    // 检测参数信号连接
    connect(m_voxelSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_statisticalFilterCheckBox, &QCheckBox::toggled,
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_filterKSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_filterStddevSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_searchRadiusSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_polynomialOrderSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_depthThresholdSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_areaThresholdSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_volumeThresholdSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_slopeAngleSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_clusterToleranceSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_minClusterSizeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    connect(m_maxClusterSizeSpinBox, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &ControlPanel::onDetectionParameterChanged);
    
    // 按钮信号连接
    connect(m_startAnalysisButton, &QPushButton::clicked, this, &ControlPanel::onStartAnalysis);
    connect(m_stopAnalysisButton, &QPushButton::clicked, this, &ControlPanel::onStopAnalysis);
    connect(m_resetButton, &QPushButton::clicked, this, &ControlPanel::onResetParameters);
    connect(m_saveConfigButton, &QPushButton::clicked, this, &ControlPanel::onSaveConfig);
    connect(m_loadConfigButton, &QPushButton::clicked, this, &ControlPanel::onLoadConfig);
}

void ControlPanel::updateControlsWithoutSignals() {
    m_updatingControls = true;
    
    // 更新可视化控件
    m_pointSizeSpinBox->setValue(m_vizConfig.pointSize);
    m_colorModeComboBox->setCurrentIndex(static_cast<int>(m_vizConfig.colorMode));
    m_darkBackgroundCheckBox->setChecked(m_vizConfig.backgroundColor[0] > 0.1);
    m_coordinateSystemCheckBox->setChecked(m_vizConfig.showCoordinateSystem);
    
    // 更新检测参数控件
    m_voxelSizeSpinBox->setValue(m_detectionConfig.downsampleVoxelSize);
    m_statisticalFilterCheckBox->setChecked(m_detectionConfig.enableStatisticalFilter);
    m_filterKSpinBox->setValue(m_detectionConfig.statisticalFilterK);
    m_filterStddevSpinBox->setValue(m_detectionConfig.statisticalFilterStddev);
    m_searchRadiusSpinBox->setValue(m_detectionConfig.searchRadius);
    m_polynomialOrderSpinBox->setValue(m_detectionConfig.polynomialOrder);
    m_depthThresholdSpinBox->setValue(m_detectionConfig.depthThreshold);
    m_areaThresholdSpinBox->setValue(m_detectionConfig.areaThreshold);
    m_volumeThresholdSpinBox->setValue(m_detectionConfig.volumeThreshold);
    m_slopeAngleSpinBox->setValue(m_detectionConfig.slopeAngle);
    m_clusterToleranceSpinBox->setValue(m_detectionConfig.clusterTolerance);
    m_minClusterSizeSpinBox->setValue(m_detectionConfig.minClusterSize);
    m_maxClusterSizeSpinBox->setValue(m_detectionConfig.maxClusterSize);
    
    m_updatingControls = false;
}

void ControlPanel::setupStyles() {
    // 使用主题管理器应用样式
    ThemeManager* themeManager = ThemeManager::instance();
    themeManager->applyThemeToWidget(this);
    
    // 应用组件特定的额外样式
    QString additionalStyles = themeManager->getControlPanelStyleSheet();
    if (!additionalStyles.isEmpty()) {
        setStyleSheet(styleSheet() + additionalStyles);
    }
    
    // 连接主题变化信号
    connect(themeManager, &ThemeManager::themeChanged, 
            this, [this]() {
                ThemeManager* tm = ThemeManager::instance();
                tm->applyThemeToWidget(this);
                QString additionalStyles = tm->getControlPanelStyleSheet();
                if (!additionalStyles.isEmpty()) {
                    setStyleSheet(styleSheet() + additionalStyles);
                }
            });
}

} // namespace ui
} // namespace pcl_viz

// MOC 文件将由CMake自动生成和包含