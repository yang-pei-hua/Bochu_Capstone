#include "ui/panels/CapturePanel.h"

#include "core/OrbitCamera.h"

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTreeWidget>
#include <QVBoxLayout>

CapturePanel::CapturePanel(QWidget* parent)
    : QWidget(parent)
{
    m_directoryLabel = new QLineEdit(this);
    m_directoryLabel->setObjectName(QStringLiteral("captureBaseDir"));
    m_directoryLabel->setReadOnly(true);

    m_browseButton = new QPushButton(tr("浏览…"), this);
    m_browseButton->setObjectName(QStringLiteral("captureBrowseButton"));

    auto* directoryRow = new QWidget(this);
    auto* directoryLayout = new QHBoxLayout(directoryRow);
    directoryLayout->setContentsMargins(0, 0, 0, 0);
    directoryLayout->setSpacing(4);
    directoryLayout->addWidget(m_directoryLabel, 1);
    directoryLayout->addWidget(m_browseButton);

    auto* outputGroup = new QGroupBox(tr("输出"), this);
    auto* outputForm = new QFormLayout(outputGroup);
    outputForm->addRow(tr("基础目录"), directoryRow);

    m_distance = new QDoubleSpinBox(this);
    m_distance->setObjectName(QStringLiteral("captureDistance"));
    m_distance->setRange(0.001, 100000.0);
    m_distance->setDecimals(3);
    m_distance->setSingleStep(1.0);
    m_distance->setValue(OrbitParameters{}.distance);

    auto* orbitGroup = new QGroupBox(tr("球面拍摄"), this);
    auto* orbitForm = new QFormLayout(orbitGroup);
    orbitForm->addRow(tr("距离"), m_distance);
    auto* gridSummary = new QLabel(
        tr("共 62 个视角：经度每 30°，纬度取 -60°、-30°、0°、+30°、+60°，另加一个顶视图和一个底视图；所有相机均朝向模型质心。"),
        this);
    gridSummary->setWordWrap(true);
    orbitForm->addRow(gridSummary);

    m_photoButton = new QPushButton(tr("拍摄照片"), this);
    m_photoButton->setObjectName(QStringLiteral("capturePhotoButton"));
    m_photoButton->setProperty("primary", true);

    m_orbitButton = new QPushButton(tr("拍摄 62 个视角"), this);
    m_orbitButton->setObjectName(QStringLiteral("captureOrbitButton"));

    // The render panel resizes the viewport image to the configured output size
    // without preserving the window's aspect ratio, so keep the user informed.
    m_aspectHint = new QLabel(
        tr("图像会缩放到「渲染」页的输出尺寸。"), this);
    m_aspectHint->setWordWrap(true);
    m_aspectHint->setEnabled(false);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(m_photoButton);
    buttonRow->addWidget(m_orbitButton);

    m_shots = new QTreeWidget(this);
    m_shots->setObjectName(QStringLiteral("captureShotList"));
    m_shots->setColumnCount(5);
    m_shots->setHeaderLabels({tr("#"), tr("分组"), tr("图像"), tr("方位角"),
                              tr("俯仰角")});
    m_shots->setRootIsDecorated(false);
    m_shots->setUniformRowHeights(true);
    m_shots->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_shots->setSelectionMode(QAbstractItemView::SingleSelection);
    m_shots->setMinimumHeight(120);
    m_shots->header()->setSectionResizeMode(1, QHeaderView::Stretch);

    auto* shotsGroup = new QGroupBox(tr("已拍摄"), this);
    auto* shotsLayout = new QVBoxLayout(shotsGroup);
    shotsLayout->setContentsMargins(6, 6, 6, 6);
    shotsLayout->addWidget(m_shots);

    m_status = new QLabel(tr("就绪"), this);
    m_status->setObjectName(QStringLiteral("captureStatus"));
    m_status->setWordWrap(true);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);
    layout->addWidget(outputGroup);
    layout->addWidget(orbitGroup);
    layout->addLayout(buttonRow);
    layout->addWidget(m_aspectHint);
    layout->addWidget(m_status);
    layout->addWidget(shotsGroup, 1);

    connect(m_browseButton, &QPushButton::clicked, this, &CapturePanel::browseForDirectory);
    connect(m_photoButton, &QPushButton::clicked, this, &CapturePanel::capturePhotoRequested);
    connect(m_orbitButton, &QPushButton::clicked, this, [this] {
        emit captureOrbitRequested(m_distance->value());
    });
}

QString CapturePanel::baseDirectory() const
{
    return m_baseDirectory;
}

void CapturePanel::setBaseDirectory(const QString& directory)
{
    if (directory.isEmpty() || directory == m_baseDirectory) {
        return;
    }
    m_baseDirectory = directory;
    updateDirectoryLabel();
}

void CapturePanel::setDefaultDistance(double distance)
{
    m_distance->setValue(distance > 0.0 ? distance : OrbitParameters{}.distance);
}

void CapturePanel::setShots(const std::vector<CaptureShot>& shots)
{
    m_shots->clear();
    for (const CaptureShot& shot : shots) {
        auto* item = new QTreeWidgetItem(m_shots);
        item->setText(0, QString::number(shot.index));
        item->setText(1, shot.groupName);
        item->setText(2, shot.imageName);
        item->setText(3, QString::number(shot.azimuthDeg, 'f', 1));
        item->setText(4, QString::number(shot.elevationDeg, 'f', 1));
    }
    m_shots->resizeColumnToContents(0);
    m_shots->resizeColumnToContents(2);
    m_shots->resizeColumnToContents(3);
    m_shots->resizeColumnToContents(4);
}

void CapturePanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void CapturePanel::setBusy(bool busy)
{
    m_photoButton->setEnabled(!busy);
    m_orbitButton->setEnabled(!busy);
    m_browseButton->setEnabled(!busy);
    m_distance->setEnabled(!busy);
}

void CapturePanel::browseForDirectory()
{
    const QString directory = QFileDialog::getExistingDirectory(
        this, tr("拍摄输出目录"), m_baseDirectory);
    if (directory.isEmpty()) {
        return;
    }
    m_baseDirectory = directory;
    updateDirectoryLabel();
    emit baseDirectoryChanged(directory);
}

void CapturePanel::updateDirectoryLabel()
{
    m_directoryLabel->setText(m_baseDirectory);
    m_directoryLabel->setToolTip(m_baseDirectory);
}
