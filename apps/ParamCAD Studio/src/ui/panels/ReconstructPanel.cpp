#include "ui/panels/ReconstructPanel.h"

#include "app/ProjectPaths.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QVBoxLayout>

ReconstructPanel::ReconstructPanel(QWidget* parent)
    : QWidget(parent)
{
    m_captureBaseDirectory = ProjectPaths::capturesRoot();

    // Editable rather than read-only so a path can be pasted in, which is also
    // what the UI automation relies on to fill the field.
    m_imageDirectory = new QLineEdit(this);
    m_imageDirectory->setObjectName(QStringLiteral("reconstructImageDir"));
    m_imageDirectory->setToolTip(tr("用于重建的多视角图像文件夹"));

    m_browseImagesButton = new QPushButton(tr("浏览…"), this);
    m_browseImagesButton->setObjectName(QStringLiteral("reconstructBrowseButton"));

    m_useLatestButton = new QPushButton(tr("使用最近一次拍摄"), this);
    m_useLatestButton->setObjectName(QStringLiteral("reconstructUseLatestButton"));

    auto* imagesRow = new QWidget(this);
    auto* imagesLayout = new QHBoxLayout(imagesRow);
    imagesLayout->setContentsMargins(0, 0, 0, 0);
    imagesLayout->setSpacing(4);
    imagesLayout->addWidget(m_imageDirectory, 1);
    imagesLayout->addWidget(m_browseImagesButton);

    auto* imagesGroup = new QGroupBox(tr("图像"), this);
    auto* imagesForm = new QFormLayout(imagesGroup);
    imagesForm->addRow(tr("目录"), imagesRow);
    imagesForm->addRow(QString(), m_useLatestButton);

    m_outputDirectory = new QLineEdit(this);
    m_outputDirectory->setObjectName(QStringLiteral("reconstructOutputDir"));
    m_outputDirectory->setText(QDir::cleanPath(ProjectPaths::reconstructionsRoot()));
    m_outputDirectory->setToolTip(tr("每次运行都会在此处创建独立的时间戳子目录"));

    m_browseOutputButton = new QPushButton(tr("浏览…"), this);
    m_browseOutputButton->setObjectName(QStringLiteral("reconstructOutputBrowseButton"));

    auto* outputRow = new QWidget(this);
    auto* outputLayout = new QHBoxLayout(outputRow);
    outputLayout->setContentsMargins(0, 0, 0, 0);
    outputLayout->setSpacing(4);
    outputLayout->addWidget(m_outputDirectory, 1);
    outputLayout->addWidget(m_browseOutputButton);

    auto* outputGroup = new QGroupBox(tr("输出"), this);
    auto* outputForm = new QFormLayout(outputGroup);
    outputForm->addRow(tr("目录"), outputRow);

    m_useCameraInfo = new QCheckBox(tr("使用拍摄位姿与内参"), this);
    m_useCameraInfo->setObjectName(QStringLiteral("reconstructUseCameraInfo"));
    m_useCameraInfo->setChecked(true);

    m_dense = new QCheckBox(tr("稠密重建（GPU）"), this);
    m_dense->setObjectName(QStringLiteral("reconstructDense"));

    m_matcher = new QComboBox(this);
    m_matcher->setObjectName(QStringLiteral("reconstructMatcher"));
    m_matcher->addItems({tr("穷举匹配"), tr("顺序匹配")});

    auto* optionsGroup = new QGroupBox(tr("选项"), this);
    auto* optionsForm = new QFormLayout(optionsGroup);
    optionsForm->addRow(QString(), m_useCameraInfo);
    optionsForm->addRow(QString(), m_dense);
    optionsForm->addRow(tr("匹配器"), m_matcher);

    m_runButton = new QPushButton(tr("重建"), this);
    m_runButton->setObjectName(QStringLiteral("reconstructRunButton"));
    m_runButton->setProperty("primary", true);

    m_cancelButton = new QPushButton(tr("取消"), this);
    m_cancelButton->setObjectName(QStringLiteral("reconstructCancelButton"));
    m_cancelButton->setEnabled(false);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->addWidget(m_runButton);
    buttonRow->addWidget(m_cancelButton);

    m_progress = new QProgressBar(this);
    m_progress->setObjectName(QStringLiteral("reconstructProgress"));
    m_progress->setRange(0, 4);
    m_progress->setValue(0);
    m_progress->setFormat(tr("%1/%2").arg(0).arg(4));

    m_status = new QLabel(tr("就绪"), this);
    m_status->setObjectName(QStringLiteral("reconstructStatus"));
    m_status->setWordWrap(true);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);
    layout->addWidget(imagesGroup);
    layout->addWidget(outputGroup);
    layout->addWidget(optionsGroup);
    layout->addLayout(buttonRow);
    layout->addWidget(m_progress);
    layout->addWidget(m_status);
    layout->addStretch(1);

    const QString latest = latestCaptureDirectory();
    if (!latest.isEmpty()) {
        setImageDirectory(latest);
    }

    connect(m_browseImagesButton, &QPushButton::clicked, this, &ReconstructPanel::browseForImages);
    connect(m_browseOutputButton, &QPushButton::clicked, this, &ReconstructPanel::browseForOutput);
    connect(m_useLatestButton, &QPushButton::clicked, this, &ReconstructPanel::useLatestCapture);
    connect(m_runButton, &QPushButton::clicked, this, &ReconstructPanel::startReconstruction);
    connect(m_cancelButton, &QPushButton::clicked, this, &ReconstructPanel::cancelRequested);
}

QString ReconstructPanel::imageDirectory() const
{
    return m_imageDirectory->text().trimmed();
}

void ReconstructPanel::setImageDirectory(const QString& directory)
{
    m_imageDirectory->setText(QDir::cleanPath(directory));
    m_imageDirectory->setToolTip(m_imageDirectory->text());
}

QString ReconstructPanel::outputRoot() const
{
    return m_outputDirectory->text().trimmed();
}

void ReconstructPanel::setOutputRoot(const QString& directory)
{
    m_outputDirectory->setText(QDir::cleanPath(directory));
    m_outputDirectory->setToolTip(m_outputDirectory->text());
}

void ReconstructPanel::setCaptureBaseDirectory(const QString& directory)
{
    if (directory.isEmpty() || directory == m_captureBaseDirectory) {
        return;
    }
    m_captureBaseDirectory = directory;
    if (m_imageDirectory->text().isEmpty()) {
        const QString latest = latestCaptureDirectory();
        if (!latest.isEmpty()) {
            setImageDirectory(latest);
        }
    }
}

void ReconstructPanel::setStatusText(const QString& text)
{
    m_status->setText(text);
}

void ReconstructPanel::setBusy(bool busy)
{
    m_runButton->setEnabled(!busy);
    m_cancelButton->setEnabled(busy);
    m_imageDirectory->setEnabled(!busy);
    m_outputDirectory->setEnabled(!busy);
    m_browseImagesButton->setEnabled(!busy);
    m_browseOutputButton->setEnabled(!busy);
    m_useLatestButton->setEnabled(!busy);
    m_useCameraInfo->setEnabled(!busy);
    m_dense->setEnabled(!busy);
    m_matcher->setEnabled(!busy);
}

void ReconstructPanel::setProgress(int value, int maximum)
{
    if (maximum > 0) {
        m_progress->setRange(0, maximum);
    }
    m_progress->setValue(value);
    m_progress->setFormat(tr("%1/%2").arg(value).arg(m_progress->maximum()));
}

QString ReconstructPanel::latestCaptureDirectory() const
{
    // Capture groups are named yyyyMMdd_HHmmss (with a -N suffix after a
    // collision). The base directory defaults to outputs/captures, which holds
    // nothing else, but it stays user-selectable and may point at a folder with
    // unrelated entries, so only matching names are considered.
    static const QRegularExpression stamp(QStringLiteral("^\\d{8}_\\d{6}(-\\d+)?$"));
    const QDir base(m_captureBaseDirectory);
    const QStringList entries =
        base.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    QString latest;
    for (const QString& entry : entries) {
        if (stamp.match(entry).hasMatch()) {
            // A plain timestamp sorts chronologically as text, and the -N suffix
            // sorts after its base name, so the last match is the newest group.
            latest = entry;
        }
    }
    return latest.isEmpty() ? QString() : base.filePath(latest);
}

void ReconstructPanel::browseForImages()
{
    const QString directory =
        QFileDialog::getExistingDirectory(this, tr("图像目录"), imageDirectory());
    if (directory.isEmpty()) {
        return;
    }
    setImageDirectory(directory);
}

void ReconstructPanel::browseForOutput()
{
    const QString directory =
        QFileDialog::getExistingDirectory(this, tr("输出目录"), outputRoot());
    if (directory.isEmpty()) {
        return;
    }
    setOutputRoot(directory);
}

void ReconstructPanel::useLatestCapture()
{
    const QString latest = latestCaptureDirectory();
    if (latest.isEmpty()) {
        setStatusText(tr("在 %1 中未找到拍摄文件夹").arg(m_captureBaseDirectory));
        return;
    }
    setImageDirectory(latest);
    setStatusText(tr("使用 %1").arg(latest));
}

void ReconstructPanel::startReconstruction()
{
    ReconstructRequest request;
    request.imageDirectory = imageDirectory();
    request.outputRoot = outputRoot();
    request.useCameraInfo = m_useCameraInfo->isChecked();
    request.dense = m_dense->isChecked();
    request.matcher = m_matcher->currentIndex() == 1 ? QStringLiteral("sequential")
                                                     : QStringLiteral("exhaustive");
    emit reconstructRequested(request);
}