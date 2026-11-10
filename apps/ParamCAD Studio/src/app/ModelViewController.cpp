#include "app/ModelViewController.h"
#include "app/CapturePlan.h"
#include "app/CaptureSettings.h"
#include "app/ProjectPaths.h"
#include "io/CameraInfoBuilder.h"
#include "io/StepModelLoader.h"
#include "io/StepModelExporter.h"
#include "ui/panels/PropertyPanel.h"
#include "ui/panels/CameraPanel.h"
#include "ui/panels/RenderPanel.h"
#include "ui/panels/ScenePanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QApplication>
#include <QAction>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <algorithm>

ModelViewController::ModelViewController(
    CaptureController* captureController, const ModelViewUi& ui,
    QWidget* dialogParent, QObject* parent)
    : WorkspaceController(dialogParent, parent), m_captureController(captureController), m_ui(ui)
{
    m_ui.exportStepAction->setEnabled(false);
    connect(m_ui.viewer, &VTKViewer::bodyShapeChanged, this, [this](const TopoDS_Shape& shape) {
        // Both STEP import and modeling rebuild publish here. Keep the exact
        // native body currently presented, without rebuilding from render data.
        m_currentBody = shape;
        m_ui.exportStepAction->setEnabled(!shape.IsNull() && m_ui.viewer->hasBody());
        m_ui.scenePanel->setNodeState(SceneNodeType::Model, m_ui.viewer->hasBody(),
                                     m_ui.viewer->objectVisible());
    });
}

void ModelViewController::connectUi()
{
    connect(m_ui.scenePanel, &ScenePanel::nodeVisibilityRequested, this,
            [this](SceneNodeType type, bool visible) {
        if (type == SceneNodeType::Model) m_ui.viewer->setObjectVisible(visible);
    });
    connect(m_ui.viewer, &VTKViewer::objectVisibilityChanged, this, [this](bool visible) {
        auto state = m_ui.propertyPanel->presentation();
        state.visible = visible;
        m_ui.propertyPanel->setPresentation(state);
        m_ui.scenePanel->setNodeState(SceneNodeType::Model, m_ui.viewer->hasBody(), visible);
    });
    RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    // The texture controls only ask for a change; the dialog and the texture
    // itself stay with this controller and the viewport.
    connect(renderPanel, &RenderPanel::textureBrowseRequested,
            this, &ModelViewController::chooseTexture);
    connect(renderPanel, &RenderPanel::textureCleared, this, &ModelViewController::clearTexture);
    connect(renderPanel, &RenderPanel::textureEnabledChanged, this, [this](bool enabled) {
        m_ui.viewer->setBodyTextureEnabled(enabled);
        emit logInfo(enabled ? workspaceTr("纹理显示已开启") : workspaceTr("纹理显示已关闭"));
    });
    connect(renderPanel, &RenderPanel::textureProjectionChanged, this, [this](int axis) {
        m_ui.viewer->setTextureProjection(axis);
        static const char* const labels[] = {"Auto", "X", "Y", "Z", "Per Face"};
        emit logInfo(workspaceTr("Texture projection set to %1")
                    .arg(QString::fromLatin1(labels[std::clamp(axis, 0, 4)])));
    });

}

void ModelViewController::openModel()
{
    // Only STEP is parsed today, so that is the only filter offered: the dialog
    // must not promise OBJ or PLY meshes the client cannot read. Point clouds
    // keep their own "Open Point Cloud" entry.
    const QString filePath = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("打开模型"), QString(), workspaceTr("STEP 文件 (*.step *.stp)"));
    if (filePath.isEmpty()) {
        return;
    }

    loadModelFromFile(filePath);
}

void ModelViewController::loadModelFromFile(const QString& filePath)
{

    const QString fileName = QFileInfo(filePath).fileName();
    StepModelLoader stepLoader;
    if (!stepLoader.canLoad(filePath)) {
        const QString message = workspaceTr("'%1' 不是 STEP 文件；仅支持导入 .step 或 .stp 模型。").arg(fileName);
        emit logError(message);
        QMessageBox::warning(m_dialogParent, workspaceTr("打开模型"), message);
        return;
    }

    LoadedModel loaded;
    QString error;
    if (!stepLoader.load(filePath, loaded, error)) {
        // A file that could not be parsed is reported and nothing else happens:
        // whatever model is already on screen keeps its shape, its name and its
        // status, so a failure can never be mistaken for a successful load.
        emit logError(error);
        QMessageBox::warning(m_dialogParent, workspaceTr("打开模型"), error);
        return;
    }

    // The model view controller takes ownership of the B-Rep, then the viewer derives its
    // display mesh from it; the actor is replaced, not stacked on the old one.
    m_loadedModel = loaded;
    m_ui.viewer->setObjectVisible(true);
    m_ui.viewer->setBodyShape(m_loadedModel.shape);
    m_ui.propertyPanel->cameraPanel()->releaseOriginLock();
    m_ui.viewer->resetCamera();
    m_ui.viewer->renderNow();
    emit sceneFramed();

    m_ui.scenePanel->updateNodeLabel(SceneNodeType::Model, fileName);
    emit statusTextChanged(workspaceTr("就绪 | 模型：%1 | 相机：%2")
                               .arg(fileName,
                                    m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("正交投影") : workspaceTr("透视投影")));
    emit logInfo(workspaceTr("STEP model loaded from %1: %2 solids, %3 faces, %4 edges, %5 vertices")
                .arg(fileName)
                .arg(m_loadedModel.solidCount)
                .arg(m_loadedModel.faceCount)
                .arg(m_loadedModel.edgeCount)
                .arg(m_loadedModel.vertexCount));
    m_ui.viewer->setFocus(Qt::OtherFocusReason);
}

void ModelViewController::chooseTexture()
{
    if (!m_ui.viewer->hasBody()) {
        emit logWarning(workspaceTr("Model a body before projecting a texture onto it."));
        emit statusMessage(workspaceTr("暂无可贴图的对象"), 3500);
        return;
    }

    // Any picture will do: it is projected onto the body rather than unwrapped
    // onto it, so no texture coordinates have to be authored first.
    const QString filePath = QFileDialog::getOpenFileName(m_dialogParent, workspaceTr("应用纹理"), ProjectPaths::outputsRoot(),
        workspaceTr("图像 (*.png *.jpg *.jpeg *.bmp *.tif *.tiff)"));
    if (filePath.isEmpty()) {
        return;
    }

    QString error;
    if (!m_ui.viewer->loadBodyTexture(filePath, error)) {
        emit logError(error);
        QMessageBox::warning(m_dialogParent, workspaceTr("应用纹理"), error);
        return;
    }

    RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    renderPanel->setTexturePath(filePath);
    renderPanel->setTextureEnabled(true);
    const QString name = QFileInfo(filePath).fileName();
    emit statusMessage(workspaceTr("已从 %1 投影纹理").arg(name), 4000);
    emit logInfo(workspaceTr("Texture projected from %1").arg(name));
}

void ModelViewController::clearTexture()
{
    m_ui.viewer->clearBodyTexture();
    RenderPanel* renderPanel = m_ui.propertyPanel->renderPanel();
    renderPanel->setTexturePath(QString());
    renderPanel->setTextureEnabled(false);
    emit logInfo(workspaceTr("Texture removed"));
}

void ModelViewController::closeModel()
{
    m_ui.viewer->setObjectVisible(false);
    m_currentBody.Nullify();
    m_ui.exportStepAction->setEnabled(false);
    emit statusTextChanged(workspaceTr("就绪 | 模型：无 | 相机：%1")
                               .arg(m_ui.viewer->cameraParameters().parallelProjection
                                        ? workspaceTr("正交投影") : workspaceTr("透视投影")));
    emit logInfo(workspaceTr("Model closed"));
}

void ModelViewController::deleteModel()
{
    m_loadedModel = LoadedModel{};
    clearTexture();
    m_ui.viewer->clearBody();
    m_ui.viewer->setObjectVisible(true);
    m_ui.scenePanel->updateNodeLabel(SceneNodeType::Model, workspaceTr("模型"));
    emit statusTextChanged(workspaceTr("就绪 | 模型：无 | 相机：%1")
        .arg(m_ui.viewer->cameraParameters().parallelProjection
            ? workspaceTr("正交投影") : workspaceTr("透视投影")));
    emit logInfo(workspaceTr("Model deleted"));
}

bool ModelViewController::exportStepTo(const QString& path, QString& error) const
{
    return StepModelExporter::write(m_currentBody, path, error);
}

void ModelViewController::exportModel()
{
    if (m_currentBody.IsNull()) {
        emit logWarning(workspaceTr("Create or import a CAD model before exporting STEP."));
        emit statusMessage(workspaceTr("没有可导出的 CAD 模型"), 3500);
        return;
    }
    const quint64 revision = m_ui.viewer->bodyRevision();
    QString path = QFileDialog::getSaveFileName(m_dialogParent, workspaceTr("导出 STEP"),
        QDir(ProjectPaths::outputsRoot()).filePath(QStringLiteral("model.step")),
        workspaceTr("STEP 文件 (*.step *.stp)"));
    if (path.isEmpty()) return;
    const QString suffix = QFileInfo(path).suffix().toLower();
    if (suffix != QStringLiteral("step") && suffix != QStringLiteral("stp")) path += QStringLiteral(".step");

    // The save dialog runs a nested event loop: imports, close and a background
    // CAD completion must not silently change which body the user is exporting.
    if (revision != m_ui.viewer->bodyRevision() || m_currentBody.IsNull()) {
        const QString message = workspaceTr("选择文件期间模型已更改，请重新导出 STEP。");
        emit logWarning(message);
        emit statusMessage(message, 5000);
        return;
    }
    QString error;
    if (!exportStepTo(path, error)) {
        const QString message = workspaceTr("无法导出 STEP 到 %1：%2").arg(path, error);
        emit logError(message);
        emit statusMessage(message, 5000);
        QMessageBox::warning(m_dialogParent, workspaceTr("导出 STEP"), message);
        return;
    }
    const QString message = workspaceTr("STEP 模型已导出到 %1").arg(QFileInfo(path).absoluteFilePath());
    emit logInfo(message);
    emit statusMessage(message, 5000);
}

bool ModelViewController::runPerformanceCapture(const QString& modelPath,
                                       const QString& outputDirectory,
                                       const QString& texturePath,
                                       QString& errorMessage)
{
    errorMessage.clear();
    QDir output(outputDirectory);
    if (!output.exists() && !QDir().mkpath(output.absolutePath())) {
        errorMessage = workspaceTr("无法创建验证目录 %1").arg(output.absolutePath());
        return false;
    }
    if (output.exists(QStringLiteral("images"))) {
        errorMessage = workspaceTr("验证图像目录已存在：%1")
                           .arg(output.filePath(QStringLiteral("images")));
        return false;
    }

    QElapsedTimer totalTimer;
    QElapsedTimer stageTimer;
    totalTimer.start();
    stageTimer.start();

    StepModelLoader loader;
    LoadedModel loaded;
    if (!loader.load(modelPath, loaded, errorMessage)) {
        return false;
    }
    const qint64 loadMilliseconds = stageTimer.elapsed();

    m_loadedModel = loaded;
    m_ui.viewer->setObjectVisible(true);
    m_ui.viewer->setBodyShape(m_loadedModel.shape);
    m_ui.viewer->resetCamera();
    if (!texturePath.isEmpty()) {
        if (!m_ui.viewer->loadBodyTexture(texturePath, errorMessage)) {
            return false;
        }
        // A per-face projection gives every side enough stable visual detail for
        // feature matching, including thin and curved mechanical components.
        m_ui.viewer->setTextureProjection(4);
    }
    m_ui.viewer->renderNow();
    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    const double distance = m_captureController->currentDistance();
    CaptureSettings captureSettings;
    try {
        captureSettings = loadCaptureSettings();
    } catch (const std::exception&) {
    }
    const std::vector<OrbitRing> rings = captureSettings.orbitRings();
    m_captureController->setBaseDirectory(output.absolutePath());
    stageTimer.restart();
    if (!m_captureController->captureOrbit(rings, distance,
                                           captureSettings.imageWidth, captureSettings.imageHeight)) {
        errorMessage = m_captureController->lastError();
        return false;
    }
    const qint64 captureMilliseconds = stageTimer.elapsed();

    const std::vector<CaptureShot>& shots = m_captureController->shots();
    if (shots.empty()) {
        errorMessage = workspaceTr("拍摄未生成任何图像");
        return false;
    }
    const QString groupName = shots.back().groupName;
    if (!output.rename(groupName, QStringLiteral("images"))) {
        errorMessage = workspaceTr("无法将拍摄分组 %1 重命名为 images").arg(groupName);
        return false;
    }

    QString cameraInfoError;
    const QString imagesPath = output.filePath(QStringLiteral("images"));
    const QString cameraInfoPath = output.filePath(QStringLiteral("camera_info.json"));
    const CameraInfoBuildResult cameraInfo =
        CameraInfoBuilder::build(imagesPath, cameraInfoPath, cameraInfoError);
    if (!cameraInfoError.isEmpty() || !cameraInfo.built) {
        errorMessage = !cameraInfoError.isEmpty()
                           ? cameraInfoError
                           : workspaceTr("无法构建相机元数据：%1")
                                 .arg(cameraInfo.skipReason);
        return false;
    }

    QJsonObject topology;
    topology.insert(QStringLiteral("solids"), loaded.solidCount);
    topology.insert(QStringLiteral("faces"), loaded.faceCount);
    topology.insert(QStringLiteral("edges"), loaded.edgeCount);
    topology.insert(QStringLiteral("vertices"), loaded.vertexCount);

    QJsonArray ringReport;
    for (const OrbitRing& ring : rings) {
        QJsonObject item;
        item.insert(QStringLiteral("count"), ring.count);
        item.insert(QStringLiteral("elevationDeg"), ring.elevationDeg);
        ringReport.append(item);
    }

    QJsonObject timing;
    timing.insert(QStringLiteral("stepLoadMs"), loadMilliseconds);
    timing.insert(QStringLiteral("captureMs"), captureMilliseconds);
    timing.insert(QStringLiteral("totalMs"), totalTimer.elapsed());

    QJsonObject report;
    report.insert(QStringLiteral("schema"), QStringLiteral("paramcad.performance-capture"));
    report.insert(QStringLiteral("version"), 1);
    report.insert(QStringLiteral("createdAt"),
                  QDateTime::currentDateTime().toString(Qt::ISODate));
    report.insert(QStringLiteral("model"), QFileInfo(modelPath).absoluteFilePath());
    report.insert(QStringLiteral("texture"),
                  texturePath.isEmpty() ? QJsonValue(QJsonValue::Null)
                                        : QJsonValue(QFileInfo(texturePath).absoluteFilePath()));
    report.insert(QStringLiteral("topology"), topology);
    report.insert(QStringLiteral("imageCount"), cameraInfo.totalImages);
    report.insert(QStringLiteral("posedImageCount"), cameraInfo.posedImages);
    report.insert(QStringLiteral("width"), captureSettings.imageWidth);
    report.insert(QStringLiteral("height"), captureSettings.imageHeight);
    report.insert(QStringLiteral("cameraDistance"), distance);
    report.insert(QStringLiteral("rings"), ringReport);
    report.insert(QStringLiteral("timing"), timing);

    QFile reportFile(output.filePath(QStringLiteral("capture_report.json")));
    if (!reportFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || reportFile.write(QJsonDocument(report).toJson(QJsonDocument::Indented)) < 0) {
        errorMessage = workspaceTr("无法写入拍摄性能报告：%1")
                           .arg(reportFile.fileName());
        return false;
    }
    return true;
}

