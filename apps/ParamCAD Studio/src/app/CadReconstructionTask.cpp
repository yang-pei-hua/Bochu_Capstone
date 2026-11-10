#include "app/CadReconstructionTask.h"
#include <QCoreApplication>
#include "io/PointCloudAdapter.h"
#include <modeling/ModelingCore.h>
#include <reconstruction/recognition/ProjectionSettings.h>
#include <reconstruction/settings/ModuleConfig.h>

#include <QElapsedTimer>
#include <QPointer>
#include <QThread>

#include <algorithm>
#include <exception>
#include <string>
#include <utility>

namespace {
using Stage = CadReconstructionTask::Stage;
struct CadWork {
    PlyCloud cloud;
    reconstruction::LengthUnit unit;
    std::vector<reconstruction::TemplateHint> hints;
    double sourceTolerance;
    std::optional<reconstruction::SurfaceAnalysisResult> analysis;
    std::vector<reconstruction::SurfaceRelationHint> relations;
    bool projectionFallback;
    bool specializedRecovery;
    std::optional<reconstruction::ProjectionExtrusionOptions> projectionOptions;
};

void runCadWork(const CadWork& input, CadReconstructionResult& result,
    const std::function<bool()>& cancelled, const std::function<void(Stage)>& stageChanged) {
    const auto& [cloud, unit, hints, sourceTolerance, analysis, relations, projectionFallback, specializedRecovery,
                 projectionOptions] = input;
    if (cancelled()) return;
    reconstruction::RecognitionSettings recognitionSettings;
    reconstruction::AssemblySettings assemblySettings;
    try {
        recognitionSettings = reconstruction::loadRecognitionSettings();
        assemblySettings = reconstruction::loadAssemblySettings();
    } catch (const std::exception& error) {
        result.outcome = CadReconstructionResult::Outcome::Failed;
        result.error = QString::fromUtf8(error.what());
        return;
    }
    const reconstruction::ObservationSnapshot source(PointCloudAdapter::toPointStore(cloud, unit));
    result.skippedInvalidObservations = cloud.positions.size() - source.observations().size();
    if (cancelled()) return;
    reconstruction::PointStore points({}, unit);
    std::string failure;
    auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud);
    // Keep every manual observation identifiable through preprocessing.
    for (const auto& hint : hints)
        preprocessing.protectedPointIds.insert(preprocessing.protectedPointIds.end(),
                                               hint.points.begin(), hint.points.end());
    if (!reconstruction::preprocessPointCloud(source, preprocessing, points,
            result.preprocessing, failure)) {
        result.outcome = CadReconstructionResult::Outcome::PreprocessingFailed;
        result.error = QString::fromStdString(failure);
        return;
    }
    result.preprocessingComplete = true;
    if (cancelled()) return;
    result.branch = hints.empty() ? CadReconstructionResult::Branch::Conventional
                                  : CadReconstructionResult::Branch::Guided;
    stageChanged(hints.empty() ? Stage::Recognizing : Stage::FittingTemplates);
    auto planes=PointCloudAdapter::planeOptionsFor(points);
    auto cylinders=PointCloudAdapter::cylinderOptionsFor(points);
    if(sourceTolerance > 0) {
        planes.distanceThreshold=cylinders.distanceThreshold=
            sourceTolerance*result.preprocessing.appliedScaleToMillimeters;
    }
    const auto& recoverySource = source.observations();
    reconstruction::GuidedSolidResult guided;
    const bool recognized=reconstruction::reconstructGuidedSolid(points,planes,cylinders,hints,guided,failure,
            cancelled, analysis ? &*analysis : nullptr,relations,&recoverySource,&recognitionSettings);
    if(cancelled())return;
    // Decompose failed/multiple-body observations before attempting
    // finite-shell closure. Explicit user constraints stay authoritative.
    const size_t bodies=guided.candidate.boxes.size()+guided.candidate.prisms.size()+guided.candidate.curvedSolids.size();
    const bool refineBox = recognized && bodies == 1 && guided.candidate.boxes.size() == 1 &&
        guided.candidate.surfaceCoverage < .99;
    if(hints.empty() && relations.empty() && (!recognized || bodies>1 || refineBox)) {
        stageChanged(Stage::SeparatingParts);
        reconstruction::AssemblyOptions options = assemblySettings;
        options.enableSpecializedRecovery = specializedRecovery;
        options.projectionOptions = projectionOptions;
        if (!projectionFallback || refineBox) options.maximumProjectionParts = 0;
        // The decomposition estimates local noise independently of the
        // coarse, whole-cloud detection tolerance.
        result.assembly.emplace();std::string decompositionError;
        const bool separatedOk = reconstruction::reconstructAssembly(recoverySource,options,*result.assembly,decompositionError,cancelled);
        if (cancelled()) return;
        bool profileRefined = false;
        if (refineBox && separatedOk && result.assembly->parts.size() == 1 &&
            result.assembly->parts.front().status == "closed_analytic_profile" &&
            result.assembly->candidate.profileBodies.size() == 1) {
            const auto& profile = result.assembly->candidate.profileBodies.front();
            const auto throughCuts = std::count_if(profile.cavities.begin(), profile.cavities.end(),
                [&](const auto& entry) {
                    return entry.start == 0 && std::any_of(profile.cavities.begin(),profile.cavities.end(),[&](const auto& exit) {
                        return exit.end == profile.depth && exit.centerX == entry.centerX && exit.centerY == entry.centerY;
                    });
                });
            profileRefined = std::size_t(throughCuts) >= guided.candidate.throughHoles.size();
        }
        if (refineBox && !profileRefined) {
            result.assembly.reset();
        } else if (separatedOk) {
            result.branch = CadReconstructionResult::Branch::Assembly;
            const auto& separated = result.assembly->candidate;
            if(!separated.profileBodies.empty() || !separated.curvedSolids.empty() ||
               !separated.projectionPrograms.empty()) {
                result.partialAssembly = !result.assembly->unassignedPointIds.empty() ||
                    std::any_of(result.assembly->parts.begin(), result.assembly->parts.end(), [](const auto& part) {
                        return part.status == "closed_circular_slab_inferred_contact" ||
                            !part.closed;
                    });
                result.candidate=separated;result.analysis=std::move(guided.analysis);
                result.outcome=CadReconstructionResult::Outcome::Success;return;
            }
            result.analysis=std::move(guided.analysis);
            result.outcome=CadReconstructionResult::Outcome::Unsupported;
            result.error=QCoreApplication::translate("CadReconstructionTask", "分离出 %1 个候选，但没有验证闭合的实体").arg(result.assembly->parts.size());
            if (!recognized && !failure.empty()) result.error += "; conventional: " + QString::fromStdString(failure);
            return;
        } else if(!recognized)failure += "; decomposition: "+decompositionError;
    }
    if (!recognized) {
        result.analysis=std::move(guided.analysis);
        result.analysisReused=guided.analysisReused;
        result.outcome = CadReconstructionResult::Outcome::Unsupported;
        result.failedRegionId = guided.failedRegionId;
        result.error = QString::fromStdString(failure);
        return;
    }
    result.candidate=std::move(guided.candidate);
    result.analysis=std::move(guided.analysis);
    result.analysisReused=guided.analysisReused;
    result.templateFits=std::move(guided.fits);
    result.outcome = CadReconstructionResult::Outcome::Success;
}
} // namespace

CadReconstructionTask::CadReconstructionTask(QObject* parent)
    : QObject(parent)
{
}

CadReconstructionTask::~CadReconstructionTask()
{
    // The geometry routines cannot be interrupted mid-call. Stop at the next
    // stage boundary and join before QObject removes queued callbacks.
    if (m_thread) {
        m_cancelled->store(true);
        disconnect(m_thread, nullptr, this, nullptr);
        m_thread->requestInterruption();
        m_thread->wait();
    }
}

bool CadReconstructionTask::isRunning() const noexcept { return m_running; }

bool CadReconstructionTask::isCancellationRequested() const noexcept
{
    return m_running && m_cancelled && m_cancelled->load();
}

void CadReconstructionTask::cancel()
{
    if (!m_running || m_cancelled->exchange(true)) return;
    emit stageChanged(Stage::Cancelling);
}

bool CadReconstructionTask::start(PlyCloud cloud, reconstruction::LengthUnit unit,
    std::vector<reconstruction::TemplateHint> hints, double sourceTolerance,
    std::optional<reconstruction::SurfaceAnalysisResult> analysis,
    std::vector<reconstruction::SurfaceRelationHint> relations, bool projectionFallback, bool specializedRecovery,
    std::optional<reconstruction::ProjectionExtrusionOptions> projectionOptions)
{
    return startWork([input = CadWork{std::move(cloud), unit, std::move(hints), sourceTolerance,
            std::move(analysis), std::move(relations), projectionFallback, specializedRecovery, std::move(projectionOptions)}]
            (auto& result, const auto& cancelled, const auto& reportStage) {
        runCadWork(input, result, cancelled, reportStage);
    }, Stage::Preprocessing);
}

bool CadReconstructionTask::startProjection(PlyCloud cloud, reconstruction::LengthUnit unit,
                                            reconstruction::ProjectionPath path, bool inspectOnly,
                                            std::optional<reconstruction::ProjectionExtrusionOptions> projectionOptions)
{
    return startWork([cloud = std::move(cloud), unit, path, inspectOnly, injected = std::move(projectionOptions)]
                     (auto& result, const auto& cancelled, const auto&) {
        QElapsedTimer timer;
        timer.start();
        auto& debug = result.projectionDebug.emplace();
        debug.path = path;
        debug.inspectOnly = inspectOnly;
        debug.sourceUnit = unit;
        debug.inputPoints = cloud.positions.size();
        const reconstruction::ObservationSnapshot source(PointCloudAdapter::toPointStore(cloud, unit));
        debug.validPoints = source.observations().size();
        auto options = injected ? *injected : reconstruction::loadProjectionOptions();
        options.retainRejectedProgram = true;
        options.path = path;
        options.collectSketchInputs = true;
        const bool accepted = inspectOnly
            ? reconstruction::inspectProjectionSketches(source.observations(), options, debug.fit, cancelled)
            : reconstruction::fitProjectionExtrusion(source.observations(), options, debug.fit, {}, cancelled);
        const auto& preview = accepted ? debug.fit.operations : debug.fit.rejectedOperations;
        if (!preview.empty() && !cancelled()) {
            modeling::ModelingCore model;
            std::string error;
            if (reconstruction::buildProjectionProgram(preview, model, error))
                debug.previewShape = model.bodyShape();
        }
        debug.elapsedMilliseconds = timer.elapsed();
        result.outcome = accepted ? CadReconstructionResult::Outcome::Success
                                  : CadReconstructionResult::Outcome::Unsupported;
        if (!accepted) result.error = QString::fromStdString(debug.fit.reason);
    }, Stage::Projecting);
}

bool CadReconstructionTask::startWork(Work work, Stage initialStage)
{
    if (m_running) return false;
    auto result = std::make_shared<CadReconstructionResult>();
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    m_cancelled = cancelled;
    m_running = true;
    m_thread = QThread::create([this, result, cancelled, work = std::move(work)] {
        try {
            auto reportStage = [this, cancelled](Stage stage) {
                QMetaObject::invokeMethod(this, [this, cancelled, stage] {
                    if (m_running && m_cancelled == cancelled && !cancelled->load())
                        emit stageChanged(stage);
                }, Qt::QueuedConnection);
            };
            work(*result, [cancelled] { return cancelled->load(); }, reportStage);
        } catch (const std::exception& error) {
            result->outcome = CadReconstructionResult::Outcome::Failed;
            result->error = QString::fromUtf8(error.what());
        } catch (...) {
            result->outcome = CadReconstructionResult::Outcome::Failed;
            result->error = QCoreApplication::translate("CadReconstructionTask", "未知的 CAD 重建错误");
        }
    });
    auto* thread = m_thread;
    thread->setParent(this);
    connect(thread, &QThread::finished, this, [this, result, cancelled] {
        m_thread = nullptr;
        // Cancellation also wins if computation has finished but delivery is
        // still queued. Keep the job busy throughout the result handler.
        if (cancelled->load()) result->outcome = CadReconstructionResult::Outcome::Cancelled;
        QPointer<CadReconstructionTask> guard(this);
        emit finished(*result);
        if (!guard) return;
        m_running = false;
        emit runningChanged(false);
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    emit runningChanged(true);
    if (!cancelled->load()) emit stageChanged(initialStage);
    thread->start();
    return true;
}
