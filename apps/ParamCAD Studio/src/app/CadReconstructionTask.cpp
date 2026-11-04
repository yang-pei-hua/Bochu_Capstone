#include "app/CadReconstructionTask.h"
#include "io/PointCloudAdapter.h"

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
};

void runCadWork(const CadWork& input, CadReconstructionResult& result,
    const std::function<bool()>& cancelled, const std::function<void(Stage)>& stageChanged) {
    const auto& [cloud, unit, hints, sourceTolerance, analysis, relations, projectionFallback] = input;
    if (cancelled()) return;
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
            cancelled, analysis ? &*analysis : nullptr,relations,&recoverySource);
    if(cancelled())return;
    // Decompose failed/multiple-body observations before attempting
    // finite-shell closure. Explicit user constraints stay authoritative.
    const size_t bodies=guided.candidate.boxes.size()+guided.candidate.prisms.size()+guided.candidate.curvedSolids.size();
    if(hints.empty() && relations.empty() && (!recognized || bodies>1)) {
        stageChanged(Stage::SeparatingParts);
        reconstruction::AssemblyOptions options;
        if (projectionFallback) options.maximumProjectionParts = 6;
        // The decomposition estimates local noise independently of the
        // coarse, whole-cloud detection tolerance.
        result.assembly.emplace();std::string decompositionError;
        if(reconstruction::reconstructAssembly(recoverySource,options,*result.assembly,decompositionError,cancelled)) {
            result.branch = CadReconstructionResult::Branch::Assembly;
            const auto& separated = result.assembly->candidate;
            if(!separated.profileBodies.empty() || !separated.curvedSolids.empty() ||
               !separated.projectionPrograms.empty()) {
                result.partialAssembly = !result.assembly->unassignedPointIds.empty() ||
                    std::any_of(result.assembly->parts.begin(), result.assembly->parts.end(), [](const auto& part) {
                        return !part.closed && (!part.projection || part.projection->operations.empty());
                    });
                result.candidate=separated;result.analysis=std::move(guided.analysis);
                result.outcome=CadReconstructionResult::Outcome::Success;return;
            }
            result.analysis=std::move(guided.analysis);
            result.outcome=CadReconstructionResult::Outcome::Unsupported;
            result.error=QStringLiteral("Separated %1 candidates; no verified closed solid").arg(result.assembly->parts.size());
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
    std::vector<reconstruction::SurfaceRelationHint> relations, bool projectionFallback)
{
    if (m_running) return false;
    auto result = std::make_shared<CadReconstructionResult>();
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    m_cancelled = cancelled;
    m_running = true;
    m_thread = QThread::create([this, result, cancelled, input = CadWork{
            std::move(cloud), unit, std::move(hints), sourceTolerance, std::move(analysis),
            std::move(relations), projectionFallback}] {
        try {
            auto reportStage = [this, cancelled](Stage stage) {
                QMetaObject::invokeMethod(this, [this, cancelled, stage] {
                    if (m_running && m_cancelled == cancelled && !cancelled->load())
                        emit stageChanged(stage);
                }, Qt::QueuedConnection);
            };
            runCadWork(input, *result, [cancelled] { return cancelled->load(); }, reportStage);
        } catch (const std::exception& error) {
            result->outcome = CadReconstructionResult::Outcome::Failed;
            result->error = QString::fromUtf8(error.what());
        } catch (...) {
            result->outcome = CadReconstructionResult::Outcome::Failed;
            result->error = QStringLiteral("Unknown CAD reconstruction error");
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
    if (!cancelled->load()) emit stageChanged(Stage::Preprocessing);
    thread->start();
    return true;
}
