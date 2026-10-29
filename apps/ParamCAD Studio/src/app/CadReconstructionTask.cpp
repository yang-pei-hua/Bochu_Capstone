#include "app/CadReconstructionTask.h"
#include "io/PointCloudAdapter.h"

#include <QPointer>
#include <QThread>

#include <exception>
#include <string>
#include <utility>

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
    std::vector<reconstruction::SurfaceRelationHint> relations)
{
    if (m_running) return false;
    auto result = std::make_shared<CadReconstructionResult>();
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    m_cancelled = cancelled;
    m_running = true;
    m_thread = QThread::create([this, cloud = std::move(cloud), unit, result, cancelled,
                              hints = std::move(hints), sourceTolerance, analysis = std::move(analysis),
                              relations = std::move(relations)] {
        try {
            if (cancelled->load()) return;
            const auto source = PointCloudAdapter::toPointStore(cloud, unit);
            if (cancelled->load()) return;
            reconstruction::PointStore points({}, unit);
            std::string failure;
            auto preprocessing = PointCloudAdapter::preprocessingOptionsFor(cloud);
            // Keep every manual observation identifiable through preprocessing.
            for (const auto& hint : hints)
                preprocessing.protectedPointIds.insert(preprocessing.protectedPointIds.end(),
                                                       hint.points.begin(), hint.points.end());
            if (!reconstruction::preprocessPointCloud(source, preprocessing, points,
                    result->preprocessing, failure)) {
                result->outcome = CadReconstructionResult::Outcome::PreprocessingFailed;
                result->error = QString::fromStdString(failure);
                return;
            }
            result->preprocessingComplete = true;
            if (cancelled->load()) return;
            QMetaObject::invokeMethod(this, [this, cancelled, guided = !hints.empty()] {
                if (m_running && m_cancelled == cancelled && !cancelled->load())
                    emit stageChanged(guided ? Stage::FittingTemplates : Stage::Recognizing);
            }, Qt::QueuedConnection);
            auto planes=PointCloudAdapter::planeOptionsFor(points);
            auto cylinders=PointCloudAdapter::cylinderOptionsFor(points);
            if(sourceTolerance > 0) {
                planes.distanceThreshold=cylinders.distanceThreshold=
                    sourceTolerance*result->preprocessing.appliedScaleToMillimeters;
            }
            auto sourceSamples=source.points();
            const double sourceScale=result->preprocessing.appliedScaleToMillimeters;
            for(auto& p:sourceSamples) {p.position.x*=sourceScale;p.position.y*=sourceScale;p.position.z*=sourceScale;}
            const reconstruction::PointStore recoverySource(std::move(sourceSamples),points.unit());
            reconstruction::GuidedSolidResult guided;
            const bool recognized=reconstruction::reconstructGuidedSolid(points,planes,cylinders,hints,guided,failure,
                    [cancelled] { return cancelled->load(); }, analysis ? &*analysis : nullptr,relations,&recoverySource);
            if(cancelled->load())return;
            // Decompose failed/multiple-body observations before attempting
            // finite-shell closure. Explicit user constraints stay authoritative.
            const size_t bodies=guided.candidate.boxes.size()+guided.candidate.prisms.size()+guided.candidate.curvedSolids.size();
            if(hints.empty() && relations.empty() && (!recognized || bodies>1)) {
                QMetaObject::invokeMethod(this,[this,cancelled]{if(m_running&&m_cancelled==cancelled&&!cancelled->load())emit stageChanged(Stage::SeparatingParts);},Qt::QueuedConnection);
                reconstruction::AssemblyOptions options;
                // The decomposition estimates local noise independently of the
                // coarse, whole-cloud detection tolerance.
                result->assembly.emplace();std::string decompositionError;
                if(reconstruction::reconstructAssembly(recoverySource,options,*result->assembly,decompositionError,[cancelled]{return cancelled->load();})) {
                    const auto& separated=result->assembly->candidate;
                    if(!separated.profileBodies.empty() || !separated.curvedSolids.empty()) {
                        result->candidate=separated;result->analysis=std::move(guided.analysis);
                        result->outcome=CadReconstructionResult::Outcome::Success;return;
                    }
                    result->analysis=std::move(guided.analysis);
                    result->outcome=CadReconstructionResult::Outcome::Unsupported;
                    result->error=QStringLiteral("Separated %1 candidates; no verified closed solid").arg(result->assembly->parts.size());
                    return;
                } else if(!recognized)failure += "; decomposition: "+decompositionError;
            }
            if (!recognized) {
                result->analysis=std::move(guided.analysis);
                result->analysisReused=guided.analysisReused;
                result->outcome = CadReconstructionResult::Outcome::Unsupported;
                result->failedRegionId = guided.failedRegionId;
                result->error = QString::fromStdString(failure);
                return;
            }
            result->candidate=std::move(guided.candidate);
            result->analysis=std::move(guided.analysis);
            result->analysisReused=guided.analysisReused;
            result->templateFits=std::move(guided.fits);
            result->outcome = CadReconstructionResult::Outcome::Success;
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
