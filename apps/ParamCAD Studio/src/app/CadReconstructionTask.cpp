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

bool CadReconstructionTask::start(PlyCloud cloud, reconstruction::LengthUnit unit)
{
    if (m_running) return false;
    auto result = std::make_shared<CadReconstructionResult>();
    auto cancelled = std::make_shared<std::atomic_bool>(false);
    m_cancelled = cancelled;
    m_running = true;
    m_thread = QThread::create([this, cloud = std::move(cloud), unit, result, cancelled] {
        try {
            if (cancelled->load()) return;
            const auto source = PointCloudAdapter::toPointStore(cloud, unit);
            if (cancelled->load()) return;
            reconstruction::PointStore points({}, unit);
            std::string failure;
            if (!reconstruction::preprocessPointCloud(source,
                    PointCloudAdapter::preprocessingOptionsFor(cloud), points,
                    result->preprocessing, failure)) {
                result->outcome = CadReconstructionResult::Outcome::PreprocessingFailed;
                result->error = QString::fromStdString(failure);
                return;
            }
            result->preprocessingComplete = true;
            if (cancelled->load()) return;
            QMetaObject::invokeMethod(this, [this, cancelled] {
                if (m_running && m_cancelled == cancelled && !cancelled->load())
                    emit stageChanged(Stage::Recognizing);
            }, Qt::QueuedConnection);
            if (!reconstruction::reconstructParametricSolid(points,
                    PointCloudAdapter::planeOptionsFor(points),
                    PointCloudAdapter::cylinderOptionsFor(points), result->candidate, failure)) {
                result->outcome = CadReconstructionResult::Outcome::Unsupported;
                result->error = QString::fromStdString(failure);
                return;
            }
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
