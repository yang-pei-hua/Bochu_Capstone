#pragma once

#include "io/PlyReader.h"

#include <reconstruction/CompoundRecognizer.h>
#include <reconstruction/PointCloudPreprocessor.h>
#include <reconstruction/GuidedSolidRecognizer.h>

#include <QObject>

#include <atomic>
#include <memory>

class QThread;

struct CadReconstructionResult
{
    enum class Outcome { Success, PreprocessingFailed, Unsupported, Failed, Cancelled };
    Outcome outcome = Outcome::Failed;
    bool preprocessingComplete = false;
    reconstruction::PointCloudPreprocessingReport preprocessing;
    reconstruction::CompoundCandidate candidate;
    std::vector<reconstruction::TemplateFit> templateFits;
    int failedRegionId = 0;
    QString error;
};

// Computes a candidate from an owned input snapshot. No view or modeling document
// is accessed by the worker; the workspace decides whether to commit the result.
class CadReconstructionTask final : public QObject
{
    Q_OBJECT
public:
    enum class Stage { Preprocessing, FittingTemplates, Recognizing, Cancelling };
    Q_ENUM(Stage)

    explicit CadReconstructionTask(QObject* parent = nullptr);
    ~CadReconstructionTask() override;
    bool start(PlyCloud cloud, reconstruction::LengthUnit unit,
               std::vector<reconstruction::TemplateHint> hints = {}, double sourceTolerance = 0);
    bool isRunning() const noexcept;
    bool isCancellationRequested() const noexcept;
    void cancel();

signals:
    void runningChanged(bool running);
    void stageChanged(CadReconstructionTask::Stage stage);
    // Delivered on this object's owning thread, while isRunning() is still true.
    void finished(const CadReconstructionResult& result);

private:
    QThread* m_thread = nullptr;
    bool m_running = false;
    std::shared_ptr<std::atomic_bool> m_cancelled;
};
