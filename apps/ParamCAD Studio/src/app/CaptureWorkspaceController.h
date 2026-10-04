#pragma once

#include "app/WorkspaceController.h"

class QAction;
class CaptureController;
class PropertyPanel;
class VTKViewer;

struct CaptureWorkspaceUi
{
    VTKViewer* viewer;
    PropertyPanel* propertyPanel;
    QAction* captureToolAction;
};

// Coordinates capture UI, output sizes and camera feedback during orbit batches.
class CaptureWorkspaceController final : public WorkspaceController
{
    Q_OBJECT
public:
    CaptureWorkspaceController(CaptureController* captureController,
                               const CaptureWorkspaceUi& ui,
                               QWidget* dialogParent, QObject* parent = nullptr);
    void connectUi();
    void captureImage();
    void syncCaptureDistance();

private:
    void capturePhoto();
    void captureOrbit(double distance);

    CaptureController* const m_captureController;
    const CaptureWorkspaceUi m_ui;
};
