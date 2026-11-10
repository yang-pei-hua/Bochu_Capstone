#pragma once

#include "app/CaptureController.h"
#include <reconstruction/settings/ModuleSettings.h>

#include <array>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

// Capture plan and output-size hyperparameters. The seven ring slots match the
// historical 62-view plan; a slot with zero shots is skipped.
struct CaptureSettings {
    int imageWidth = 1920;
    int imageHeight = 1080;
    std::array<int, 7> ringShots{12, 12, 12, 12, 12, 1, 1};
    std::array<double, 7> ringElevationDegrees{-60.0, -30.0, 0.0, 30.0, 60.0, 90.0, -90.0};

    std::vector<OrbitRing> orbitRings() const;
};

std::filesystem::path captureConfigPath();
CaptureSettings loadCaptureSettings(const std::filesystem::path& path = {});
std::string serializeCaptureSettings(const CaptureSettings& settings);
void setCaptureSetting(CaptureSettings& settings, std::string_view assignment);
void validateCaptureSettings(const CaptureSettings& settings);
reconstruction::settings::DocumentPtr makeCaptureSettingsDocument();
