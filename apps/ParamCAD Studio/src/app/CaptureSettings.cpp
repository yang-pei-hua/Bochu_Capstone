#include "app/CaptureSettings.h"

#include <algorithm>
#include <stdexcept>

namespace {

const std::vector<reconstruction::settings::Setting<CaptureSettings>>& captureSchema()
{
    using S = CaptureSettings;
    using T = reconstruction::settings::Setting<S>;
    static const std::vector<T> values{
        { "image_width", "输出", "截图宽度", T::Type::Integer, 64, 16384,
          [](const S& s) { return double(s.imageWidth); },
          [](S& s, double v) { s.imageWidth = static_cast<int>(v); } },
        { "image_height", "输出", "截图高度", T::Type::Integer, 64, 16384,
          [](const S& s) { return double(s.imageHeight); },
          [](S& s, double v) { s.imageHeight = static_cast<int>(v); } },
        { "ring1_shots", "轨道环", "第 1 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[0]); },
          [](S& s, double v) { s.ringShots[0] = static_cast<int>(v); } },
        { "ring1_elevation_deg", "轨道环", "第 1 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[0]; },
          [](S& s, double v) { s.ringElevationDegrees[0] = v; } },
        { "ring2_shots", "轨道环", "第 2 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[1]); },
          [](S& s, double v) { s.ringShots[1] = static_cast<int>(v); } },
        { "ring2_elevation_deg", "轨道环", "第 2 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[1]; },
          [](S& s, double v) { s.ringElevationDegrees[1] = v; } },
        { "ring3_shots", "轨道环", "第 3 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[2]); },
          [](S& s, double v) { s.ringShots[2] = static_cast<int>(v); } },
        { "ring3_elevation_deg", "轨道环", "第 3 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[2]; },
          [](S& s, double v) { s.ringElevationDegrees[2] = v; } },
        { "ring4_shots", "轨道环", "第 4 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[3]); },
          [](S& s, double v) { s.ringShots[3] = static_cast<int>(v); } },
        { "ring4_elevation_deg", "轨道环", "第 4 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[3]; },
          [](S& s, double v) { s.ringElevationDegrees[3] = v; } },
        { "ring5_shots", "轨道环", "第 5 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[4]); },
          [](S& s, double v) { s.ringShots[4] = static_cast<int>(v); } },
        { "ring5_elevation_deg", "轨道环", "第 5 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[4]; },
          [](S& s, double v) { s.ringElevationDegrees[4] = v; } },
        { "ring6_shots", "轨道环", "第 6 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[5]); },
          [](S& s, double v) { s.ringShots[5] = static_cast<int>(v); } },
        { "ring6_elevation_deg", "轨道环", "第 6 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[5]; },
          [](S& s, double v) { s.ringElevationDegrees[5] = v; } },
        { "ring7_shots", "轨道环", "第 7 环张数", T::Type::Integer, 0, 512,
          [](const S& s) { return double(s.ringShots[6]); },
          [](S& s, double v) { s.ringShots[6] = static_cast<int>(v); } },
        { "ring7_elevation_deg", "轨道环", "第 7 环仰角", T::Type::Real, -90, 90,
          [](const S& s) { return s.ringElevationDegrees[6]; },
          [](S& s, double v) { s.ringElevationDegrees[6] = v; } },
    };
    return values;
}

void validateCapture(const CaptureSettings& settings, std::string& reason)
{
    std::size_t total = 0;
    for (const int shots : settings.ringShots) total += static_cast<std::size_t>(std::max(0, shots));
    if (total == 0) reason = "At least one capture ring must have a positive shot count";
}

}  // namespace

std::vector<OrbitRing> CaptureSettings::orbitRings() const
{
    std::vector<OrbitRing> rings;
    for (std::size_t index = 0; index < ringShots.size(); ++index)
        if (ringShots[index] > 0)
            rings.push_back({ringShots[index], ringElevationDegrees[index]});
    return rings;
}

std::filesystem::path captureConfigPath()
{
    return reconstruction::settings::configPath("capture", "capture.conf");
}

CaptureSettings loadCaptureSettings(const std::filesystem::path& path)
{
    return reconstruction::settings::loadOptions(captureSchema(), "capture", "capture.conf", path, validateCapture);
}

std::string serializeCaptureSettings(const CaptureSettings& settings)
{
    validateCaptureSettings(settings);
    return reconstruction::settings::serializeOptions(captureSchema(), settings, "Capture");
}

void setCaptureSetting(CaptureSettings& settings, std::string_view assignment)
{
    reconstruction::settings::setOption(captureSchema(), settings, assignment, "capture");
}

void validateCaptureSettings(const CaptureSettings& settings)
{
    reconstruction::settings::validateOptions(captureSchema(), settings, "capture", validateCapture);
}

reconstruction::settings::DocumentPtr makeCaptureSettingsDocument()
{
    return reconstruction::settings::makeDocument(captureSchema(), "capture", "拍摄", "capture.conf", validateCapture);
}
