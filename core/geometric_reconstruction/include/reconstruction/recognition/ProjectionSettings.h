#pragma once

#include "reconstruction/recognition/ProjectionExtrusion.h"
#include "reconstruction/settings/ModuleSettings.h"

#include <filesystem>
#include <string_view>
#include <vector>

namespace reconstruction {

using ProjectionSetting = settings::Setting<ProjectionExtrusionOptions>;

const std::vector<ProjectionSetting>& projectionSettings();
std::filesystem::path projectionConfigPath();
ProjectionExtrusionOptions loadProjectionOptions(const std::filesystem::path& path = {});
std::string serializeProjectionOptions(const ProjectionExtrusionOptions& options);
void setProjectionOption(ProjectionExtrusionOptions& options, std::string_view assignment);
void validateProjectionOptions(const ProjectionExtrusionOptions& options);
settings::DocumentPtr makeProjectionSettings();

}  // namespace reconstruction
