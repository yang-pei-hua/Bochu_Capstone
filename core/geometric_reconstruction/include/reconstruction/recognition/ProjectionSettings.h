#pragma once

#include "reconstruction/recognition/ProjectionExtrusion.h"

#include <filesystem>
#include <string_view>
#include <vector>

namespace reconstruction {

struct ProjectionSetting {
    enum class Type { Integer, Real, Boolean };
    const char* key;
    const char* group;
    const char* label;
    Type type;
    double minimum, maximum;
    double (*value)(const ProjectionExtrusionOptions&);
    void (*assign)(ProjectionExtrusionOptions&, double);
};

const std::vector<ProjectionSetting>& projectionSettings();
std::filesystem::path projectionConfigPath();
ProjectionExtrusionOptions loadProjectionOptions(const std::filesystem::path& path = {});
std::string serializeProjectionOptions(const ProjectionExtrusionOptions& options);
void setProjectionOption(ProjectionExtrusionOptions& options, std::string_view assignment);
void validateProjectionOptions(const ProjectionExtrusionOptions& options);

} // namespace reconstruction
