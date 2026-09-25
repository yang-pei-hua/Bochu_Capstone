#pragma once

#include <TopoDS_Shape.hxx>

#include <filesystem>
#include <string>

namespace modeling {

bool exportStep(
    const TopoDS_Shape& shape,
    const std::filesystem::path& outputPath,
    std::string* errorText = nullptr);

}  // namespace modeling
