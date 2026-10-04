#pragma once

#include <TopoDS_Shape.hxx>

#include <filesystem>
#include <iosfwd>
#include <string>

namespace modeling {

bool exportStep(
    const TopoDS_Shape& shape,
    const std::filesystem::path& outputPath,
    std::string* errorText = nullptr);

// Allows clients to supply their own Unicode-aware/atomic file sink.
bool exportStep(
    const TopoDS_Shape& shape,
    std::ostream& output,
    std::string* errorText = nullptr);

}  // namespace modeling
