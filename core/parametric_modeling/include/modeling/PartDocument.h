#pragma once

#include "modeling/Feature.h"

#include <TopoDS_Shape.hxx>

#include <string>
#include <vector>

namespace modeling {

class RebuildEngine;

class PartDocument {
public:
    FeatureId addFeature(const FeatureParams& params);
    bool editFeature(FeatureId id, const FeatureParams& params);
    bool removeFeature(FeatureId id);
    bool setFeatureSuppressed(FeatureId id, bool suppressed);

    bool rebuild();

    const std::vector<Feature>& features() const noexcept;
    const TopoDS_Shape& bodyShape() const noexcept;
    const std::string& lastError() const noexcept;

    const Feature* findFeature(FeatureId id) const noexcept;

private:
    friend class RebuildEngine;

    std::vector<Feature> features_;
    TopoDS_Shape bodyShape_;
    FeatureId nextFeatureId_ = 1;
    std::string lastError_;
};

}  // namespace modeling
