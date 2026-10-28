#pragma once
#include "reconstruction/recognition/MixedFeatureRecognizer.h"
#include "reconstruction/recognition/ThroughHoleRecognizer.h"
#include "common/Math3.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace reconstruction::detail::mixed {
using namespace math3;
using modeling::Vec3;
struct Frame {
    Vec3 x,y,z;
    std::array<double,2> a{},b{},c{};
    Vec3 world(double u,double v,double w) const { return add(add(scale(x,u),scale(y,v)),scale(z,w)); }
};

double median(std::vector<double> values);

std::vector<PrimitiveEvidence> restrictEvidence(const SurfaceAnalysisResult& a,const std::vector<PointId>& group);

bool inBox(const modeling::BoxPrimitiveParams& b,Vec3 p);

bool inPrism(const PrismCandidate& prism,Vec3 p);

bool material(const CompoundCandidate& model,Vec3 p);

bool hostFrame(const PointStore& points,const SurfaceAnalysisResult& analysis,Frame& frame,BoxCandidate& host);

bool attachedCurved(const PointStore& local,const SurfaceAnalysisResult& a,const Frame& host,
                    double tolerance,CurvedSolidCandidate& output);

bool rectangularCut(const PointStore& local,const SurfaceAnalysisResult& a,const Frame& host,
                    double tolerance,PrismCandidate& cut);
} // namespace reconstruction::detail::mixed
