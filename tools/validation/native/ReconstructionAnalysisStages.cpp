#include "ReconstructionContractData.h"
#include "io/FeatureCodec.h"

#include "common/Math3.h"
#include "preprocessing/detail/PreprocessingInternal.h"
#include "recognition/RepairInternal.h"
#include <reconstruction/recognition/GuidedSolidRecognizer.h>
#include <reconstruction/recognition/MixedFeatureRecognizer.h>
#include <reconstruction/recognition/ThroughHoleRecognizer.h>

#include <algorithm>
#include <limits>
#include <unordered_set>

namespace contract {
using namespace reconstruction;
namespace {
QJsonArray sampleRows(const std::vector<PointSample>& points) {
    QJsonArray rows;
    for (const auto& p : points)
        rows.append(QJsonArray{QString::number(p.id), p.normal ? QJsonValue(vector(*p.normal)) : QJsonValue(),
            p.normalOrientationKnown, p.curvature ? QJsonValue(*p.curvature) : QJsonValue()});
    return rows;
}

PointStore readWorking(const QJsonValue& value, const PointStore& source) {
    const auto index = pointIndex(source.points());
    std::vector<PointSample> samples;
    for (const auto& entry : value.toArray()) {
        const auto row = entry.toArray();
        require(row.size() == 4, "Invalid analysis observation row");
        const auto id = ids(QJsonArray{row[0]}).front();
        const auto found = index.find(id);
        require(found != index.end(), "Analysis observation ID is absent from source");
        auto p = *found->second;
        p.normal = row[1].isNull() ? std::optional<Vec3>{} : vector(row[1]);
        p.normalOrientationKnown = row[2].toBool();
        p.curvature = row[3].isDouble() ? std::optional<double>{row[3].toDouble()} : std::nullopt;
        samples.push_back(p);
    }
    require(!samples.empty(), "Analysis working observations are empty");
    return PointStore(std::move(samples), source.unit());
}

template<class Evidence>
QJsonObject evidenceFields(const Evidence& e) {
    return {{"id", QString::number(e.id)}, {"support_ids", ids(e.supportPointIds)},
        {"mean_residual", e.meanAbsoluteResidual}, {"max_residual", e.maxAbsoluteResidual},
        {"coverage", e.coverage}, {"confidence", e.confidence}};
}

QJsonArray writeEvidence(const std::vector<PrimitiveEvidence>& evidence) {
    QJsonArray result;
    for (const auto& item : evidence) {
        auto fields = std::visit([](const auto& e) { return evidenceFields(e); }, item);
        fields["type"] = static_cast<int>(primitiveKind(item));
        std::visit([&](const auto& e) {
            using E = std::decay_t<decltype(e)>;
            if constexpr (std::is_same_v<E, PlaneEvidence>) {
                fields["axis"] = vector(e.plane.normal);
                fields["offset"] = e.plane.offset;
            } else if constexpr (std::is_same_v<E, SphereEvidence>) {
                fields["center"] = vector(e.sphere.center);
                fields["radius"] = e.sphere.radius;
            } else if constexpr (std::is_same_v<E, TorusEvidence>) {
                fields["center"] = vector(e.torus.center);
                fields["axis"] = vector(e.torus.axisDirection);
                fields["radius"] = e.torus.majorRadius;
                fields["minor_radius"] = e.torus.minorRadius;
            } else {
                if constexpr (std::is_same_v<E, CylinderEvidence>) {
                    fields["center"] = vector(e.cylinder.axisPoint);
                    fields["axis"] = vector(e.cylinder.axisDirection);
                    fields["radius"] = e.cylinder.radius;
                } else {
                    fields["center"] = vector(e.cone.apex);
                    fields["axis"] = vector(e.cone.axisDirection);
                    fields["opening_angle"] = e.cone.openingAngleRadians;
                }
                fields["axial_min"] = e.axialMinimum;
                fields["axial_max"] = e.axialMaximum;
                fields["angular_coverage"] = e.angularCoverage;
            }
        }, item);
        result.append(fields);
    }
    return result;
}

std::vector<PrimitiveEvidence> readEvidence(const QJsonValue& value, const PointStore& working) {
    require(value.isArray(), "Missing primitive evidence");
    const auto points = pointIndex(working.points());
    std::vector<PrimitiveEvidence> result;
    for (const auto& entry : value.toArray()) {
        const auto o = entry.toObject();
        const auto radius = o["radius"].toDouble();
        PrimitiveEvidence item;
        switch (static_cast<PrimitiveKind>(o["type"].toInt(-1))) {
        case PrimitiveKind::Plane: {
            PlaneEvidence e;
            e.plane = {vector(o["axis"]), o["offset"].toDouble()};
            item = e;
            break;
        }
        case PrimitiveKind::Cylinder: {
            require(radius > 0, "Invalid cylinder radius");
            CylinderEvidence e;
            e.cylinder = {vector(o["center"]), vector(o["axis"]), radius};
            e.axialMinimum = o["axial_min"].toDouble();
            e.axialMaximum = o["axial_max"].toDouble();
            e.angularCoverage = o["angular_coverage"].toDouble();
            item = e;
            break;
        }
        case PrimitiveKind::Sphere: {
            require(radius > 0, "Invalid sphere radius");
            SphereEvidence e;
            e.sphere = {vector(o["center"]), radius};
            item = e;
            break;
        }
        case PrimitiveKind::Cone: {
            require(o["opening_angle"].toDouble() > 0, "Invalid cone opening");
            ConeEvidence e;
            e.cone = {vector(o["center"]), vector(o["axis"]), o["opening_angle"].toDouble()};
            e.axialMinimum = o["axial_min"].toDouble();
            e.axialMaximum = o["axial_max"].toDouble();
            e.angularCoverage = o["angular_coverage"].toDouble();
            item = e;
            break;
        }
        case PrimitiveKind::Torus: {
            require(radius > o["minor_radius"].toDouble() && o["minor_radius"].toDouble() > 0, "Invalid torus radii");
            TorusEvidence e;
            e.torus = {vector(o["center"]), vector(o["axis"]), radius, o["minor_radius"].toDouble()};
            item = e;
            break;
        }
        default:
            throw std::runtime_error("Unknown primitive type");
        }
        std::visit([&](auto& e) {
            e.id = o["id"].toString().toULongLong();
            e.supportPointIds = ids(o["support_ids"]);
            for (const auto id : e.supportPointIds) require(points.count(id), "Evidence references a point outside the working view");
            e.meanAbsoluteResidual = o["mean_residual"].toDouble();
            e.maxAbsoluteResidual = o["max_residual"].toDouble();
            e.coverage = o["coverage"].toDouble();
            e.confidence = o["confidence"].toDouble();
        }, item);
        result.push_back(std::move(item));
    }
    return result;
}

QJsonArray writePatches(const std::vector<SurfacePatch>& patches) {
    QJsonArray result;
    for (const auto& p : patches)
        result.append(QJsonObject{{"surface_id", qint64(p.surfaceId)}, {"support_ids", ids(p.supportPointIds)},
            {"lower", vector(p.lower)}, {"upper", vector(p.upper)}, {"centroid", vector(p.centroid)},
            {"normal", vector(p.materialNormal)}, {"oriented", p.orientationKnown}});
    return result;
}

QJsonArray writeRelations(const std::vector<SurfaceRelation>& relations) {
    QJsonArray result;
    for (const auto& r : relations)
        result.append(QJsonArray{qint64(r.first), qint64(r.second), static_cast<int>(r.kind),
            r.error, r.confidence, r.automatic});
    return result;
}

SurfaceAnalysisOptions analysisOptions(const QJsonObject& settings) {
    SurfaceAnalysisOptions options;
    auto& d = options.detection;
    const auto families = settings["families"].toArray();
    d.detectPlanes = families.isEmpty() || families.contains("plane");
    d.detectCylinders = families.contains("cylinder");
    d.detectSpheres = families.contains("sphere");
    d.detectCones = families.contains("cone");
    d.detectTori = families.contains("torus");
    const auto seed = settings["seed"].toInteger(4500);
    require(seed >= 0 && seed <= std::numeric_limits<std::uint32_t>::max(), "Invalid detection seed");
    d.randomSeed = static_cast<std::uint32_t>(seed);
    const double tolerance = settings["tolerance"].toDouble(.04);
    require(tolerance > 0, "Invalid analysis tolerance");
    d.plane.distanceThreshold = d.cylinder.distanceThreshold = d.sphere.distanceThreshold =
        d.cone.distanceThreshold = d.torus.distanceThreshold = tolerance;
    const auto support = settings["minimum_support"].toInt(25);
    require(support >= 3, "Invalid minimum support");
    d.plane.minimumSupportPoints = d.cylinder.minimumSupportPoints = d.sphere.minimumSupportPoints =
        d.cone.minimumSupportPoints = d.torus.minimumSupportPoints = static_cast<std::size_t>(support);
    d.plane.maximumPlanes = 16;
    options.connectivityDistance = settings["connectivity_mm"].toDouble(1.25);
    options.relations.distanceTolerance = settings["relation_distance_mm"].toDouble(tolerance * 4);
    options.recoverResidualSurfaces = settings["multiscale"].toBool();
    return options;
}

SurfaceAnalysisResult readAnalysis(const QJsonObject& input, const PointStore& working,
    const SurfaceAnalysisOptions& options) {
    SurfaceAnalysisResult analysis;
    analysis.pointFingerprint = pointStoreFingerprint(working);
    analysis.unit = working.unit();
    analysis.options = options;
    analysis.surfaces = readEvidence(input["evidence"], working);
    if (input["unassigned_ids"].isArray()) analysis.unassignedPointIds = ids(input["unassigned_ids"]);
    require(input["surface_patches"].isArray(), "Missing indexed surface patches");
    const auto index = pointIndex(working.points());
    for (const auto& entry : input["surface_patches"].toArray()) {
        const auto p = entry.toObject();
        const auto surface = p["surface_id"].toInteger();
        require(surface > 0 && static_cast<std::size_t>(surface) <= analysis.surfaces.size(), "Invalid patch surface reference");
        auto support = ids(p["support_ids"]);
        for (const auto id : support) require(index.count(id), "Patch observation is outside the working view");
        analysis.patches.push_back({analysis.patches.size() + 1, static_cast<SurfaceId>(surface),
            std::move(support), vector(p["lower"]), vector(p["upper"]), vector(p["centroid"]),
            vector(p["normal"]), p["oriented"].toBool()});
    }
    for (const auto& entry : input["surface_relations"].toArray()) {
        const auto r = entry.toArray();
        require(r.size() == 6 && r[2].toInt(-1) >= 0 &&
            r[2].toInt() <= static_cast<int>(SurfaceRelationKind::AxisParallelToPlane), "Invalid relation row");
        const auto a = r[0].toInteger(), b = r[1].toInteger();
        require(a > 0 && b > 0 && static_cast<std::size_t>(std::max(a, b)) <= analysis.patches.size(),
            "Invalid relation patch reference");
        analysis.relations.push_back({static_cast<PatchId>(a), static_cast<PatchId>(b),
            static_cast<SurfaceRelationKind>(r[2].toInt()), r[3].toDouble(), r[4].toDouble(), r[5].toBool()});
    }
    return analysis;
}

void saveAnalysis(QJsonObject& input, const SurfaceAnalysisResult& analysis) {
    input["evidence"] = writeEvidence(analysis.surfaces);
    input["surface_patches"] = writePatches(analysis.patches);
    input["surface_relations"] = writeRelations(analysis.relations);
    input["unassigned_ids"] = ids(analysis.unassignedPointIds);
}

template<class Function>
void checked(Function call) {
    std::string error;
    if (!call(error)) throw std::runtime_error(error);
}

std::vector<PointId> pointIds(const std::vector<PointSample>& points) {
    std::vector<PointId> values;
    for (const auto& p : points) values.push_back(p.id);
    return values;
}

const SurfacePatch& patchAt(const SurfaceAnalysisResult& analysis, Vec3 target) {
    require(!analysis.patches.empty(), "No patch near the requested test region");
    const auto distance = [&](const SurfacePatch& patch) {
        return math3::length(math3::subtract(patch.centroid, target));
    };
    return *std::min_element(analysis.patches.begin(), analysis.patches.end(),
        [&](const auto& a, const auto& b) { return distance(a) < distance(b); });
}


QJsonObject writeCandidate(const CompoundCandidate& candidate) {
    std::vector<modeling::Feature> primitives;
    auto append = [&](const modeling::FeatureParams& params) {
        modeling::Feature feature;
        feature.id = primitives.size() + 1;
        feature.type = modeling::featureTypeOf(params);
        feature.params = params;
        primitives.push_back(std::move(feature));
    };
    for (const auto& box : candidate.boxes) append(box.primitive);
    for (const auto& hole : candidate.throughHoles) append(hole.primitive);
    for (const auto& curved : candidate.curvedSolids)
        std::visit([&](const auto& params) { append(params); }, curved.primitive);
    QJsonArray prisms;
    for (const auto& prism : candidate.prisms) {
        QJsonArray polygon;
        for (const auto& point : prism.polygon) polygon.append(QJsonArray{point.x, point.y});
        prisms.append(QJsonObject{{"origin", vector(prism.plane.origin)}, {"normal", vector(prism.plane.normal)},
            {"x_axis", vector(prism.plane.xDirection)}, {"polygon", polygon},
            {"depth", prism.depth}, {"subtract", prism.subtract}});
    }
    return {{"primitives", FeatureCodec::encode(primitives)}, {"prisms", prisms}};
}

BoxCandidate readHost(const QJsonObject& input) {
    std::vector<modeling::Feature> features;
    QString error;
    require(FeatureCodec::decode(input["candidate"].toObject()["primitives"].toArray(), features, error) &&
        features.size() == 1 && std::holds_alternative<modeling::BoxPrimitiveParams>(features.front().params),
        "Through-hole recognition requires one upstream host box");
    BoxCandidate host;
    host.primitive = std::get<modeling::BoxPrimitiveParams>(features.front().params);
    return host;
}

QJsonObject recognizeStage(int module, const PointStore& working, const PointStore& source,
    const SurfaceAnalysisOptions& options, const QJsonObject& settings, QJsonObject input, QJsonObject& metrics) {
    CompoundCandidate candidate;
    std::string error;
    bool recognized = true;
    if (module == 15) {
        CompoundRecognitionOptions fitting;
        fitting.distanceTolerance = options.detection.plane.distanceThreshold;
        recognized = reconstructCompound(working, fitting, candidate, error);
        metrics["minimum_box_count_proven"] = candidate.minimumBoxCountProven;
    } else if (module == 17) {
        std::vector<TemplateHint> hints;
        if (input["template_hints"].isArray()) {
            for (const auto& entry : input["template_hints"].toArray()) {
                const auto hint = entry.toObject();
                const auto region = hint["region_id"].toInt();
                const auto kind = hint["kind"].toInt(-1);
                require(region > 0 && kind >= 0 && kind <= static_cast<int>(TemplateKind::Box),
                    "Invalid template hint");
                hints.push_back({region, static_cast<TemplateKind>(kind), ids(hint["point_ids"])});
            }
        } else {
            std::vector<PointId> selected;
            if (input["template_ids"].isArray()) {
                selected = ids(input["template_ids"]);
            } else {
                // The dataset declares a manual top-face selection in the
                // same frame used by the working observations.
                const auto z = settings["template_z_mm"].toDouble();
                for (const auto& point : working.points())
                    if (point.normal && point.normal->z > .95 && std::abs(point.position.z - z) < 1e-5)
                        selected.push_back(point.id);
            }
            input["template_ids"] = ids(selected);
            hints.push_back({settings["template_region_id"].toInt(1), TemplateKind::Plane, std::move(selected)});
        }
        require(!hints.empty(), "Guided contract requires explicit templates");
        GuidedSolidResult guided;
        recognized = reconstructGuidedSolid(working, options.detection.plane, options.detection.cylinder,
            hints, guided, error, {}, nullptr, {}, &source);
        metrics["template_fit_count"] = qint64(guided.fits.size());
        metrics["failed_region"] = guided.failedRegionId;
        std::size_t support = 0;
        for (const auto& hint : hints) support += hint.points.size();
        metrics["template_support"] = qint64(support);
        QJsonArray fits;
        for (const auto& fit : guided.fits) {
            QJsonObject item{{"region_id", fit.regionId}};
            if (fit.surface) item["evidence"] = writeEvidence({*fit.surface});
            fits.append(item);
        }
        input["template_fits"] = fits;
        candidate = std::move(guided.candidate);
    } else {
        const auto analysis = readAnalysis(input, working, options);
        if (module == 12) {
            BoxCandidate box;
            recognized = recognizeBoxFromAnalysis(working, analysis, {}, box, error);
            if (recognized) candidate.boxes.push_back(std::move(box));
        } else if (module == 13) {
            auto host = readHost(input);
            std::vector<CylinderEvidence> walls;
            for (const auto& item : analysis.surfaces)
                if (const auto* wall = std::get_if<CylinderEvidence>(&item)) walls.push_back(*wall);
            recognized = recognizeThroughHoles(host, walls, {}, candidate.throughHoles, error);
            if (recognized) candidate.boxes.push_back(std::move(host));
            metrics["wall_count"] = qint64(walls.size());
            if (candidate.throughHoles.size() == 1) {
                metrics["hole_radius_mm"] = candidate.throughHoles.front().primitive.radius;
                metrics["body_thickness_mm"] = candidate.throughHoles.front().bodyThickness;
            }
        } else if (module == 14) {
            CurvedSolidCandidate solid;
            CurvedSolidRecognitionOptions fitting;
            fitting.distanceTolerance = options.detection.plane.distanceThreshold;
            if (settings.contains("required_curved_kind")) {
                const auto kind = settings["required_curved_kind"].toInt(-1);
                require(kind >= static_cast<int>(PrimitiveKind::Cylinder) &&
                    kind <= static_cast<int>(PrimitiveKind::Torus), "Invalid required curved kind");
                fitting.requiredKind = static_cast<PrimitiveKind>(kind);
            }
            recognized = recognizeCurvedSolidFromAnalysis(working, analysis, fitting, solid, error);
            if (recognized) {
                if (const auto* cylinder = std::get_if<modeling::CylinderPrimitiveParams>(&solid.primitive)) {
                    metrics["radius_mm"] = cylinder->radius;
                    metrics["height_mm"] = cylinder->height;
                }
                metrics["boundary_coverage"] = solid.boundaryCoverage;
                candidate.surfaceCoverage = solid.surfaceCoverage;
                candidate.curvedSolids.push_back(std::move(solid));
            }
        } else {
            std::vector<LocalAnalysisFailure> failures;
            recognized = recognizeMixedFeatures(working, analysis, candidate, error, &failures, &source);
            metrics["local_failures"] = qint64(failures.size());
            QJsonArray diagnostics;
            for (const auto& failure : failures)
                diagnostics.append(QJsonObject{{"reason", localFailureReasonName(failure.reason)},
                    {"point_ids", ids(failure.pointIds)}, {"recovered_ids", ids(failure.recoveredPointIds)},
                    {"repaired", failure.repaired}});
            input["local_failures"] = diagnostics;
        }
    }
    metrics["recognized"] = recognized;
    metrics["box_count"] = qint64(candidate.boxes.size());
    metrics["hole_count"] = qint64(candidate.throughHoles.size());
    metrics["curved_count"] = qint64(candidate.curvedSolids.size());
    metrics["prism_count"] = qint64(candidate.prisms.size());
    metrics["surface_coverage"] = candidate.surfaceCoverage;
    if (recognized) {
        double boxVolume = 0;
        for (const auto& box : candidate.boxes)
            boxVolume += box.primitive.sizeX * box.primitive.sizeY * box.primitive.sizeZ;
        metrics["box_volume_mm3"] = boxVolume;
        input["candidate"] = writeCandidate(candidate);
    } else {
        input.remove("candidate");
        input["recognition_reason"] = QString::fromStdString(error);
    }
    return input;
}

} // namespace

QJsonObject analysisStage(int module, const ObservationSnapshot& snapshot,
    const QJsonObject& settings, QJsonObject input, QJsonObject& metrics, std::size_t skipped) {
    const auto& source = snapshot.observations();
    if (module == 1) {
        const PointStore selected(input.isEmpty() ? source.points() :
            observations(input["observation_ids"], pointIndex(source.points())), source.unit());
        require(!selected.empty(), "Empty observation snapshot");
        const auto& points = selected.points();
        const auto span = [&](auto axis) {
            const auto bounds = std::minmax_element(points.begin(), points.end(),
                [&](const auto& a, const auto& b) { return a.position.*axis < b.position.*axis; });
            return bounds.second->position.*axis - bounds.first->position.*axis;
        };
        metrics = {{"points", qint64(points.size())}, {"scale_to_mm", snapshot.sourceToWorkingScale()},
            {"skipped_rows", qint64(skipped)}, {"span_x_mm", span(&Vec3::x)}, {"span_y_mm", span(&Vec3::y)}};
        return {{"samples", sampleRows(points)}, {"point_ids", ids(pointIds(points))}};
    }
    const auto working = readWorking(input["samples"], source);
    if (module <= 4) {
        using namespace detail::preprocessing;
        const auto protect = ids(settings["protected_ids"].isArray() ? settings["protected_ids"] : QJsonValue(QJsonArray{}));
        const std::unordered_set<PointId> protectedIds(protect.begin(), protect.end());
        PointCloudPreprocessingOptions options;
        PointCloudPreprocessingReport report;
        options.voxelSize = settings["voxel_mm"].toDouble();
        options.removeStatisticalOutliers = settings["remove_outliers"].toBool(true);
        options.normalNeighborCount = static_cast<std::size_t>(std::max(3, settings["normal_neighbors"].toInt(18)));
        options.outlierStandardDeviationMultiplier = settings["outlier_sigma"].toDouble(2.5);
        std::string error;
        if (!validOptions(options, error)) throw std::runtime_error(error);
        auto points = working.points();
        if (module == 2) points = voxelDownsample(points, options.voxelSize, protectedIds);
        if (module == 3) points = removeOutliers(points, options, report, protectedIds);
        if (module == 4) estimateNormalsAndCurvature(points, options, report);
        require(!points.empty(), "Preprocessing produced no working points");
        const auto oriented = std::count_if(points.begin(), points.end(),
            [](const auto& p) { return p.normal && p.normalOrientationKnown; });
        metrics = {{"points", qint64(points.size())}, {"removed", qint64(working.size() - points.size())},
            {"normals_estimated", qint64(report.normalsEstimated)}, {"oriented", qint64(oriented)}};
        input["samples"] = sampleRows(points);
        input["point_ids"] = ids(pointIds(points));
        return input;
    }
    const auto options = analysisOptions(settings);
    if (module >= 12) return recognizeStage(module, working, source, options, settings, std::move(input), metrics);
    if (module == 5 || module == 6) {
        SurfaceEvidenceCollection result;
        if (module == 5) {
            PrimitiveDetectionResult detected;
            checked([&](auto& error) { return detectPrimitiveEvidence(working, options.detection, detected, error); });
            result = {std::move(detected.evidence), std::move(detected.unassignedPointIds), 1};
        } else {
            const auto seeds = readEvidence(input.value("seeds").isArray() ? input["seeds"] : QJsonValue(QJsonArray{}), working);
            checked([&](auto& error) { return collectSurfaceEvidence(working, options, result, error, seeds); });
        }
        input["evidence"] = writeEvidence(result.evidence);
        input["unassigned_ids"] = ids(result.unassignedPointIds);
        const auto cylinders = std::count_if(result.evidence.begin(), result.evidence.end(),
            [](const auto& item) { return std::holds_alternative<CylinderEvidence>(item); });
        metrics = {{"surface_count", qint64(result.evidence.size())}, {"cylinder_count", qint64(cylinders)},
            {"unassigned_count", qint64(result.unassignedPointIds.size())}, {"passes", qint64(result.detectionPasses)}};
        return input;
    }
    if (module == 7) {
        SurfaceAnalysisResult analysis;
        const auto evidence = readEvidence(input["evidence"], working);
        checked([&](auto& error) { return indexSurfacePatches(working, evidence, options, analysis, error); });
        saveAnalysis(input, analysis);
        metrics = {{"patch_count", qint64(analysis.patches.size())},
            {"unassigned_count", qint64(analysis.unassignedPointIds.size())}};
        return input;
    }
    auto analysis = readAnalysis(input, working, options);
    if (module == 8) {
        checked([&](auto& error) { return inferPatchRelations(working, analysis, analysis.relations, error); });
        const auto count = [&](SurfaceRelationKind kind) {
            return qint64(std::count_if(analysis.relations.begin(), analysis.relations.end(),
                [&](const auto& relation) { return relation.kind == kind; }));
        };
        metrics = {{"parallel_count", count(SurfaceRelationKind::Parallel)},
            {"perpendicular_count", count(SurfaceRelationKind::Perpendicular)}};
        if (const auto probes = settings["relation_probes"].toArray(); probes.size() == 2) {
            const auto& first = patchAt(analysis, vector(probes[0]));
            const auto& second = patchAt(analysis, vector(probes[1]));
            SurfaceRelationHint hint{first.supportPointIds, second.supportPointIds, SurfaceRelationKind::Perpendicular};
            std::string error;
            auto manual = analysis;
            metrics["manual_hint_accepted"] = applySurfaceRelationHints(working, {hint}, manual, error);
            hint.kind = SurfaceRelationKind::Parallel;
            metrics["conflicting_hint_rejected"] = !applySurfaceRelationHints(working, {hint}, manual, error);
        }
    } else if (module == 9) {
        RelationOptimizationReport report;
        const auto proposals = analysis.relations;
        checked([&](auto& error) { return optimizeSurfaceRelations(working, analysis, proposals, report, error); });
        metrics = {{"accepted_count", qint64(report.accepted.size())}, {"rejected_count", qint64(report.rejected.size())},
            {"before_error", report.beforeError}, {"after_error", report.afterError}};
    } else if (module == 10) {
        LocalAnalysisFailure failure;
        failure.pointIds = patchAt(analysis, vector(settings["recovery_probe"])).supportPointIds;
        PointStore local({});
        SurfaceAnalysisResult recovered;
        checked([&](auto& error) { return recoverLocalSurfaces(working, source, analysis, failure, local, recovered, error); });
        input["recovery"] = QJsonObject{{"region_ids", ids(failure.pointIds)}, {"samples", sampleRows(local.points())},
            {"added_ids", ids(failure.recoveredPointIds)}, {"evidence", writeEvidence(recovered.surfaces)}};
        metrics = {{"recovered_count", qint64(failure.recoveredPointIds.size())},
            {"local_surface_count", qint64(recovered.surfaces.size())}};
    } else {
        const auto recovery = input["recovery"].toObject();
        auto local = readWorking(recovery["samples"], source);
        LocalAnalysisFailure failure;
        failure.pointIds = ids(recovery["region_ids"]);
        failure.recoveredPointIds = ids(recovery["added_ids"]);
        failure.observations = local.points();
        failure.repairedCandidates = readEvidence(recovery["evidence"], local);
        failure.localFingerprint = pointStoreFingerprint(local);
        failure.repaired = true; // Submit the supplied proposal to merge and revalidation.
        const std::vector<LocalAnalysisFailure> proposals{std::move(failure)};
        const auto merged = detail::repair::collectRepairEvidence(working, analysis, proposals);
        SurfaceAnalysisResult combined;
        checked([&](auto& error) { return indexSurfaceEvidence(merged.observations, merged.evidence, options, combined, error); });
        CompoundCandidate baseline, checkedCandidate;
        checked([&](auto& error) { return recognizeParametricSolidFromAnalysis(working, analysis, baseline, error); });
        checked([&](auto& error) { return recognizeParametricSolidFromAnalysis(merged.observations, combined, checkedCandidate, error); });
        RepairTransactionReport report;
        const auto reason = detail::repair::validateRepairCandidate(working, options, baseline, checkedCandidate, proposals, report);
        metrics = {{"accepted", reason.empty()}, {"box_count", qint64(checkedCandidate.boxes.size())},
            {"added_count", qint64(merged.observations.size() - working.size())}, {"mapping_count", qint64(merged.mappings.size())},
            {"before_coverage", report.beforeCoverage}, {"after_coverage", report.afterCoverage}};
        input["validation_reason"] = QString::fromStdString(reason);
        if (reason.empty()) {
            input["samples"] = sampleRows(merged.observations.points());
            input["point_ids"] = ids(pointIds(merged.observations.points()));
            saveAnalysis(input, combined);
            QJsonArray mappings;
            for (const auto& mapping : merged.mappings)
                mappings.append(QJsonArray{QString::number(mapping.localFingerprint),
                    qint64(mapping.local), qint64(mapping.global)});
            input["repair_mappings"] = mappings;
        }
        return input;
    }
    saveAnalysis(input, analysis);
    return input;
}
} // namespace contract
