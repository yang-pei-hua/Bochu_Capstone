#include "ReconstructionContractData.h"

#include "io/FeatureCodec.h"
#include "recognition/ProjectionInternal.h"

#include <algorithm>

namespace contract {
using namespace reconstruction;
using namespace reconstruction::detail::projection;
namespace {
QJsonArray writeSamples(const std::vector<PointSample>& points) {
    QJsonArray result;
    for (const auto& p : points)
        result.append(QJsonArray{QString::number(p.id), p.normal ? QJsonValue(vector(*p.normal)) : QJsonValue(),
            p.normalOrientationKnown});
    return result;
}

std::vector<PointSample> readSamples(const QJsonValue& value, const PointIndex& source) {
    std::vector<PointSample> result;
    for (const auto& entry : value.toArray()) {
        const auto row = entry.toArray();
        require(row.size() == 3, "Invalid projection observation row");
        const auto id = ids(QJsonArray{row[0]}).front();
        const auto found = source.find(id);
        require(found != source.end(), "Unknown projection point ID");
        auto p = *found->second;
        p.normal = row[1].isNull() ? std::optional<Vec3>{} : vector(row[1]);
        p.normalOrientationKnown = row[2].toBool();
        result.push_back(p);
    }
    require(!result.empty(), "Empty projection observation view");
    return result;
}

QJsonArray writeEvidence(const std::vector<PrimitiveEvidence>& evidence) {
    QJsonArray result;
    for (const auto& entry : evidence) {
        if (const auto* p = std::get_if<PlaneEvidence>(&entry))
            result.append(QJsonObject{{"type", "plane"}, {"normal", vector(p->plane.normal)}, {"offset", p->plane.offset}});
        else if (const auto* c = std::get_if<CylinderEvidence>(&entry))
            result.append(QJsonObject{{"type", "cylinder"}, {"axis_point", vector(c->cylinder.axisPoint)},
                {"axis", vector(c->cylinder.axisDirection)}, {"radius", c->cylinder.radius},
                {"axial_min", c->axialMinimum}, {"axial_max", c->axialMaximum},
                {"angular_coverage", c->angularCoverage}, {"support_ids", ids(c->supportPointIds)}});
    }
    return result;
}

std::vector<PrimitiveEvidence> readEvidence(const QJsonValue& value) {
    std::vector<PrimitiveEvidence> result;
    for (const auto& entry : value.toArray()) {
        const auto item = entry.toObject();
        if (item["type"] == "plane") {
            PlaneEvidence plane;
            plane.plane = {vector(item["normal"]), item["offset"].toDouble()};
            result.emplace_back(plane);
        } else {
            require(item["type"] == "cylinder" && item["radius"].toDouble() > 0, "Invalid projection evidence");
            CylinderEvidence cylinder;
            cylinder.cylinder = {vector(item["axis_point"]), vector(item["axis"]), item["radius"].toDouble()};
            cylinder.axialMinimum = item["axial_min"].toDouble();
            cylinder.axialMaximum = item["axial_max"].toDouble();
            cylinder.angularCoverage = item["angular_coverage"].toDouble();
            cylinder.supportPointIds = ids(item["support_ids"]);
            result.emplace_back(std::move(cylinder));
        }
    }
    return result;
}

QJsonArray writeSketch(const modeling::SketchFeatureParams& sketch) {
    modeling::Feature feature;
    feature.id = 1;
    feature.params = sketch;
    return FeatureCodec::encode({feature});
}

modeling::SketchFeatureParams readSketch(const QJsonValue& value) {
    std::vector<modeling::Feature> features;
    QString error;
    require(FeatureCodec::decode(value.toArray(), features, error) && features.size() == 1 &&
        std::holds_alternative<modeling::SketchFeatureParams>(features.front().params), "Invalid projection sketch");
    return std::get<modeling::SketchFeatureParams>(std::move(features.front().params));
}

QJsonObject writeOperation(const ProjectionOperation& op) {
    return {{"sketch", writeSketch(op.sketch)}, {"operation", static_cast<int>(op.operation)},
        {"depth", op.depth}, {"reverse", op.reverse}, {"support_ids", ids(op.supportPointIds)}};
}

std::vector<ProjectionOperation> readOperations(const QJsonValue& value) {
    require(value.isArray(), "Expected projection operations");
    std::vector<ProjectionOperation> result;
    for (const auto& entry : value.toArray()) {
        const auto item = entry.toObject();
        const auto kind = item["operation"].toInt(-1);
        require(kind >= 0 && kind <= 2 && item["depth"].toDouble() > 0, "Invalid projection operation");
        result.push_back({readSketch(item["sketch"]), static_cast<modeling::ExtrudeOperation>(kind),
            item["depth"].toDouble(), item["reverse"].toBool(), ids(item["support_ids"])});
    }
    return result;
}

QJsonObject writeFit(const Fit& fit) {
    const auto& q = fit.quality;
    return {{"volume_mm3", fit.volume}, {"scored_points", qint64(q.scoredPoints)},
        {"median_mm", q.medianDistance}, {"p95_mm", q.p95Distance}, {"inlier_fraction", q.surfaceInlierFraction},
        {"unsupported_fraction", q.unsupportedFaceAreaFraction}, {"normal_error_deg", q.meanNormalErrorDegrees},
        {"normal_comparisons", qint64(q.normalComparisons)}, {"score", q.score}, {"residual_ids", ids(fit.residuals)}};
}

Fit readFit(const QJsonObject& item) {
    for (const auto key : {"volume_mm3", "scored_points", "median_mm", "p95_mm", "inlier_fraction",
            "unsupported_fraction", "normal_error_deg", "normal_comparisons", "score"})
        require(item[key].isDouble(), "Missing trial score field");
    Fit fit;
    fit.volume = item["volume_mm3"].toDouble();
    fit.quality = {static_cast<std::size_t>(item["scored_points"].toInteger()), item["median_mm"].toDouble(),
        item["p95_mm"].toDouble(), item["inlier_fraction"].toDouble(), item["unsupported_fraction"].toDouble(),
        item["normal_error_deg"].toDouble(), static_cast<std::size_t>(item["normal_comparisons"].toInteger()),
        item["score"].toDouble()};
    fit.residuals = ids(item["residual_ids"]);
    return fit;
}

QJsonObject writeCap(const Cap& cap) {
    std::vector<PointId> support;
    for (const auto& point : cap.points) support.push_back(point.id);
    return {{"depth", cap.depth}, {"ids", ids(support)}};
}

Cap readCap(const QJsonValue& value, const PointIndex& index) {
    const auto item = value.toObject();
    require(item["depth"].isDouble(), "Invalid cap depth");
    auto points = observations(item["ids"], index);
    require(points.size() >= minimumCapPoints, "Cap lacks support");
    return {item["depth"].toDouble(), std::move(points)};
}

std::optional<double> linearProfileArea(const std::vector<modeling::SketchEntity>& entities) {
    double area = 0;
    for (const auto& entity : entities) {
        const auto* line = std::get_if<modeling::Line2D>(&entity.geometry);
        if (!line) return {};
        area += .5 * (line->x1 * line->y2 - line->x2 * line->y1);
    }
    return std::abs(area);
}
} // namespace

std::vector<ProjectionOperation> readProjectionProgram(const QJsonValue& value) {
    return readOperations(value);
}

QJsonObject projectionStage(int module, const PointStore& source, const QJsonObject& settings,
    QJsonObject input, QJsonObject& metrics) {
    ProjectionExtrusionOptions options;
    options.distanceTolerance = settings["tolerance"].toDouble(.05);
    const auto maxOperations = settings["maximum_operations"].toInt(3);
    require(maxOperations > 0 && maxOperations <= 12, "Invalid projection operation budget");
    options.maximumOperations = static_cast<std::size_t>(maxOperations);
    if (module == 22) {
        ProjectionWork work;
        std::string error;
        if (input.isEmpty()) {
            if (!prepareProjection(source, options, {}, work, error)) throw std::runtime_error(error);
        } else {
            const auto index = pointIndex(source.points());
            work = {readSamples(input["fit_points"], index), readSamples(input["scoring_points"], index),
                readEvidence(input["evidence"]), vector(input["center"]), input["tolerance"].toDouble()};
            require(work.tolerance > 0, "Missing prepared projection tolerance");
        }
        std::vector<ProjectionAxisReport> axes;
        require(generateAxes(work.points, work.scoringPoints, work.evidence, work.center, options, axes, {}) &&
            !axes.empty(), "No projection axes");
        QJsonArray values;
        double maximumZ = 0;
        for (const auto& axis : axes) {
            values.append(QJsonObject{{"axis", vector(axis.axis)}, {"origin", vector(work.center)}, {"source", QString::fromStdString(axis.source)}});
            maximumZ = std::max(maximumZ, std::abs(axis.axis.z));
        }
        metrics = {{"axis_count", qint64(axes.size())}, {"maximum_abs_z", maximumZ}};
        return {{"axes", values}, {"fit_points", writeSamples(work.points)},
            {"scoring_points", writeSamples(work.scoringPoints)}, {"evidence", writeEvidence(work.evidence)}, {"tolerance", work.tolerance}};
    }
    const auto sourceIndex = pointIndex(source.points());
    const double tolerance = input["tolerance"].toDouble();
    require(tolerance > 0, "Missing projection tolerance");
    if (module == 26 || module == 27) {
        const auto scoring = readSamples(input["scoring_points"], sourceIndex);
        const ProgramEvaluator evaluate = [&](const std::vector<ProjectionOperation>& operations) {
            return trialProgram(operations, scoring, tolerance);
        };
        if (module == 26) {
            QJsonArray programs, operations = input["secondary_operations"].toArray();
            std::optional<Fit> best;
            for (const auto& item : input["proposals"].toArray()) {
                const auto proposal = item.toObject();
                auto base = readOperations(QJsonArray{proposal["operation"]});
                if (base.front().operation == modeling::ExtrudeOperation::Cut) {
                    operations.append(writeOperation(base.front()));
                    continue;
                }
                if (const auto fit = evaluate(base)) {
                    programs.append(QJsonObject{{"operations", QJsonArray{writeOperation(base.front())}}, {"fit", writeFit(*fit)}});
                    if (!best || fit->quality.score < best->quality.score) best = fit;
                    if (proposal["allow_join"].toBool()) {
                        base.front().operation = modeling::ExtrudeOperation::Join;
                        operations.append(writeOperation(base.front()));
                    }
                }
            }
            require(best.has_value(), "No valid base trial");
            metrics = writeFit(*best);
            metrics["valid_program_count"] = programs.size();
            input["programs"] = programs;
            input["secondary_operations"] = operations;
        } else {
            std::vector<Program> programs;
            for (const auto& item : input["programs"].toArray()) {
                auto operations = readOperations(item.toObject()["operations"]);
                require(!operations.empty(), "Trial has no operations");
                programs.push_back({std::move(operations), readFit(item.toObject()["fit"].toObject())});
            }
            const auto result = searchPrograms(std::move(programs), readOperations(input["secondary_operations"]),
                options.maximumOperations, tolerance, evaluate);
            metrics = writeFit(result.best.fit);
            metrics["accepted"] = result.status == SearchStatus::Accepted;
            metrics["operation_count"] = qint64(result.best.operations.size());
            QJsonArray selected;
            if (result.status == SearchStatus::Accepted)
                for (const auto& operation : result.best.operations) selected.append(writeOperation(operation));
            input["selected_operations"] = selected;
        }
        return input;
    }
    const auto points = readSamples(input["fit_points"], sourceIndex);
    const auto axes = input["axes"].toArray();
    const auto axisIndex = settings["axis_index"].toInt(0);
    require(axisIndex >= 0 && axisIndex < axes.size(), "Projection axis index out of range");
    const auto selected = (module == 23 ? axes[axisIndex] : input["selected_axis"]).toObject();
    const auto axis = vector(selected["axis"]);
    require(std::abs(math3::length(axis) - 1) < 1e-5, "Expected a unit projection axis");
    const auto frame = makeFrame(vector(selected["origin"]), axis);
    if (module == 23) {
        const auto interval = inspectInterval(points, frame, tolerance);
        require(interval.first && interval.last && interval.last->depth - interval.first->depth >= 3 * tolerance,
            "Projection interval lacks supported ends");
        metrics = {{"depth_mm", interval.last->depth - interval.first->depth}, {"abs_z", std::abs(axis.z)},
            {"first_support", qint64(interval.first->points.size())}, {"last_support", qint64(interval.last->points.size())}};
        input["selected_axis"] = selected;
        input["first"] = writeCap(*interval.first);
        input["last"] = writeCap(*interval.last);
        return input;
    }
    const auto index = pointIndex(points);
    const auto first = readCap(input["first"], index);
    const auto last = readCap(input["last"], index);
    require(last.depth > first.depth, "Reversed projection interval");
    const auto evidence = readEvidence(input["evidence"]);
    if (module == 24) {
        QJsonArray profiles;
        for (const auto* cap : {&first, &last}) {
            auto entities = recoverProfile(*cap, frame, tolerance, evidence, first, last, points);
            require(!entities.empty(), "Empty recovered profile");
            profiles.append(writeSketch({frame.plane(first.depth), std::move(entities)}));
        }
        if (const auto area = linearProfileArea(readSketch(profiles[0]).entities))
            metrics["linear_profile_area_mm2"] = *area;
        input["profiles"] = profiles;
    } else {
        const auto profiles = input["profiles"].toArray();
        require(profiles.size() == 2, "Expected two end profiles");
        const EndProfiles ends{readSketch(profiles[0]).entities, readSketch(profiles[1]).entities};
        std::vector<AxisProposal> proposals;
        require(proposeAxisOperations(points, evidence, frame, first, last, tolerance,
            options.maximumOperations > 1, proposals, {}, &ends), "Proposal generation failed");
        QJsonArray ordered, secondary;
        qsizetype baseCount = 0, cutCount = 0;
        if (options.maximumOperations > 1)
            for (const auto& operation : blindHoles(points, evidence, tolerance))
                secondary.append(writeOperation(operation));
        for (const auto& p : proposals) {
            ordered.append(QJsonObject{{"operation", writeOperation(p.operation)}, {"allow_join", p.allowJoin}});
            if (p.operation.operation == modeling::ExtrudeOperation::Cut) ++cutCount;
            else ++baseCount;
        }
        require(baseCount > 0, "No base proposals");
        metrics = {{"base_count", baseCount}, {"secondary_count", secondary.size() + cutCount}};
        input["proposals"] = ordered;
        input["secondary_operations"] = secondary;
    }
    return input;
}
} // namespace contract
