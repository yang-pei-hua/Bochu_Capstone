#include "app/ProjectionDebugReport.h"
#include "app/CadReconstructionTask.h"

#include <QJsonArray>

namespace {
QJsonArray coordinates(modeling::Vec3 value)
{
    return {value.x, value.y, value.z};
}

QJsonObject qualityReport(const reconstruction::ProjectionFitQuality& quality)
{
    return {{"scored_points", qint64(quality.scoredPoints)},
            {"median_distance_mm", quality.medianDistance}, {"p95_distance_mm", quality.p95Distance},
            {"surface_inlier_fraction", quality.surfaceInlierFraction},
            {"unsupported_face_area_fraction", quality.unsupportedFaceAreaFraction},
            {"mean_normal_error_degrees", quality.meanNormalErrorDegrees},
            {"normal_comparisons", qint64(quality.normalComparisons)}, {"score", quality.score}};
}

QString operationName(modeling::ExtrudeOperation operation)
{
    using Operation = modeling::ExtrudeOperation;
    switch (operation) {
    case Operation::NewBody: return QStringLiteral("new_body");
    case Operation::Join: return QStringLiteral("join");
    case Operation::Cut: return QStringLiteral("cut");
    case Operation::Intersect: return QStringLiteral("intersect");
    }
    return {};
}

QString pathName(reconstruction::ProjectionPath path)
{
    using Path = reconstruction::ProjectionPath;
    switch (path) {
    case Path::Automatic: return QStringLiteral("automatic");
    case Path::SingleAxis: return QStringLiteral("single_axis");
    case Path::TwoAxisIntersection: return QStringLiteral("two_axis_intersection");
    case Path::Segmented: return QStringLiteral("segmented");
    }
    return {};
}
}

QJsonObject projectionDebugReport(const QString& sourcePath, const CadReconstructionResult& result)
{
    using Outcome = CadReconstructionResult::Outcome;
    const auto& debug = result.projectionDebug.value();
    const auto& fit = debug.fit;
    const bool accepted = result.outcome == Outcome::Success;
    QString status = QStringLiteral("failed");
    if (accepted) status = QStringLiteral("accepted");
    else if (result.outcome == Outcome::Unsupported) status = QStringLiteral("unsupported");
    else if (result.outcome == Outcome::Cancelled) status = QStringLiteral("cancelled");
    using Unit = reconstruction::LengthUnit;
    const auto sourceUnit = debug.sourceUnit == Unit::Meter ? "m" :
        debug.sourceUnit == Unit::Millimeter ? "mm" : "arbitrary";
    QJsonObject report{{"schema", "paramcad.projection-debug"}, {"version", 1},
        {"status", status}, {"source_cloud", sourcePath}, {"requested_path", pathName(debug.path)},
        {"source_unit", sourceUnit},
        {"input_points", qint64(debug.inputPoints)}, {"valid_points", qint64(debug.validPoints)},
        {"elapsed_ms", debug.elapsedMilliseconds}, {"distance_tolerance_mm", fit.distanceTolerance}};
    if (!result.error.isEmpty()) report["reason"] = result.error;
    if (fit.quality.scoredPoints) report["quality"] = qualityReport(fit.quality);

    QJsonArray axes;
    for (const auto& axis : fit.axes) {
        QJsonObject entry{{"direction", coordinates(axis.axis)}, {"source", QString::fromStdString(axis.source)},
            {"status", QString::fromStdString(axis.status)}, {"convex_projection_area_mm2", axis.convexProjectionArea},
            {"convex_envelope_volume_mm3", axis.convexEnvelopeVolume}};
        if (axis.bestBaseQuality) entry["best_base_quality"] = qualityReport(*axis.bestBaseQuality);
        axes.append(entry);
    }
    report["axes"] = axes;
    if (fit.bestAxisIndex) {
        report["best_axis_index"] = qint64(*fit.bestAxisIndex);
        if (accepted) report["selected_axis"] = axes.at(qsizetype(*fit.bestAxisIndex));
    }
    QJsonArray operations;
    const auto& program = accepted ? fit.operations : fit.rejectedOperations;
    for (const auto& operation : program) {
        QJsonObject entry{{"operation", operationName(operation.operation)}, {"depth_mm", operation.depth},
            {"reverse", operation.reverse}, {"curve_count", qint64(operation.sketch.entities.size())},
            {"support_point_count", qint64(operation.supportPointIds.size())}};
        if (const auto* plane = std::get_if<modeling::Plane3d>(&operation.sketch.plane)) {
            entry["origin_mm"] = coordinates(plane->origin);
            entry["axis"] = coordinates(plane->normal);
            entry["x_direction"] = coordinates(plane->xDirection);
        }
        operations.append(entry);
    }
    report[accepted ? "operations" : "preview_operations"] = operations;
    QJsonArray residuals;
    for (const auto id : fit.residualPointIds) residuals.append(qint64(id));
    report["sampled_residual_point_ids"] = residuals;
    return report;
}
