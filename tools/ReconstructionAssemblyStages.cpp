#include "ReconstructionContractData.h"

#include "preprocessing/detail/KdTree.h"
#include "recognition/assembly/AssemblyInternal.h"
#include <reconstruction/modeling/ModelingAdapter.h>

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <algorithm>
#include <limits>

namespace contract {
using namespace reconstruction;
using namespace reconstruction::detail::assembly;
namespace {
QJsonArray loopsJson(const std::vector<std::vector<Vec3>>& loops) {
    QJsonArray result;
    for (const auto& loop : loops) {
        QJsonArray vertices;
        for (const auto& v : loop) vertices.append(vector(v));
        result.append(vertices);
    }
    return result;
}

std::vector<std::vector<Vec3>> readLoops(const QJsonValue& value) {
    std::vector<std::vector<Vec3>> loops;
    for (const auto& item : value.toArray()) {
        std::vector<Vec3> loop;
        for (const auto& vertex : item.toArray()) loop.push_back(vector(vertex));
        require(loop.size() >= 3, "Boundary loop has fewer than three vertices");
        loops.push_back(std::move(loop));
    }
    return loops;
}

QJsonArray writePatches(const std::vector<FiniteSurfacePatch>& patches) {
    QJsonArray result;
    for (const auto& p : patches)
        result.append(QJsonObject{{"origin", vector(p.origin)}, {"normal", vector(p.normal)},
            {"x", vector(p.xDirection)}, {"lower", vector(p.lower)}, {"upper", vector(p.upper)},
            {"spacing", p.spacing}, {"residual", p.residual}, {"planar", p.planar},
            {"oriented", p.orientationKnown}, {"working_ids", ids(p.workingPointIds)},
            {"source_ids", ids(p.pointIds)}, {"loops", loopsJson(p.loops)},
            {"snapped", qint64(p.snappedVertices)}});
    return result;
}

PatchExtraction readExtraction(const QJsonObject& input, const PointStore& source) {
    PatchExtraction result;
    auto& work = result.work;
    work.tolerance = input["tolerance"].toDouble();
    require(work.tolerance > 0, "Missing assembly working tolerance");
    const auto index = pointIndex(source.points());
    std::unordered_map<PointId, std::size_t> indices;
    for (const auto& value : input["work"].toArray()) {
        const auto row = value.toArray();
        require(row.size() == 4, "Expected a working observation row");
        const auto id = ids(QJsonArray{row[0]}).front();
        const auto found = index.find(id);
        require(found != index.end() && indices.emplace(id, work.points.size()).second, "Invalid working observation ID");
        auto point = *found->second;
        point.normal = row[1].isNull() ? std::optional<Vec3>{} : vector(row[1]);
        point.normalOrientationKnown = row[2].toBool();
        work.points.push_back(point);
        require(row[3].toDouble() > 0, "Missing working observation spacing");
        work.spacing.push_back(row[3].toDouble());
    }
    require(!work.points.empty(), "Empty working observations");
    work.labels.resize(work.points.size(), -1);
    for (const auto& value : input["patches"].toArray()) {
        const auto item = value.toObject();
        FiniteSurfacePatch patch;
        patch.id = result.patches.size() + 1;
        patch.origin = vector(item["origin"]);
        patch.normal = vector(item["normal"]);
        patch.xDirection = vector(item["x"]);
        patch.lower = vector(item["lower"]);
        patch.upper = vector(item["upper"]);
        patch.spacing = item["spacing"].toDouble();
        patch.residual = item["residual"].toDouble();
        patch.planar = item["planar"].toBool();
        patch.orientationKnown = item["oriented"].toBool();
        patch.loops = readLoops(item["loops"]);
        patch.snappedVertices = item["snapped"].toInteger();
        patch.pointIds = ids(item["source_ids"]);
        for (const auto id : patch.pointIds) require(index.count(id), "Invalid full-resolution patch ID");
        patch.workingPointIds = ids(item["working_ids"]);
        require(!patch.workingPointIds.empty() && patch.spacing > 0, "Patch needs support and spacing");
        std::vector<std::size_t> group;
        for (const auto id : patch.workingPointIds) {
            const auto found = indices.find(id);
            require(found != indices.end() && work.labels[found->second] == -1, "Invalid patch working membership");
            require(work.points[found->second].normal.has_value(), "Patch support is missing its normal");
            work.labels[found->second] = static_cast<int>(result.patches.size());
            group.push_back(found->second);
        }
        work.groups.push_back(std::move(group));
        result.patches.push_back(std::move(patch));
    }
    require(!result.patches.empty(), "Empty patch extraction");
    return result;
}

std::size_t patchId(const QJsonValue& value, std::size_t count) {
    const auto id = value.toInteger();
    require(id > 0 && static_cast<std::size_t>(id) <= count, "Invalid patch reference");
    return static_cast<std::size_t>(id);
}

PatchGrouping readGrouping(const QJsonObject& input, std::size_t patchCount, const PointIndex& index) {
    PatchGrouping result;
    result.patchParts.resize(patchCount, std::numeric_limits<std::size_t>::max());
    for (const auto& value : input["connections"].toArray()) {
        const auto a = value.toArray();
        require(a.size() == 7, "Invalid patch connection");
        result.connections.push_back({patchId(a[0], patchCount), patchId(a[1], patchCount),
            a[2].toBool(), static_cast<std::size_t>(a[3].toInteger()), a[4].toDouble(), a[5].toBool(), a[6].toBool()});
    }
    for (const auto& value : input["parts"].toArray()) {
        const auto o = value.toObject();
        AssemblyPart part;
        part.id = result.parts.size() + 1;
        part.orientationKnown = o["oriented"].toBool();
        part.lower = vector(o["lower"]);
        part.upper = vector(o["upper"]);
        part.workingPointIds = ids(o["working_ids"]);
        part.pointIds = ids(o["source_ids"]);
        for (const auto id : part.pointIds) require(index.count(id), "Invalid full-resolution part ID");
        for (const auto& item : o["patch_ids"].toArray()) {
            const auto id = patchId(item, patchCount);
            require(result.patchParts[id - 1] >= patchCount, "Patch belongs to multiple parts");
            result.patchParts[id - 1] = part.id - 1;
            part.patchIds.push_back(id);
        }
        require(!part.patchIds.empty(), "Empty part");
        result.parts.push_back(std::move(part));
    }
    require(std::all_of(result.patchParts.begin(), result.patchParts.end(),
        [&](auto part) { return part < result.parts.size(); }), "Part assignment omits patches");
    return result;
}

void writeGrouping(QJsonObject& output, const PatchGrouping& grouping) {
    QJsonArray parts, connections;
    for (const auto& c : grouping.connections)
        connections.append(QJsonArray{qint64(c.first), qint64(c.second), c.convex, qint64(c.observations),
            c.distance, c.accepted, c.orientationKnown});
    for (const auto& p : grouping.parts) {
        QJsonArray patchIds;
        for (const auto id : p.patchIds) patchIds.append(qint64(id));
        parts.append(QJsonObject{{"patch_ids", patchIds}, {"oriented", p.orientationKnown},
            {"lower", vector(p.lower)}, {"upper", vector(p.upper)}, {"working_ids", ids(p.workingPointIds)},
            {"source_ids", ids(p.pointIds)}, {"closed", p.closed}, {"status", QString::fromStdString(p.status)},
            {"open_edges", qint64(p.openEdges)}, {"nonmanifold_edges", qint64(p.nonmanifoldEdges)}});
    }
    output["connections"] = connections;
    output["parts"] = parts;
}
} // namespace

QJsonObject assemblyStage(int module, const PointStore& source, const QJsonObject& settings,
    QJsonObject input, QJsonObject& metrics, bool backfill) {
    AssemblyOptions options;
    options.distanceTolerance = settings["tolerance"].toDouble(.02);
    const auto maximum = settings["maximum_working_points"].toInteger(options.maximumWorkingPoints);
    require(maximum >= static_cast<qint64>(options.minimumPatchPoints), "Invalid working point budget");
    options.maximumWorkingPoints = static_cast<std::size_t>(maximum);
    auto extraction = [&] {
        if (module != 18) return readExtraction(input, source);
        if (input.isEmpty()) return extractPatches(source, options, {});
        const auto selected = observations(input["observation_ids"], pointIndex(source.points()));
        return extractPatches(PointStore(selected, source.unit()), options, {});
    }();
    auto& [work, patches, unassigned] = extraction;
    require(!patches.empty(), "No observed patches");
    metrics["patch_count"] = qint64(patches.size());
    if (module == 18) {
        QJsonArray rows;
        for (std::size_t i = 0; i < work.points.size(); ++i) {
            const auto& p = work.points[i];
            rows.append(QJsonArray{QString::number(p.id), p.normal ? QJsonValue(vector(*p.normal)) : QJsonValue(),
                p.normalOrientationKnown, work.spacing[i]});
        }
        input["work"] = rows;
        input["tolerance"] = work.tolerance;
        input["unassigned_working_ids"] = ids(unassigned);
        metrics["working_count"] = qint64(work.points.size());
    } else {
        PatchGrouping grouping;
        if (module == 19 && !backfill) {
            detail::preprocessing::KdTree tree(work.points);
            for (std::size_t i = 0; i < work.points.size(); ++i)
                work.neighbors.push_back(tree.nearest(i, std::min(options.neighbors, work.points.size() - 1)));
            grouping = groupPatches(work, patches, options);
        } else {
            grouping = readGrouping(input, patches.size(), pointIndex(source.points()));
        }
        metrics["part_count"] = qint64(grouping.parts.size());
        if (module == 20) {
            require(std::all_of(patches.begin(), patches.end(), [](const auto& patch) { return patch.loops.empty(); }),
                "Grouping input already contains recovered boundaries");
            recoverBoundaries(work, options, grouping.connections, patches, {});
            std::size_t loops = 0;
            for (const auto& patch : patches) loops += patch.loops.size();
            metrics["loop_count"] = qint64(loops);
        }
        if (backfill) {
            require(!input.contains("unassigned_source_ids"), "Boundary input already has full-resolution assignment");
            input["unassigned_source_ids"] = ids(assignSourceObservations(source, work, options, patches, grouping, {}));
            metrics["unassigned_source_count"] = input["unassigned_source_ids"].toArray().size();
        }
        if (module == 21) {
            require(input["unassigned_source_ids"].isArray(), "Closure requires full-resolution assignment");
            const auto candidate = closeParts(work, options, patches, grouping.parts, source.size(), {});
            metrics["closed_count"] = qint64(std::count_if(grouping.parts.begin(), grouping.parts.end(),
                [](const auto& part) { return part.closed; }));
            double mass = 0;
            if (!candidate.profileBodies.empty() || !candidate.curvedSolids.empty()) {
                modeling::ModelingCore model;
                const auto committed = commitCompoundCandidate(candidate, model);
                if (!committed.success) throw std::runtime_error(committed.error);
                require(!model.bodyShape().IsNull() && BRepCheck_Analyzer(model.bodyShape()).IsValid(), "Invalid closed B-rep");
                GProp_GProps properties;
                BRepGProp::VolumeProperties(model.bodyShape(), properties);
                mass = properties.Mass();
            }
            metrics["volume_mm3"] = mass;
            QJsonArray revised;
            for (const auto& patch : patches) if (const auto& prior = patch.boundaryBeforeClosure)
                revised.append(QJsonObject{{"patch_id", qint64(patch.id)}, {"loops", loopsJson(prior->loops)}});
            input["boundaries_before_closure"] = revised;
        }
        writeGrouping(input, grouping);
    }
    input["patches"] = writePatches(patches);
    return input;
}
} // namespace contract
