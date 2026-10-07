#include "core/PointCloudSelection.h"
#include "io/PlyReader.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
bool hasRelation(const PointCloudSelection& document, int a, int b, const QString& type)
{
    for (const auto& r : document.relations()) if (std::min(r.first,r.second) == std::min(a,b)
        && std::max(r.first,r.second) == std::max(a,b) && r.type == type) return true;
    return false;
}

void testDenseSeparatedPatches()
{
    PlyCloud cloud;
    std::vector<DetectedPointCloudSurface> surfaces;
    for (int face = 0; face < 2; ++face) {
        PointCloudRegion region;
        region.geometry = "plane";
        region.name = QStringLiteral("Dense face %1").arg(face);
        region.confidence = 0.95;
        region.parameters = {{"normal", QJsonArray{0, 0, 1}}, {"offset", -1000.0 * face}};
        DetectedPointCloudSurface surface{region, {}};
        for (int x = 0; x < 256; ++x) for (int y = 0; y < 256; ++y) {
            surface.points.push_back(static_cast<vtkIdType>(cloud.positions.size()));
            cloud.positions.push_back({x * 0.01F, y * 0.01F, face * 1000.0F});
        }
        surfaces.push_back(std::move(surface));
    }
    PointCloudSelection document;
    document.setCloud(cloud);
    document.addSurfaces(surfaces);
    QElapsedTimer elapsed;
    elapsed.start();
    document.inferRelations(0.005, 2);
    require(hasRelation(document, 1, 2, "parallel") && !hasRelation(document, 1, 2, "adjacent"),
            "dense separated surfaces retain analytic relations without false adjacency");
    require(elapsed.elapsed() < 5000,
            "dense separated patches must not perform unrestricted nearest-point searches");
}

void testAdjacencyToleranceBoundary()
{
    PlyCloud cloud;
    cloud.positions = {{0, 0, 0}, {0.125F, 0, 0}};
    PointCloudRegion region;
    region.geometry = "plane";
    PointCloudSelection document;
    document.setCloud(cloud);
    document.addSurfaces({{region, {0}}, {region, {1}}});
    document.inferRelations(0.125, 2);
    require(hasRelation(document, 1, 2, "adjacent"), "adjacency includes points exactly at tolerance");
    document.inferRelations(0.12, 2);
    require(!hasRelation(document, 1, 2, "adjacent"), "points outside tolerance remain separate");
}
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    try {
        testDenseSeparatedPatches();
        testAdjacencyToleranceBoundary();
        PlyCloud cloud;
        std::vector<DetectedPointCloudSurface> surfaces;
        for (int face = 0; face < 3; ++face) {
            PointCloudRegion region;
            region.geometry = "plane"; region.name = QStringLiteral("面 %1").arg(face+1); region.confidence = 0.95;
            region.parameters = face == 2 ? QJsonObject{{"normal", QJsonArray{1,0,0}}, {"offset", 0}}
                : QJsonObject{{"normal", QJsonArray{0,0,1}}, {"offset", face == 0 ? 0 : -3}};
            DetectedPointCloudSurface surface{region, {}};
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) {
                surface.points.push_back(static_cast<vtkIdType>(cloud.positions.size()));
                cloud.positions.push_back(face == 2 ? std::array<float,3>{0,static_cast<float>(i),static_cast<float>(j)}
                    : std::array<float,3>{static_cast<float>(i),static_cast<float>(j),face == 0 ? 0.0F : 3.0F});
                cloud.normals.push_back(face == 2 ? std::array<float,3>{1,0,0} : std::array<float,3>{0,0,1});
                cloud.colors.push_back({static_cast<unsigned char>(cloud.positions.size()), 100, 200});
            }
            surfaces.push_back(surface);
        }
        PointCloudSelection document; document.setCloud(cloud); document.addSurfaces(surfaces);
        require(document.regions().size() == 3 && document.regionPointCount(0) == 0, "detected support becomes original point membership");
        document.inferRelations(0.1, 2);
        require(hasRelation(document,1,2,"parallel") && !hasRelation(document,1,2,"adjacent"),
            "parallel infinite planes are not treated as adjacent finite patches");
        require(hasRelation(document,1,3,"perpendicular") && hasRelation(document,1,3,"adjacent")
            && hasRelation(document,1,3,"intersect"), "orthogonal touching patches expose typed relationships");
        int parallelId = 0;
        for (const auto& r : document.relations()) if (r.type == "parallel") parallelId = r.id;
        document.removeRelation(parallelId);
        document.inferRelations(0.1,2);
        require(!hasRelation(document,1,2,"parallel"), "deleted automatic relationship stays deleted on reanalysis");
        require(document.undo() && document.undo() && hasRelation(document,1,2,"parallel"), "undo restores deleted relationship");
        {
            QTemporaryDir saved; QString error;
            const auto path = saved.filePath("suppressed.json");
            document.removeRelation(parallelId);
            require(document.save(path,"original.ply",error), "save suppressed automatic relationship");
            PointCloudSelection reopened; reopened.setCloud(cloud);
            require(reopened.load(path,error), "reload suppression state");
            reopened.inferRelations(0.1,2);
            require(!hasRelation(reopened,1,2,"parallel"), "relationship suppression survives a saved session");
            reopened.removeRegion(3);
            require(reopened.save(path,"original.ply",error) && reopened.load(path,error), "reload after removing highest collection ID");
            reopened.select({32});
            require(reopened.assign("plane","新集合") == 4, "saved ID counters prevent reuse of removed collection identities");
            document.save(path,"original.ply",error);
            QFile legacy(path); require(legacy.open(QIODevice::ReadOnly), "read legacy fixture");
            auto root = QJsonDocument::fromJson(legacy.readAll()).object(); legacy.close();
            root["version"] = 1; root.remove("relations"); root.remove("deletedPointIndices"); root.remove("suppressedRelations");
            auto groups = root["regions"].toArray();
            for (int i=0;i<groups.size();++i) {
                auto group = groups[i].toObject();
                for (const auto* key : {"parameters","source","residual","confidence","fitStale"}) group.remove(key);
                groups[i] = group;
            }
            root["regions"] = groups;
            require(legacy.open(QIODevice::WriteOnly | QIODevice::Truncate), "write version 1 fixture");
            legacy.write(QJsonDocument(root).toJson()); legacy.close();
            require(reopened.load(path,error) && reopened.activePointCount()==48
                && reopened.regions().size()==3 && reopened.relations().empty() && reopened.region(1)->parameters.isEmpty(),
                "version 1 semantic collections remain loadable without inferred metadata");
            require(document.undo(), "restore fixture after suppression persistence test");
        }
        const int manual = document.setRelation(0,1,2,"same_radius");
        require(manual && !document.relation(manual)->automatic, "manual relationship stored separately from evidence");
        require(document.setRelation(0,2,1,"same_radius") == 0 && document.setRelation(0,1,1,"parallel") == 0,
            "duplicate or self relationships rejected");
        require(document.setRelation(manual,1,2,"coplanar") == manual, "editing relationship retains identity");
        require(document.editRegion(1,"顶面","plane") && document.region(1)->parameters.size() == 2,
            "rename retains fitted parameters");
        document.select({0,16});
        document.assign("plane","ignored",1);
        require(document.region(1)->fitStale && document.region(2)->fitStale, "membership change invalidates both fitted surfaces");
        require(!hasRelation(document,1,2,"parallel") && hasRelation(document,1,2,"coplanar"),
            "automatic relations invalidated; explicit manual annotation retained");
        document.undo();
        document.selectRegion(3); document.deleteRegionPoints(3);
        require(document.activePointCount() == 32 && document.regionPointCount(3) == 0 && document.selected().empty(),
            "delete collection points updates active points and selection");
        for (const auto& r : document.relations()) require(r.first != 3 && r.second != 3, "deleted collection points remove dangling relations");
        document.select({32,33}); require(document.selected().empty(), "deleted IDs cannot be selected");
        document.select({0}); document.expand(100);
        require(document.selected().size() == 32, "deleted points excluded from rebuilt spatial locator");
        std::vector<vtkIdType> originalIds;
        const auto working = document.workingCloud(&originalIds);
        require(working.positions.size() == 32 && working.normals.size() == 32 && working.colors.size() == 32
            && originalIds.front() == 0 && originalIds.back() == 31, "working CAD input retains attributes and source IDs");
        QTemporaryDir temp; require(temp.isValid(), "temporary directory"); QString error;
        const auto session = temp.filePath("session.json"); require(document.save(session,"original.ply",error), "save graph and deletion mask");
        PointCloudSelection restored; restored.setCloud(cloud);
        require(restored.load(session,error) && restored.activePointCount() == 32
            && restored.relations().size() == document.relations().size() && restored.region(1)->name == "顶面",
            "graph, names, fit metadata and deleted points round trip");
        const auto exported = temp.filePath("edited.ply"); require(restored.exportCloud(exported,error), "export active cloud");
        PlyCloud exportedCloud; require(PlyReader::read(exported,exportedCloud,error), "read binary export");
        require(exportedCloud.positions == working.positions && exportedCloud.normals == working.normals
            && exportedCloud.colors == working.colors, "export excludes deleted points and preserves source attributes");
        restored.select({0}); const auto before = restored.revision();
        QFile file(session); require(file.open(QIODevice::ReadOnly), "read fixture");
        auto root = QJsonDocument::fromJson(file.readAll()).object(); file.close();
        auto relations = root["relations"].toArray(); auto invalid = relations[0].toObject(); invalid["second"] = 999;
        relations[0] = invalid; root["relations"] = relations;
        require(file.open(QIODevice::WriteOnly | QIODevice::Truncate), "write fixture"); file.write(QJsonDocument(root).toJson()); file.close();
        require(!restored.load(session,error) && restored.revision() == before && restored.activePointCount() == 32
            && restored.selected() == std::vector<vtkIdType>{0}, "invalid graph import is atomic");
        restored.removeRegion(1);
        require(restored.activePointCount() == 32 && restored.regionPointCount(0) == 16, "delete collection releases points without deleting observations");
        require(restored.undo() && restored.region(1), "collection deletion undo");
        restored.select({16}); restored.replaceRegionPoints(1);
        require(restored.regionPointCount(1) == 1 && restored.regionPointCount(0) == 16, "replace collection from viewport selection");
        restored.undo(); restored.selectRegion(1); restored.deleteSelectedPoints();
        restored.selectRegion(2); restored.deleteSelectedPoints();
        require(restored.activePointCount() == 0, "all points can be removed");
        restored.expand(1); require(restored.selected().empty(), "empty locator remains safe");
        require(restored.undo() && restored.activePointCount() == 16, "undo restores observations and locator after empty cloud");
        std::cout << "Point cloud relationship tests passed.\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
