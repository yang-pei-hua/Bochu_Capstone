#include "core/PointCloudSelection.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTemporaryDir>
#include <vtkNew.h>
#include <vtkPlanes.h>
#include <vtkTransform.h>

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
void writeJson(const QString& path, const QJsonObject& root)
{
    QFile file(path);
    require(file.open(QIODevice::WriteOnly), "open JSON fixture");
    file.write(QJsonDocument(root).toJson());
}
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    try {
        PlyCloud cloud;
        cloud.positions = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0},
                           {0, 1, 0}, {20, 0, 0}, {0, 0, 2}};
        PointCloudSelection selection;
        selection.setCloud(cloud);
        selection.select({0, 0, -1, 99});
        require(selection.selected() == std::vector<vtkIdType>{0}, "seed IDs filtered and deduplicated");
        selection.expand(1.01);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 4}), "first expansion is one Euclidean step");
        selection.expand(1.01);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 2, 4}), "second expansion grows from all previous seeds");
        selection.expand(1.01);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 2, 3, 4}), "third expansion reaches end of chain");
        const auto beforeInvalid = selection.selected();
        selection.expand(std::numeric_limits<double>::quiet_NaN());
        selection.expand(-1);
        require(selection.selected() == beforeInvalid, "invalid radius leaves selection intact");
        selection.select({5});
        selection.select({0}, true);
        selection.expand(1.01);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 4, 5}), "all disconnected seeds grow, preserve existing points");
        selection.toggle(0); selection.toggle(2); selection.toggle(-1);
        require(selection.selected() == std::vector<vtkIdType>({1, 2, 4, 5}), "modifier selection toggles membership");

        vtkNew<vtkPlanes> planes;
        double bounds[6]{-0.1, 1.1, -0.1, 1.1, -0.1, 0.1};
        planes->SetBounds(bounds);
        selection.selectBox(planes, false);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 4}), "3D box excludes out-of-depth points");
        // A rotated box must use its six world-space planes, not its AABB.
        vtkNew<vtkTransform> transform;
        transform->RotateZ(45);
        planes->SetTransform(transform);
        selection.selectBox(planes, false);
        require(selection.selected() != std::vector<vtkIdType>({0, 1, 4}), "rotated box obeys planes");
        planes->SetTransform(static_cast<vtkAbstractTransform*>(nullptr));
        selection.select({5});
        selection.selectBox(planes, true);
        require(selection.selected() == std::vector<vtkIdType>({0, 1, 4, 5}), "box add retains previous selection");

        selection.select({0, 1});
        const int face = selection.assign("plane", "顶面");
        selection.select({1, 2});
        const int sphere = selection.assign("sphere", "球 1");
        require(face > 0 && sphere != face, "same template can have separate named groups");
        require(selection.regionPointCount(face) == 1 && selection.regionPointCount(sphere) == 2,
                "overlap reassigns selected points and preserves other points");
        selection.select({3});
        selection.assign("plane", "ignored", sphere);
        require(selection.regionPointCount(sphere) == 3 && selection.regions()[1].geometry == "sphere",
                "append to existing region retains its template");
        selection.selectRegion(sphere);
        require(selection.selected() == std::vector<vtkIdType>({1, 2, 3}), "recall group selection");
        selection.select({6});
        const int ring = selection.assign("ring", "环");
        selection.select({4});
        selection.assign("cuboid", "长方体");
        selection.select({5});
        selection.assign("cylinder", "圆柱");
        require(selection.assign("invalid", "bad") == 0, "invalid template rejected");

        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory");
        const auto path = temp.filePath("regions.json");
        QString error;
        require(selection.save(path, "cloud.ply", error), "save annotations");
        PointCloudSelection restored;
        restored.setCloud(cloud);
        require(restored.load(path, error), "load annotations");
        require(restored.memberships() == selection.memberships()
                && restored.regions().size() == selection.regions().size(), "annotation round trip");
        require(restored.regions()[0].name == "顶面", "Unicode region names round trip");
        PlyCloud wrongCloud = cloud;
        std::swap(wrongCloud.positions[0], wrongCloud.positions[1]);
        PointCloudSelection wrong;
        wrong.setCloud(wrongCloud);
        require(!wrong.load(path, error), "same count but reordered points rejected");

        QFile source(path);
        require(source.open(QIODevice::ReadOnly), "read JSON fixture");
        auto root = QJsonDocument::fromJson(source.readAll()).object();
        auto regions = root["regions"].toArray();
        auto malformed = regions[0].toObject();
        malformed["pointIndices"] = QJsonArray{999};
        regions[0] = malformed;
        root["regions"] = regions;
        const auto invalidPath = temp.filePath("invalid.json");
        writeJson(invalidPath, root);
        const auto membershipsBefore = restored.memberships();
        restored.select({5});
        require(!restored.load(invalidPath, error), "invalid index rejected");
        require(restored.memberships() == membershipsBefore && restored.selected() == std::vector<vtkIdType>{5},
                "bad import preserves current annotations and selection");
        restored.selectRegion(ring); restored.unassign();
        require(restored.regionPointCount(ring) == 0, "unassign selected points");
        restored.removeRegion(sphere);
        require(restored.regionPointCount(sphere) == 0 && restored.regionPointCount(face) == 1,
                "delete region preserves other labels");
        restored.setCloud(PlyCloud{});
        restored.expand(1); restored.selectBox(planes, false);
        require(restored.selected().empty() && restored.regions().empty() && restored.memberships().empty(),
                "new or cleared cloud resets selection, labels, and locator");
        std::cout << "Point cloud selection tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
