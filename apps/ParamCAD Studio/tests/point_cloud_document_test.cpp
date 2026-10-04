#include "core/PointCloudDocument.h"

#include <QCoreApplication>
#include <QFile>
#include <QJsonDocument>
#include <QTemporaryDir>

#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
void writeCloud(const QString& path, bool different = false)
{
    QFile file(path);
    require(file.open(QIODevice::WriteOnly), "create fixture");
    file.write("ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\n"
               "property float y\nproperty float z\nproperty float nx\n"
               "property float ny\nproperty float nz\nproperty uchar red\n"
               "property uchar green\nproperty uchar blue\nend_header\n");
    file.write(different ? "9 0 0 0 0 1 10 20 30\n" : "0 0 0 0 0 1 10 20 30\n");
    file.write("1 0 0 0 0 1 40 50 60\n2 0 0 0 1 0 70 80 90\n3 0 0 0 1 0 100 110 120\n");
}
}

int main(int argc, char** argv)
{
    // This complete document workflow needs no QApplication or render window.
    QCoreApplication application(argc, argv);
    try {
        QTemporaryDir temp;
        require(temp.isValid(), "temporary directory");
        const QString path = temp.filePath("cloud.ply");
        const QString annotationPath = temp.filePath("regions.json");
        writeCloud(path);
        PointCloudDocument document;
        QStringList events;
        QObject::connect(&document, &PointCloudDocument::cloudChanged, [&] { events << "cloud"; });
        QObject::connect(&document, &PointCloudDocument::documentChanged, [&] { events << "document"; });
        QObject::connect(&document, &PointCloudDocument::selectionChanged, [&] { events << "selection"; });
        require(!document.hasCloud() && document.sourcePath().isEmpty(), "empty document");
        QString error;
        require(document.load(path, error), "load without viewport");
        require(events == QStringList({"cloud", "document", "selection"}), "replacement notification order");
        require(document.hasCloud() && document.pointCount() == 4 && document.sourcePath() == path,
                "document owns source identity and observations");
        const auto loadedRevision = document.revision();
        events.clear();
        document.select({0, 1});
        require(events == QStringList{"selection"} && document.revision() == loadedRevision,
                "selection alone keeps background snapshots valid");
        const int first = document.assignRegion("plane", "first");
        document.select({2, 3});
        const int second = document.assignRegion("plane", "second");
        const int relation = document.setRelation(0, first, second, "perpendicular");
        require(relation != 0 && document.revision() > loadedRevision, "semantic edits advance revision");
        document.selectRelation(relation);
        require(document.selection().selected() == std::vector<vtkIdType>({0, 1, 2, 3}),
                "relation selection uses both collections");
        require(document.saveAnnotations(annotationPath, error), "save annotations through document");
        QFile annotations(annotationPath);
        require(annotations.open(QIODevice::ReadOnly), "read annotations");
        require(QJsonDocument::fromJson(annotations.readAll()).object()["source"].toString() == path,
                "saved annotations retain authoritative source path");

        document.selectPoint(1);
        const auto beforeDelete = document.revision();
        document.deleteSelectedPoints();
        require(document.pointCount() == 3 && document.sourceCloud().positions.size() == 4,
                "deletion keeps immutable original observations");
        require(document.revision() > beforeDelete, "point edit invalidates pending jobs");
        std::vector<vtkIdType> ids;
        const auto snapshot = document.workingCloud(&ids);
        require(ids == std::vector<vtkIdType>({0, 2, 3}) && snapshot.positions.size() == 3,
                "working snapshot preserves original IDs");
        require(snapshot.normals.size() == 3 && snapshot.colors.size() == 3
                && snapshot.colors[1] == document.sourceCloud().colors[2], "snapshot preserves normals and colors");
        const auto subset = document.workingCloud(&ids, false, second);
        require(ids == std::vector<vtkIdType>({2, 3}) && subset.positions.size() == 2,
                "region analysis snapshot preserves membership filtering");
        const auto deletedRevision = document.revision();
        require(document.undo() && document.pointCount() == 4 && document.revision() > deletedRevision,
                "undo restores observations and advances revision");
        document.selectPoint(1);
        document.deleteSelectedPoints();
        require(document.exportCloud(temp.filePath("edited.ply"), error), "export working cloud");
        PlyCloud exported;
        require(PlyReader::read(temp.filePath("edited.ply"), exported, error)
                && exported.positions == snapshot.positions && exported.colors == snapshot.colors
                && exported.normals == snapshot.normals, "export round trip preserves surviving observations");
        require(document.loadAnnotations(annotationPath, error) && document.pointCount() == 4
                && document.selection().relation(relation), "annotation load restores edits and relationships");
        require(document.undo() && document.pointCount() == 3, "annotation load remains undoable");

        const auto beforeFailure = document.revision();
        const auto previousSelection = document.selection().selected();
        events.clear();
        require(!document.load(temp.filePath("missing.ply"), error), "failed load");
        require(document.revision() == beforeFailure && document.sourcePath() == path
                && document.pointCount() == 3 && document.selection().selected() == previousSelection
                && events.isEmpty(), "failed load preserves live document and signals");
        require(document.load(path, error) && document.revision() > beforeFailure,
                "identical reload invalidates old snapshots");
        require(document.selection().regions().empty() && document.selection().relations().empty()
                && !document.selection().canUndo(), "replacement resets annotations and history");
        const auto beforeClear = document.revision();
        events.clear();
        document.clear();
        require(!document.hasCloud() && document.pointCount() == 0 && document.sourcePath().isEmpty()
                && document.revision() > beforeClear
                && events == QStringList({"cloud", "document", "selection"}), "clear resets identity and invalidates jobs");
        require(snapshot.positions.size() == 3, "owned worker snapshot survives document reset");

        const QString otherPath = temp.filePath("other.ply");
        writeCloud(otherPath, true);
        require(document.load(otherPath, error), "load different cloud");
        const auto otherRevision = document.revision();
        events.clear();
        require(!document.loadAnnotations(annotationPath, error) && document.revision() == otherRevision
                && document.selection().regions().empty() && events.isEmpty(),
                "incompatible annotations preserve new document");
        std::cout << "Point cloud document tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
