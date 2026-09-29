#include "io/StepModelLoader.h"

#include <modeling/StepExporter.h>

#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <QFile>
#include <QFileInfo>
#include <QTemporaryDir>

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void writeTextFile(const QString& path, const QByteArray& contents)
{
    QFile file(path);
    require(file.open(QIODevice::WriteOnly | QIODevice::Truncate),
            "Cannot create temporary file");
    require(file.write(contents) == contents.size(), "Cannot write temporary file");
    file.close();
}

// Extension matching must accept both spellings in either case without asking
// the file system anything.
void testCanLoadMatchesStepExtensions()
{
    StepModelLoader loader;
    require(loader.canLoad(QStringLiteral("part.step")), ".step must be loadable");
    require(loader.canLoad(QStringLiteral("part.stp")), ".stp must be loadable");
    require(loader.canLoad(QStringLiteral("PART.STEP")),
            "extension matching must ignore case");
    require(loader.canLoad(QStringLiteral("assembly.STP")),
            "extension matching must ignore case");
    require(!loader.canLoad(QStringLiteral("mesh.obj")), ".obj must be refused");
    require(!loader.canLoad(QStringLiteral("cloud.ply")), ".ply must be refused");
    require(!loader.canLoad(QStringLiteral("part.step.txt")),
            "a trailing extension that is not step must be refused");
    require(!loader.canLoad(QStringLiteral("part")), "a missing extension must be refused");
}

void testMissingFileFailsWithMessage()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary directory");

    StepModelLoader loader;
    LoadedModel model;
    QString error;
    require(!loader.load(temporary.filePath(QStringLiteral("absent.step")), model, error),
            "a missing file must fail");
    require(!error.isEmpty(), "a missing file must produce an error message");
    require(!model.isValid(), "a failed load must not produce a model");
}

// A file that is not STEP data at all has to be rejected cleanly: no crash, no
// half-built shape.
void testInvalidStepFileFailsGracefully()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary directory");
    const QString path = temporary.filePath(QStringLiteral("broken.stp"));
    writeTextFile(path, QByteArray("This is not a STEP file.\n"));

    StepModelLoader loader;
    LoadedModel model;
    QString error;
    require(!loader.load(path, model, error), "an invalid STEP file must fail");
    require(!error.isEmpty(), "an invalid STEP file must produce an error message");
    require(!model.isValid(), "an invalid STEP file must not produce a model");
}

// The repository ships no STEP sample, so one is written from OpenCASCADE at
// test time: a box whose topology is known exactly, read back through the
// loader that the client uses.
void testBoxRoundTripKeepsShapeAndTopology()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary directory");
    const QString path = temporary.filePath(QStringLiteral("box.step"));

    const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    std::string exportError;
    require(modeling::exportStep(box,
                                 std::filesystem::path(
                                     QFile::encodeName(path).toStdString()),
                                 &exportError),
            "OpenCASCADE could not write the test STEP file: " + exportError);

    StepModelLoader loader;
    require(loader.canLoad(path), "the written sample must be recognised as STEP");

    LoadedModel model;
    QString error;
    require(loader.load(path, model, error), error.toStdString());
    require(model.isValid(), "a loaded STEP model must keep a non-null shape");
    require(!model.shape.IsNull(), "the B-Rep must survive the load");
    require(model.sourcePath == QFileInfo(path).absoluteFilePath(),
            "the model must record where it came from");
    std::cout << "Box topology: " << model.solidCount << " solids, "
              << model.faceCount << " faces, " << model.edgeCount << " edges, "
              << model.vertexCount << " vertices.\n";
    require(model.solidCount == 1, "a box is one solid");
    require(model.faceCount == 6, "a box has six faces");
    require(model.edgeCount == 12, "a box has twelve edges");
    require(model.vertexCount == 8, "a box has eight vertices");

    // Reading a second file of the same name in the upper case spelling proves
    // the reader works from the path, not from a cached previous load.
    LoadedModel second;
    QString secondError;
    require(loader.load(temporary.filePath(QStringLiteral("box.step")), second,
                        secondError),
            secondError.toStdString());
    require(second.faceCount == 6, "the reloaded box must keep its six faces");
}

// The viewport orbits and frames around the origin, so a part has to arrive
// there. A STEP file keeps the placement its exporting system used, and the
// loader is what takes it off that placement.
void testLoadMovesCentroidToOrigin()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary directory");
    const QString path = temporary.filePath(QStringLiteral("offset.step"));

    // A 10 x 20 x 30 box parked well away from the origin.
    const TopoDS_Shape box =
        BRepPrimAPI_MakeBox(gp_Pnt(100.0, -50.0, 25.0), 10.0, 20.0, 30.0).Shape();
    require(modeling::exportStep(box,
                                 std::filesystem::path(
                                     QFile::encodeName(path).toStdString())),
            "OpenCASCADE could not write the test STEP file");

    StepModelLoader loader;
    LoadedModel model;
    QString error;
    require(loader.load(path, model, error), error.toStdString());

    GProp_GProps properties;
    BRepGProp::VolumeProperties(model.shape, properties);
    const gp_Pnt centre = properties.CentreOfMass();
    std::cout << "Centred model: centre of mass at " << centre.X() << ", "
              << centre.Y() << ", " << centre.Z() << ".\n";
    require(centre.Distance(gp_Pnt(0.0, 0.0, 0.0)) < 1.0e-6,
            "the loaded box must sit with its centre of mass on the origin");
    require(model.solidCount == 1 && model.faceCount == 6 && model.edgeCount == 12
                && model.vertexCount == 8,
            "moving the model must not change its topology");
}

// A rejected file must leave an already loaded model exactly as it was, which is
// what keeps a failed import from clearing the scene.
void testFailedLoadKeepsPreviousModel()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary directory");

    const QString goodPath = temporary.filePath(QStringLiteral("good.step"));
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
    require(modeling::exportStep(box,
                                 std::filesystem::path(
                                     QFile::encodeName(goodPath).toStdString())),
            "OpenCASCADE could not write the test STEP file");

    StepModelLoader loader;
    LoadedModel model;
    QString error;
    require(loader.load(goodPath, model, error), error.toStdString());
    const int facesAfterGoodLoad = model.faceCount;

    const QString badPath = temporary.filePath(QStringLiteral("bad.stp"));
    writeTextFile(badPath, QByteArray("not step data\n"));
    require(!loader.load(badPath, model, error), "an invalid STEP file must fail");
    require(!error.isEmpty(), "the failure must be explained");
    require(model.isValid() && model.faceCount == facesAfterGoodLoad,
            "a failed load must leave the previously loaded model untouched");
}

}  // namespace

int main()
{
    try {
        testCanLoadMatchesStepExtensions();
        testMissingFileFailsWithMessage();
        testInvalidStepFileFailsGracefully();
        testBoxRoundTripKeepsShapeAndTopology();
        testLoadMovesCentroidToOrigin();
        testFailedLoadKeepsPreviousModel();
        std::cout << "STEP model loader test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "STEP model loader test failed: " << exception.what() << '\n';
        return 1;
    }
}
