#include "io/StepModelLoader.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <QFile>
#include <QFileInfo>

#include <exception>
#include <string>

namespace {

// OCCT 8 exposes the failure text through what(); older releases only had the
// message string. CadModelCore carries the same shim, but its header lives in
// the core's private include directory, so the client keeps its own copy.
std::string failureText(const Standard_Failure& failure)
{
#if OCC_VERSION_HEX >= 0x080000
    return failure.what();
#else
    const char* message = failure.GetMessageString();
    return message == nullptr ? std::string{} : std::string(message);
#endif
}

int countSubShapes(const TopoDS_Shape& shape, TopAbs_ShapeEnum type)
{
    // MapShapes counts every distinct sub-shape once. A STEP file often comes
    // back as a compound that reaches the same face or edge through more than
    // one parent, and walking it with an explorer would count those repeatedly.
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(shape, type, map);
    return map.Extent();
}

// Centre of mass of a shape: the volume centre for anything that encloses a
// volume, which is what a part's centre of mass means, and the centre of the
// bounding box for a shape that does not - an open shell or a loose set of
// faces - so that every model can still be brought to the origin.
bool centroidOf(const TopoDS_Shape& shape, gp_Pnt& centroid)
{
    GProp_GProps volume;
    BRepGProp::VolumeProperties(shape, volume);
    if (volume.Mass() > 0.0) {
        centroid = volume.CentreOfMass();
        return true;
    }

    Bnd_Box bounds;
    BRepBndLib::Add(shape, bounds);
    if (bounds.IsVoid()) {
        return false;
    }
    double xMin = 0.0;
    double yMin = 0.0;
    double zMin = 0.0;
    double xMax = 0.0;
    double yMax = 0.0;
    double zMax = 0.0;
    bounds.Get(xMin, yMin, zMin, xMax, yMax, zMax);
    centroid = gp_Pnt(0.5 * (xMin + xMax), 0.5 * (yMin + yMax), 0.5 * (zMin + zMax));
    return true;
}

// Moves the shape so that its centre of mass sits on the origin. A shape whose
// centre cannot be worked out stays where it is, and the caller is told so.
bool moveCentroidToOrigin(TopoDS_Shape& shape)
{
    gp_Pnt centroid;
    if (!centroidOf(shape, centroid)) {
        return false;
    }

    const gp_Vec shift(-centroid.X(), -centroid.Y(), -centroid.Z());
    if (shift.SquareMagnitude() <= 1.0e-12) {
        return false;
    }

    gp_Trsf translation;
    translation.SetTranslation(shift);
    BRepBuilderAPI_Transform moved(shape, translation, true);
    if (!moved.IsDone()) {
        return false;
    }
    shape = moved.Shape();
    return true;
}

QString stepError(const QString& fileName, const QString& reason)
{
    return QStringLiteral("Could not import STEP file '%1': %2").arg(fileName, reason);
}

}  // namespace

bool StepModelLoader::canLoad(const QString& filePath) const
{
    // Extension matching is case insensitive, so "PART.STP" is recognised too.
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("step") || suffix == QStringLiteral("stp");
}

bool StepModelLoader::load(const QString& filePath, LoadedModel& model,
                           QString& errorMessage)
{
    errorMessage.clear();

    const QFileInfo info(filePath);
    const QString fileName = info.fileName();
    if (!info.exists() || !info.isFile()) {
        errorMessage = QStringLiteral("STEP file not found: %1").arg(filePath);
        return false;
    }
    if (!canLoad(filePath)) {
        errorMessage = QStringLiteral(
                           "'%1' is not a STEP file; only .step and .stp are supported")
                           .arg(fileName);
        return false;
    }

    try {
        STEPControl_Reader reader;

        // OCCT opens the path with the process' native encoding, so the name is
        // converted rather than passed as UTF-16.
        const QByteArray nativePath = QFile::encodeName(info.absoluteFilePath());
        if (reader.ReadFile(nativePath.constData()) != IFSelect_RetDone) {
            errorMessage = stepError(
                fileName,
                QStringLiteral("OpenCASCADE could not read it; the file is "
                               "corrupt or is not STEP data"));
            return false;
        }

        // Roots are what the transfer produces; a file whose entities never make
        // it into a shape has nothing to show, so it counts as a failure.
        const int transferredRoots = reader.TransferRoots();
        if (transferredRoots <= 0) {
            errorMessage = stepError(
                fileName, QStringLiteral("it contains no transferable shape"));
            return false;
        }
        if (reader.NbShapes() <= 0) {
            errorMessage = stepError(
                fileName, QStringLiteral("OpenCASCADE transferred no shape"));
            return false;
        }

        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
            errorMessage = stepError(
                fileName, QStringLiteral("OpenCASCADE returned an empty shape"));
            return false;
        }

        // A STEP file carries whatever placement its exporting system used, so
        // the part can be parked anywhere in space. It is moved onto the origin
        // by its centre of mass, because the viewport orbits and frames around
        // the origin: a model left off it would swing around empty space instead
        // of turning in place.
        moveCentroidToOrigin(shape);

        // Everything validated, so the caller's model is only now replaced.
        LoadedModel loaded;
        loaded.shape = shape;
        loaded.sourcePath = info.absoluteFilePath();
        loaded.solidCount = countSubShapes(shape, TopAbs_SOLID);
        loaded.faceCount = countSubShapes(shape, TopAbs_FACE);
        loaded.edgeCount = countSubShapes(shape, TopAbs_EDGE);
        loaded.vertexCount = countSubShapes(shape, TopAbs_VERTEX);
        model = loaded;
        return true;
    } catch (const Standard_Failure& failure) {
        errorMessage = stepError(
            fileName,
            QStringLiteral("OpenCASCADE failed: %1")
                .arg(QString::fromUtf8(failureText(failure).c_str())));
        return false;
    } catch (const std::exception& error) {
        errorMessage = stepError(fileName, QString::fromUtf8(error.what()));
        return false;
    }
}
