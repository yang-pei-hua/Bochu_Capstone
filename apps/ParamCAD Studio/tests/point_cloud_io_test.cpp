#include "io/PlyReader.h"
#include "io/PointCloudAdapter.h"

#include <reconstruction/BoxRecognizer.h>
#include <reconstruction/PrimitiveDetector.h>

#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <cmath>
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

void testAsciiNormalsAndManifestUnit()
{
    QTemporaryDir temporary;
    require(temporary.isValid(), "Cannot create temporary point-cloud directory");
    const QString cloudPath = temporary.filePath(QStringLiteral("cloud.ply"));
    QFile cloudFile(cloudPath);
    require(cloudFile.open(QIODevice::WriteOnly | QIODevice::Truncate),
            "Cannot create temporary PLY");
    const QByteArray contents =
        "ply\n"
        "format ascii 1.0\n"
        "element vertex 2\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property float nx\n"
        "property float ny\n"
        "property float nz\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
        "0 0 0 0 0 2 255 0 0\n"
        "1 2 3 0 1 0 0 255 0\n";
    require(cloudFile.write(contents) == contents.size(),
            "Cannot write temporary PLY");
    cloudFile.close();

    PlyCloud cloud;
    QString error;
    require(PlyReader::read(cloudPath, cloud, error), error.toStdString());
    require(cloud.positions.size() == 2U && cloud.normals.size() == 2U &&
                cloud.colors.size() == 2U,
            "PLY attributes must remain aligned");
    require(std::abs(cloud.normals[0][2] - 2.0F) < 1.0e-6F,
            "PLY normal components must be read by property name");

    const reconstruction::PointStore store = PointCloudAdapter::toPointStore(
        cloud, reconstruction::LengthUnit::Meter);
    require(store.points()[0].normal.has_value(),
            "PLY normal must reach PointStore");
    require(std::abs(store.points()[0].normal->z - 2.0) < 1.0e-12,
            "PointCloudAdapter must preserve the input normal before preprocessing");

    QDir root(temporary.path());
    require(root.mkpath(QStringLiteral("workspace")),
            "Cannot create manifest directory");
    QFile manifest(root.filePath(QStringLiteral("workspace/reconstruction.json")));
    require(manifest.open(QIODevice::WriteOnly | QIODevice::Truncate),
            "Cannot create reconstruction manifest");
    manifest.write("{\"schema_version\":1,\"unit\":\"m\"}\n");
    manifest.close();
    require(PointCloudAdapter::lengthUnitFor(cloudPath) ==
                reconstruction::LengthUnit::Meter,
            "Point-cloud unit must be read from the reconstruction manifest");
}

}  // namespace

int main(int argc, char* argv[])
{
    try {
        testAsciiNormalsAndManifestUnit();
        if (argc > 1) {
            const QString path = QString::fromLocal8Bit(argv[1]);
            PlyCloud cloud;
            QString error;
            require(PlyReader::read(path, cloud, error), error.toStdString());
            const reconstruction::LengthUnit unit =
                PointCloudAdapter::lengthUnitFor(path);
            const reconstruction::PointStore source =
                PointCloudAdapter::toPointStore(cloud, unit);
            reconstruction::PointStore processed({}, unit);
            reconstruction::PointCloudPreprocessingReport report;
            std::string preprocessingError;
            require(reconstruction::preprocessPointCloud(
                        source,
                        PointCloudAdapter::preprocessingOptionsFor(cloud),
                        processed,
                        report,
                        preprocessingError),
                    preprocessingError);
            std::cout << "Preprocessed " << report.inputPointCount << " -> "
                      << report.outputPointCount << " points (voxel removed "
                      << report.removedByVoxel << ", outliers removed "
                      << report.removedAsOutliers << ", normals estimated "
                      << report.normalsEstimated << ").\n";
            reconstruction::BoxCandidate candidate;
            std::string recognitionError;
            if (reconstruction::reconstructBox(
                    processed,
                    PointCloudAdapter::planeOptionsFor(processed),
                    reconstruction::BoxRecognitionOptions{},
                    candidate,
                    recognitionError)) {
                std::cout << "Recognized box " << candidate.primitive.sizeX << " x "
                          << candidate.primitive.sizeY << " x "
                          << candidate.primitive.sizeZ << ", confidence "
                          << candidate.confidence << ".\n";
            } else {
                std::cout << "No box recognized: " << recognitionError << "\n";
            }

            reconstruction::PrimitiveDetectionOptions proposalOptions;
            proposalOptions.plane = PointCloudAdapter::planeOptionsFor(processed);
            proposalOptions.cylinder.distanceThreshold =
                proposalOptions.plane.distanceThreshold;
            proposalOptions.cylinder.minimumSupportPoints =
                proposalOptions.plane.minimumSupportPoints;
            proposalOptions.sphere.distanceThreshold =
                proposalOptions.plane.distanceThreshold;
            proposalOptions.sphere.minimumSupportPoints =
                proposalOptions.plane.minimumSupportPoints;
            proposalOptions.cone.distanceThreshold =
                proposalOptions.plane.distanceThreshold;
            proposalOptions.cone.minimumSupportPoints =
                proposalOptions.plane.minimumSupportPoints;
            proposalOptions.torus.distanceThreshold =
                proposalOptions.plane.distanceThreshold;
            proposalOptions.torus.minimumSupportPoints =
                proposalOptions.plane.minimumSupportPoints;
            reconstruction::PrimitiveDetectionResult proposals;
            std::string proposalError;
            if (reconstruction::detectPrimitiveEvidence(
                    processed, proposalOptions, proposals, proposalError)) {
                std::size_t planeCount = 0;
                std::size_t cylinderCount = 0;
                std::size_t sphereCount = 0;
                std::size_t coneCount = 0;
                std::size_t torusCount = 0;
                for (const reconstruction::PrimitiveEvidence& evidence :
                     proposals.evidence) {
                    switch (reconstruction::primitiveKind(evidence)) {
                    case reconstruction::PrimitiveKind::Plane:
                        ++planeCount;
                        break;
                    case reconstruction::PrimitiveKind::Cylinder:
                        ++cylinderCount;
                        break;
                    case reconstruction::PrimitiveKind::Sphere:
                        ++sphereCount;
                        break;
                    case reconstruction::PrimitiveKind::Cone:
                        ++coneCount;
                        break;
                    case reconstruction::PrimitiveKind::Torus:
                        ++torusCount;
                        break;
                    }
                }
                std::cout << "Primitive proposals: " << planeCount
                          << " planes, " << cylinderCount << " cylinders, "
                          << sphereCount << " spheres, " << coneCount
                          << " cones, " << torusCount << " tori, "
                          << proposals.unassignedPointIds.size()
                          << " unassigned points.\n";
            } else {
                std::cout << "No primitive proposals: " << proposalError << "\n";
            }
        }
        std::cout << "Point-cloud I/O test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Point-cloud I/O test failed: " << exception.what() << '\n';
        return 1;
    }
}
