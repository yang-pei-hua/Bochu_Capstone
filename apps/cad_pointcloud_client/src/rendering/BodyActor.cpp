#include "rendering/BodyActor.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkFeatureEdges.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkTexture.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

// Projection direction of a facing direction: the axis it is most aligned with.
int dominantAxis(const double normal[3])
{
    int axis = 0;
    double best = std::fabs(normal[0]);
    for (int i = 1; i < 3; ++i) {
        const double value = std::fabs(normal[i]);
        if (value > best) {
            best = value;
            axis = i;
        }
    }
    return axis;
}

// Geometric normal of a cell, used for the faces the tessellator cannot describe
// by a plane (a curved or otherwise non-planar face).
bool cellNormal(vtkPolyData* mesh, vtkIdType cell, double normal[3])
{
    vtkIdType pointCount = 0;
    const vtkIdType* pointIds = nullptr;
    mesh->GetCellPoints(cell, pointCount, pointIds);
    if (pointCount < 3) {
        return false;
    }
    const double* a = mesh->GetPoint(pointIds[0]);
    const double* b = mesh->GetPoint(pointIds[1]);
    const double* c = mesh->GetPoint(pointIds[2]);
    const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    normal[0] = ab[1] * ac[2] - ab[2] * ac[1];
    normal[1] = ab[2] * ac[0] - ab[0] * ac[2];
    normal[2] = ab[0] * ac[1] - ab[1] * ac[0];
    const double length =
        std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (length <= 1.0e-12) {
        return false;
    }
    normal[0] /= length;
    normal[1] /= length;
    normal[2] /= length;
    return true;
}

}  // namespace

BodyActor::BodyActor()
{
    m_surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_surfaceMapper->ScalarVisibilityOff();

    m_surfaceActor = vtkSmartPointer<vtkActor>::New();
    m_surfaceActor->SetMapper(m_surfaceMapper);
    m_surfaceActor->GetProperty()->SetInterpolationToPhong();
    m_surfaceActor->SetVisibility(0);
    applyActorColor();

    // Every modeled face gets its own set of points during tessellation, so the
    // borders between faces are boundary edges rather than feature edges.
    m_featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
    m_featureEdges->FeatureEdgesOff();
    m_featureEdges->BoundaryEdgesOn();
    m_featureEdges->NonManifoldEdgesOff();
    m_featureEdges->ManifoldEdgesOff();
    m_featureEdges->ColoringOff();

    m_edgeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_edgeMapper->SetInputConnection(m_featureEdges->GetOutputPort());
    m_edgeMapper->ScalarVisibilityOff();

    m_edgeActor = vtkSmartPointer<vtkActor>::New();
    m_edgeActor->SetMapper(m_edgeMapper);
    m_edgeActor->GetProperty()->SetColor(0.06, 0.07, 0.09);
    m_edgeActor->GetProperty()->SetLineWidth(1.5);
    m_edgeActor->SetVisibility(0);

    m_highlightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_highlightMapper->ScalarVisibilityOff();
    m_highlightMapper->SetResolveCoincidentTopologyToPolygonOffset();
    m_highlightMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-2.0, -2.0);

    m_highlightActor = vtkSmartPointer<vtkActor>::New();
    m_highlightActor->SetMapper(m_highlightMapper);
    m_highlightActor->GetProperty()->SetColor(1.0, 0.65, 0.10);
    m_highlightActor->GetProperty()->SetOpacity(0.55);
    m_highlightActor->GetProperty()->SetInterpolationToFlat();
    m_highlightActor->SetVisibility(0);
}

void BodyActor::setShape(const TopoDS_Shape& shape)
{
    m_tessellation = OcctShapeTessellator::tessellate(shape);
    m_hasShape = !m_tessellation.empty();

    // Only the mapper input is refreshed so the camera stays where the user
    // left it; the tessellator returns a brand new data object every time. A
    // texture outlives a rebuild, so its projection is recomputed from the new
    // mesh here as well.
    m_featureEdges->SetInputData(m_tessellation.mesh);
    m_featureEdges->Update();
    refreshSurfaceInput();

    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    clearHighlight();
    updateEdgeVisibility();
}

void BodyActor::clear()
{
    setShape(TopoDS_Shape());
}

bool BodyActor::hasShape() const noexcept
{
    return m_hasShape;
}

void BodyActor::setVisible(bool visible)
{
    m_visible = visible;
    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    m_highlightActor->SetVisibility(
        m_visible && m_hasShape && m_highlightedFace >= 0 ? 1 : 0);
    updateEdgeVisibility();
}

void BodyActor::setRepresentation(int representation)
{
    m_representation = representation;
    switch (representation) {
    case 1:
        m_surfaceActor->GetProperty()->SetRepresentationToWireframe();
        break;
    case 2:
        m_surfaceActor->GetProperty()->SetRepresentationToPoints();
        m_surfaceActor->GetProperty()->SetPointSize(4.0F);
        break;
    default:
        m_surfaceActor->GetProperty()->SetRepresentationToSurface();
        break;
    }
    updateEdgeVisibility();
}

void BodyActor::setOpacity(double opacity)
{
    m_surfaceActor->GetProperty()->SetOpacity(std::clamp(opacity, 0.0, 1.0));
}

void BodyActor::setColor(const QColor& color)
{
    m_color = color;
    applyActorColor();
}

void BodyActor::setLightingEnabled(bool enabled)
{
    m_surfaceActor->GetProperty()->SetLighting(enabled ? 1 : 0);
}

bool BodyActor::setTextureFromFile(const QString& imagePath, QString& error)
{
    const QByteArray fileName = imagePath.toUtf8();
    vtkSmartPointer<vtkImageReader2> reader =
        vtkImageReader2Factory::CreateImageReader2(fileName.constData());
    if (reader == nullptr) {
        error = QStringLiteral("Unsupported texture format: %1").arg(imagePath);
        return false;
    }
    reader->SetFileName(fileName.constData());
    reader->Update();
    vtkImageData* image = reader->GetOutput();
    if (image == nullptr || image->GetNumberOfPoints() == 0) {
        error = QStringLiteral("Could not read the texture image: %1").arg(imagePath);
        return false;
    }

    m_textureReader = reader;
    m_texture = vtkSmartPointer<vtkTexture>::New();
    m_texture->SetInputConnection(m_textureReader->GetOutputPort());
    m_texture->InterpolateOn();
    // The coordinates are normalised to the bounding box, so the picture covers
    // the body exactly once and never tiles across it.
    m_texture->RepeatOff();
    m_textureEnabled = true;
    refreshSurfaceInput();
    return true;
}

void BodyActor::setTextureEnabled(bool enabled)
{
    if (m_textureEnabled == enabled) {
        return;
    }
    m_textureEnabled = enabled;
    refreshSurfaceInput();
}

void BodyActor::setTextureProjection(int axis)
{
    const int clamped = std::clamp(axis, 0, 4);
    if (m_textureAxis == clamped) {
        return;
    }
    m_textureAxis = clamped;
    if (m_textureEnabled && m_texture != nullptr) {
        refreshSurfaceInput();
    }
}

void BodyActor::clearTexture()
{
    m_textureEnabled = false;
    m_texture = nullptr;
    m_textureReader = nullptr;
    refreshSurfaceInput();
    m_texturedMesh = nullptr;
}

bool BodyActor::hasTexture() const noexcept
{
    return m_texture != nullptr;
}

int BodyActor::textureProjection() const noexcept
{
    return m_textureAxis;
}

void BodyActor::refreshSurfaceInput()
{
    const bool textured = m_textureEnabled && m_texture != nullptr && m_hasShape;
    if (textured) {
        m_texturedMesh = projectTextureCoordinates(m_tessellation.mesh, m_textureAxis);
        m_surfaceMapper->SetInputData(m_texturedMesh);
        m_surfaceActor->SetTexture(m_texture);
    } else {
        m_surfaceActor->SetTexture(nullptr);
        m_surfaceMapper->SetInputData(m_tessellation.mesh);
    }
    applyActorColor();
    m_surfaceMapper->Update();
}

void BodyActor::applyActorColor()
{
    if (m_textureEnabled && m_texture != nullptr && m_hasShape) {
        // The picture is the colour of the surface, so the object tint drops to
        // white; anything else would multiply the texture and wash it out.
        m_surfaceActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
        return;
    }
    m_surfaceActor->GetProperty()->SetColor(m_color.redF(), m_color.greenF(), m_color.blueF());
}

vtkSmartPointer<vtkPolyData> BodyActor::projectTextureCoordinates(vtkPolyData* mesh,
                                                                 int axis) const
{
    auto projected = vtkSmartPointer<vtkPolyData>::New();
    if (mesh == nullptr) {
        return projected;
    }
    // The points and polygons are shared, but the point data is copied into a
    // fresh container so the texture coordinates never leak into the mesh the
    // face index and the outline are read from.
    projected->SetPoints(mesh->GetPoints());
    projected->SetPolys(mesh->GetPolys());
    projected->GetPointData()->ShallowCopy(mesh->GetPointData());

    vtkPoints* points = mesh->GetPoints();
    if (points == nullptr || points->GetNumberOfPoints() == 0) {
        return projected;
    }

    double bounds[6];
    mesh->GetBounds(bounds);
    double extent[3] = {bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]};

    // A perfectly flat body would divide by zero; a zero-sized side keeps its
    // coordinates at the lower bound instead.
    const double size[3] = {extent[0] > 1.0e-9 ? extent[0] : 1.0,
                            extent[1] > 1.0e-9 ? extent[1] : 1.0,
                            extent[2] > 1.0e-9 ? extent[2] : 1.0};

    // Axis 0 means "let the body decide": the picture is thrown at it from the
    // side it is widest, which keeps the projection from smearing on a thin
    // direction. 1/2/3 pin the direction to X/Y/Z. 4 projects every face from
    // the direction that face looks at.
    int autoAxis = 0;
    for (int i = 1; i < 3; ++i) {
        if (extent[i] > extent[autoAxis]) {
            autoAxis = i;
        }
    }

    // Projects one point from the given direction. Every direction normalises
    // against the whole body's bounds, so the faces of a box stay in register
    // with each other instead of each stretching the picture across itself.
    const auto project = [&](const double* point, int normalAxis, double& u, double& v) {
        const int uAxis = (normalAxis + 1) % 3;
        const int vAxis = (normalAxis + 2) % 3;
        u = (point[uAxis] - bounds[2 * uAxis]) / size[uAxis];
        v = (point[vAxis] - bounds[2 * vAxis]) / size[vAxis];
    };

    vtkNew<vtkFloatArray> coordinates;
    coordinates->SetName("TextureCoordinates");
    coordinates->SetNumberOfComponents(2);
    coordinates->SetNumberOfTuples(points->GetNumberOfPoints());

    if (axis != 4) {
        const int normalAxis = axis >= 1 ? axis - 1 : autoAxis;
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            double u = 0.0;
            double v = 0.0;
            project(points->GetPoint(i), normalAxis, u, v);
            coordinates->SetTuple2(i, u, v);
        }
        projected->GetPointData()->SetTCoords(coordinates);
        return projected;
    }

    // Per-face (tri-planar) mode. A single direction cannot face all six sides
    // of a box, so the sides it misses are smeared; picking the direction from
    // each face's own normal gives every side a square-on projection. The
    // tessellator gives every face its own points, so one direction per point is
    // enough and no vertex has to be split.
    vtkDataArray* faceIds = mesh->GetCellData()->GetArray("FaceIds");
    std::vector<int> pointAxis(static_cast<std::size_t>(points->GetNumberOfPoints()), -1);
    for (vtkIdType cell = 0; cell < mesh->GetNumberOfCells(); ++cell) {
        double normal[3] = {0.0, 0.0, 1.0};
        const int faceIndex =
            faceIds != nullptr && cell < faceIds->GetNumberOfTuples()
                ? static_cast<int>(faceIds->GetComponent(cell, 0))
                : -1;
        const BodyFaceInfo* info = faceInfo(faceIndex);
        if (info != nullptr && info->planar) {
            std::copy(info->normal, info->normal + 3, normal);
        } else if (!cellNormal(mesh, cell, normal)) {
            continue;
        }

        const int faceAxis = dominantAxis(normal);
        vtkIdType pointCount = 0;
        const vtkIdType* pointIds = nullptr;
        mesh->GetCellPoints(cell, pointCount, pointIds);
        for (vtkIdType k = 0; k < pointCount; ++k) {
            if (pointAxis[static_cast<std::size_t>(pointIds[k])] < 0) {
                pointAxis[static_cast<std::size_t>(pointIds[k])] = faceAxis;
            }
        }
    }

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        const int pointDirection = pointAxis[static_cast<std::size_t>(i)];
        double u = 0.0;
        double v = 0.0;
        project(points->GetPoint(i), pointDirection >= 0 ? pointDirection : autoAxis, u, v);
        coordinates->SetTuple2(i, u, v);
    }
    projected->GetPointData()->SetTCoords(coordinates);
    return projected;
}

void BodyActor::setTransform(double px, double py, double pz,
                             double rx, double ry, double rz,
                             double sx, double sy, double sz)
{
    vtkActor* actors[] = {m_surfaceActor, m_edgeActor, m_highlightActor};
    for (vtkActor* actor : actors) {
        actor->SetPosition(px, py, pz);
        actor->SetOrientation(rx, ry, rz);
        actor->SetScale(sx, sy, sz);
    }
}

vtkActor* BodyActor::surfaceActor() const noexcept
{
    return m_surfaceActor;
}

vtkActor* BodyActor::edgeActor() const noexcept
{
    return m_edgeActor;
}

int BodyActor::faceCount() const noexcept
{
    return static_cast<int>(m_tessellation.faces.size());
}

const BodyFaceInfo* BodyActor::faceInfo(int faceIndex) const noexcept
{
    if (faceIndex < 0 || faceIndex >= faceCount()) {
        return nullptr;
    }
    return &m_tessellation.faces[static_cast<std::size_t>(faceIndex)];
}

int BodyActor::faceIdOfCell(vtkIdType cellId) const noexcept
{
    if (m_tessellation.mesh == nullptr || cellId < 0) {
        return -1;
    }
    vtkDataArray* ids = m_tessellation.mesh->GetCellData()->GetArray("FaceIds");
    if (ids == nullptr || cellId >= ids->GetNumberOfTuples()) {
        return -1;
    }
    return static_cast<int>(ids->GetComponent(cellId, 0));
}

void BodyActor::highlightFace(int faceIndex)
{
    const BodyFaceInfo* info = faceInfo(faceIndex);
    if (info == nullptr || info->cellCount <= 0) {
        clearHighlight();
        return;
    }

    vtkPolyData* source = m_tessellation.mesh;
    vtkNew<vtkCellArray> selection;
    const vtkIdType lastCell = info->firstCell + info->cellCount;
    for (vtkIdType cell = info->firstCell; cell < lastCell; ++cell) {
        vtkIdType pointCount = 0;
        const vtkIdType* pointIds = nullptr;
        source->GetCellPoints(cell, pointCount, pointIds);
        selection->InsertNextCell(pointCount, pointIds);
    }

    vtkNew<vtkPolyData> overlay;
    overlay->SetPoints(source->GetPoints());
    overlay->SetPolys(selection);
    m_highlightMapper->SetInputData(overlay);

    m_highlightedFace = faceIndex;
    m_highlightActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
}

void BodyActor::clearHighlight()
{
    m_highlightedFace = -1;
    m_highlightMapper->SetInputData(vtkSmartPointer<vtkPolyData>::New());
    m_highlightActor->SetVisibility(0);
}

vtkActor* BodyActor::highlightActor() const noexcept
{
    return m_highlightActor;
}

void BodyActor::updateEdgeVisibility()
{
    const bool showEdges = m_visible && m_hasShape && m_representation == 0;
    m_edgeActor->SetVisibility(showEdges ? 1 : 0);
}
