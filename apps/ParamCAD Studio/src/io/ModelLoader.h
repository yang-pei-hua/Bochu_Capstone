#pragma once

#include <TopoDS_Shape.hxx>

#include <QString>

// What a loader recovers from a CAD file.
//
// The native OpenCASCADE B-Rep is what is kept - not a triangle mesh - so the
// window can still walk faces, edges and solids, recognise surfaces and write
// the model back out to STEP. The mesh the viewport draws is derived from this
// shape on demand by OcctShapeTessellator and never replaces it.
//
// Ownership: the shape is a copyable OCCT handle that keeps the underlying
// geometry alive for as long as the LoadedModel lives. Whoever holds the
// LoadedModel owns the model; the viewer only reads the shape while it
// tessellates and stores the resulting mesh, so no actor can end up pointing at
// a shape that has gone away.
struct LoadedModel {
    TopoDS_Shape shape;
    QString sourcePath;
    int solidCount = 0;
    int faceCount = 0;
    int edgeCount = 0;
    int vertexCount = 0;

    bool isValid() const noexcept { return !shape.IsNull(); }

    void reset() { *this = LoadedModel{}; }
};

// Extension point for the file formats the client can import. A loader answers
// for one family of files, decides whether a path is one of its own, and either
// returns a fully validated model or explains why it could not.
class ModelLoader
{
public:
    virtual ~ModelLoader() = default;

    // Format test on the path alone; it does not touch the file system, so it
    // stays usable for building a file dialog filter.
    virtual bool canLoad(const QString& filePath) const = 0;

    // Reads the file. On success `model` holds the loaded B-Rep and the error
    // string is empty. On failure `model` is left exactly as the caller passed
    // it - a rejected file must never disturb a model that is already loaded -
    // and `errorMessage` explains what went wrong.
    virtual bool load(const QString& filePath, LoadedModel& model,
                      QString& errorMessage) = 0;
};
