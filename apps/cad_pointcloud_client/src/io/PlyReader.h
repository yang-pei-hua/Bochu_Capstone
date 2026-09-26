#pragma once

#include <QString>

#include <array>
#include <vector>

// Raw vertex data as produced by COLMAP's PLY writer. Colors are empty when the
// file carries no per-vertex color; they are never partially filled.
struct PlyCloud
{
    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<unsigned char, 3>> colors;
};

// Minimal PLY reader. The vendored VTK ships without the IOPLY module and the
// linked module list has no FiltersGeneral, so neither vtkPLYReader nor
// vtkVertexGlyphFilter is available; reading the vertex element directly is the
// smallest way to get a cloud on screen.
//
// Supports ascii, binary_little_endian and binary_big_endian, reads vertex
// properties by name (so "x y z nx ny nz red green blue" and "x y z red green
// blue" both work), and skips every non-vertex element.
class PlyReader
{
public:
    static bool read(const QString& path, PlyCloud& cloud, QString& error);
};