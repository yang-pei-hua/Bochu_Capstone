#include "io/StepModelExporter.h"
#include "io/StepModelLoader.h"
#include <modeling/StepExporter.h>

#include <BRepAlgoAPI_Cut.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <gp_Ax2.hxx>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>

#include <cmath>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <stdexcept>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}
double volume(const TopoDS_Shape& shape)
{
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}
QByteArray readFile(const QString& path)
{
    QFile file(path);
    require(file.open(QIODevice::ReadOnly), "read output");
    return file.readAll();
}
// Fail during serialization rather than merely rejecting a null input.
class FailingBuffer final : public std::streambuf
{
    std::streamsize xsputn(const char*, std::streamsize count) override
    {
        const auto written = std::min(count, m_remaining);
        m_remaining -= written;
        return written;
    }
    int_type overflow(int_type value) override
    {
        if (traits_type::eq_int_type(value, traits_type::eof())) return traits_type::not_eof(value);
        if (m_remaining == 0) return traits_type::eof();
        --m_remaining;
        return value;
    }
    std::streamsize m_remaining = 128;
};
}

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    try {
        QTemporaryDir temporary;
        require(temporary.isValid(), "temporary directory");
        const auto box = BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        const auto cylinder = BRepPrimAPI_MakeCylinder(
            gp_Ax2(gp_Pnt(5, 10, -1), gp_Dir(0, 0, 1)), 2, 32).Shape();
        const auto body = BRepAlgoAPI_Cut(box, cylinder).Shape();
        require(BRepCheck_Analyzer(body).IsValid(), "valid through-hole fixture");
        std::ostringstream bytes;
        std::string failure;
        require(modeling::exportStep(body, bytes, &failure) && failure.empty()
                && bytes.str().find("ISO-10303-21") != std::string::npos, "core stream serialization");
        FailingBuffer buffer;
        std::ostream failing(&buffer);
        require(!modeling::exportStep(body, failing, &failure) && !failure.empty(),
                "core reports serialization sink failure");

        const QString directory = temporary.filePath(QStringLiteral("中文目录"));
        require(QDir().mkpath(directory), "Unicode output directory");
        const QString path = QDir(directory).filePath(QStringLiteral("带孔零件.step"));
        QString error;
        require(StepModelExporter::write(body, path, error) && error.isEmpty(), "Unicode STEP export");
        StepModelLoader loader;
        LoadedModel loaded;
        if (!loader.load(path, loaded, error)) throw std::runtime_error(error.toStdString());
        require(loaded.solidCount == 1 && loaded.faceCount == 7 && BRepCheck_Analyzer(loaded.shape).IsValid(),
                "exported hole and solid topology survive client import");
        require(std::abs(volume(loaded.shape) - volume(body)) < 1.0e-6,
                "STEP round trip preserves metric volume and hole");

        const auto original = readFile(path);
        require(!StepModelExporter::write(TopoDS_Shape{}, path, error) && !error.isEmpty()
                && readFile(path) == original, "invalid export preserves existing output");
        require(!StepModelExporter::write(body, QString(), error) && !error.isEmpty(), "empty path reports error");
        require(!StepModelExporter::write(body, directory, error) && !error.isEmpty()
                && readFile(path) == original, "directory destination fails without replacing files");
        require(!StepModelExporter::write(body, temporary.filePath("missing/part.step"), error)
                && !error.isEmpty(), "missing output directory reports error");
#ifdef _WIN32
        const HANDLE lock = CreateFileW(reinterpret_cast<LPCWSTR>(path.utf16()), GENERIC_READ,
            FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        require(lock != INVALID_HANDLE_VALUE, "lock existing STEP against replacement");
        const bool replacedLockedFile = StepModelExporter::write(box, path, error);
        CloseHandle(lock);
        require(!replacedLockedFile && !error.isEmpty() && readFile(path) == original,
                "failed commit after serialization preserves the existing STEP");
#endif
        require(StepModelExporter::write(box, path, error), "atomic replacement of existing STEP");
        require(loader.load(path, loaded, error) && loaded.faceCount == 6
                && std::abs(volume(loaded.shape) - 6000) < 1.0e-6, "replacement contains current body");
        std::cout << "STEP model exporter tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
