#include "modeling/StepExporter.h"

#include "occt/OcctError.h"

#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <STEPControl_StepModelType.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>

#include <system_error>
#include <ostream>

namespace modeling {

namespace {
template<class Write>
bool writeStep(const TopoDS_Shape& shape, Write write, std::string* errorText) {
    const auto fail = [errorText](const std::string& message) {
        if (errorText != nullptr) *errorText = message;
        return false;
    };
    if (shape.IsNull()) return fail("Cannot export a null shape");
    try {
        Interface_Static::SetCVal("write.step.schema", "AP214IS");
        STEPControl_Writer writer;
        if (writer.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone)
            return fail("OpenCASCADE could not transfer the shape to STEP");
        if (!write(writer)) return fail("OpenCASCADE could not write the STEP file");
        if (errorText != nullptr) errorText->clear();
        return true;
    } catch (const Standard_Failure& failure) {
        return fail(std::string("OpenCASCADE STEP export failure: ") +
                    occt::failureMessage(failure));
    }
}
}  // namespace

bool exportStep(
    const TopoDS_Shape& shape,
    const std::filesystem::path& outputPath,
    std::string* errorText) {
    const auto fail = [errorText](const std::string& message) {
        if (errorText != nullptr) {
            *errorText = message;
        }
        return false;
    };

    if (shape.IsNull()) {
        return fail("Cannot export a null shape");
    }

    try {
        std::error_code filesystemError;
        if (outputPath.has_parent_path()) {
            std::filesystem::create_directories(
                outputPath.parent_path(), filesystemError);
            if (filesystemError) {
                return fail("Could not create STEP output directory: " +
                    filesystemError.message());
            }
        }

        const std::string nativePath = outputPath.string();
        return writeStep(shape, [&nativePath](STEPControl_Writer& writer) {
            return writer.Write(nativePath.c_str()) == IFSelect_RetDone;
        }, errorText);
    } catch (const Standard_Failure& failure) {
        return fail(std::string("OpenCASCADE STEP export failure: ") +
            occt::failureMessage(failure));
    }
}

bool exportStep(const TopoDS_Shape& shape, std::ostream& output, std::string* errorText) {
    return writeStep(shape, [&output](STEPControl_Writer& writer) {
        return writer.WriteStream(output) == IFSelect_RetDone && output.good();
    }, errorText);
}

}  // namespace modeling
