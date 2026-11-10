#include "reconstruction/recognition/ProjectionSettings.h"
#include "reconstruction/settings/ModuleConfig.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool value, const std::string& message)
{
    if (!value) throw std::runtime_error(message);
}

template <class Function>
void expectThrow(Function&& function, const std::string& message)
{
    try {
        function();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error("Expected an error: " + message);
}

void writeFile(const std::filesystem::path& path, const std::string& contents, bool bom = false)
{
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("Cannot write fixture: " + path.u8string());
    if (bom) output << "\xef\xbb\xbf";
    output << contents;
}

void setEnvironment(const char* name, const std::string& value)
{
#ifdef _WIN32
    _putenv_s(name, value.c_str());
#else
    if (value.empty()) unsetenv(name);
    else setenv(name, value.c_str(), 1);
#endif
}

}  // namespace

int main()
{
    using namespace reconstruction;
    try {
        const std::filesystem::path configDirectory(RECONSTRUCTION_CONFIG_DIR);
        settings::clearOverrides();

        // The tracked configuration files must always parse and validate.
        loadPreprocessingSettings(configDirectory / "preprocessing.conf");
        loadDetectionSettings(configDirectory / "detection.conf");
        loadAnalysisSettings(configDirectory / "analysis.conf");
        loadRecognitionSettings(configDirectory / "recognition.conf");
        loadAssemblySettings(configDirectory / "assembly.conf");
        loadProjectionOptions(configDirectory / "projection.conf");

        const auto temporary = std::filesystem::temp_directory_path() / "pcad-module-settings-test";
        std::filesystem::create_directories(temporary);
        const auto partial = temporary / "preprocessing-partial.conf";
        writeFile(partial,
            "# a comment\n"
            "\n"
            "voxel_size_mm = 3\n"
            "outlier_neighbor_count = 9\n");
        const auto preprocessing = loadPreprocessingSettings(partial);
        require(preprocessing.voxelSizeMm == 3.0, "explicit voxel size");
        require(preprocessing.options.outlierNeighborCount == 9, "explicit outlier neighbor count");
        require(preprocessing.options.outlierStandardDeviationMultiplier == 2.5,
                "unspecified values keep their defaults");

        const auto bomFile = temporary / "detection-bom.conf";
        writeFile(bomFile, "# comment\nplane_maximum_planes = 11\n", true);
        require(loadDetectionSettings(bomFile).options.plane.maximumPlanes == 11,
                "BOM and comment handling");

        const auto duplicate = temporary / "duplicate.conf";
        writeFile(duplicate, "voxel_size_mm = 1\nvoxel_size_mm = 2\n");
        expectThrow([&] { loadPreprocessingSettings(duplicate); }, "duplicate keys");

        const auto unknown = temporary / "unknown.conf";
        writeFile(unknown, "not_a_parameter = 1\n");
        expectThrow([&] { loadPreprocessingSettings(unknown); }, "unknown keys");

        const auto range = temporary / "range.conf";
        writeFile(range, "outlier_neighbor_count = 0\n");
        expectThrow([&] { loadPreprocessingSettings(range); }, "range validation");

        const auto projectionRange = temporary / "projection-range.conf";
        writeFile(projectionRange,
            "minimum_cap_points = 100\n"
            "maximum_fit_points = 150\n");
        expectThrow([&] { loadProjectionOptions(projectionRange); }, "cross-field validation");

        // Serialisation must round-trip through the same loader.
        PreprocessingSettings serializable;
        serializable.voxelSizeMm = 1.25;
        serializable.options.normalNeighborCount = 21;
        const auto roundTripFile = temporary / "round-trip.conf";
        writeFile(roundTripFile, serializePreprocessingSettings(serializable));
        const auto roundTrip = loadPreprocessingSettings(roundTripFile);
        require(roundTrip.voxelSizeMm == 1.25, "serialized voxel size");
        require(roundTrip.options.normalNeighborCount == 21, "serialized normal neighbor count");

        ProjectionExtrusionOptions projection;
        projection.maximumAxes = 9;
        projection.scoreVoxelScale = 3.5;
        const auto projectionFile = temporary / "projection-round-trip.conf";
        writeFile(projectionFile, serializeProjectionOptions(projection));
        const auto projectionRoundTrip = loadProjectionOptions(projectionFile);
        require(projectionRoundTrip.maximumAxes == 9, "serialized projection axes");
        require(projectionRoundTrip.scoreVoxelScale == 3.5, "serialized projection voxel scale");

        // Dotted overrides target one module; bare keys stay projection aliases.
        settings::addOverride("detection.plane_maximum_planes=9");
        require(loadDetectionSettings(configDirectory / "detection.conf").options.plane.maximumPlanes == 9,
                "dotted override");
        settings::addOverride("maximum_axes=7");
        require(loadProjectionOptions(configDirectory / "projection.conf").maximumAxes == 7,
                "bare projection override");
        settings::clearOverrides();

        // The environment variable takes precedence over the installed file.
        const auto environmentFile = temporary / "environment.conf";
        writeFile(environmentFile, "voxel_size_mm = 4\n");
        setEnvironment("PARAMCAD_PREPROCESSING_CONFIG", environmentFile.string());
        require(settings::configPath("preprocessing", "preprocessing.conf") == environmentFile,
                "environment override path");
        require(loadPreprocessingSettings().voxelSizeMm == 4.0, "environment override value");
        setEnvironment("PARAMCAD_PREPROCESSING_CONFIG", "");

        std::cout << "module_settings_test passed\n";
        return 0;
    } catch (const std::exception& failure) {
        std::cerr << "module_settings_test failed: " << failure.what() << '\n';
        return 1;
    }
}
