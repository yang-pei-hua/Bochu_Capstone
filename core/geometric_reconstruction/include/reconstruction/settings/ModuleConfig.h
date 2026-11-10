#pragma once

#include "reconstruction/analysis/SurfaceAnalysis.h"
#include "reconstruction/detection/PrimitiveDetector.h"
#include "reconstruction/preprocessing/PointCloudPreprocessor.h"
#include "reconstruction/recognition/AssemblyReconstruction.h"
#include "reconstruction/recognition/BoxRecognizer.h"
#include "reconstruction/recognition/CompoundRecognizer.h"
#include "reconstruction/recognition/CurvedSolidRecognizer.h"
#include "reconstruction/recognition/GuidedSolidRecognizer.h"
#include "reconstruction/recognition/MixedFeatureRecognizer.h"
#include "reconstruction/recognition/ThroughHoleRecognizer.h"
#include "reconstruction/settings/ModuleSettings.h"

#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace reconstruction {

// Client-side scale policy kept beside the core options so a metric and an
// arbitrary-unit cloud can share one configuration file. Zero means "derive
// from the observed bounding box or distance tolerance".
struct PreprocessingSettings {
    PointCloudPreprocessingOptions options;
    double voxelSizeMm = 0;
    double voxelDiagonalFraction = 2.0e-3;
};

struct DetectionSettings {
    PrimitiveDetectionOptions options;
    double distanceToleranceDiagonalFraction = 1.0e-3;
    std::size_t planeSupportFloor = 20;
    std::size_t planeSupportCeiling = 200;
    std::size_t planeSupportDivisor = 200;
    std::size_t cylinderSupportFloor = 30;
    std::size_t cylinderSupportCeiling = 150;
    std::size_t cylinderSupportDivisor = 500;
    double cylinderMinimumRadiusToleranceMultiplier = 2.0;
    double cylinderMaximumRadiusDiagonalFraction = 0.5;
    std::size_t curvedSupportFloor = 20;
    std::size_t curvedSupportCeiling = 200;
    std::size_t curvedSupportDivisor = 200;
};

struct AnalysisSettings {
    double connectivityDistance = 0;
    bool independentFamilies = true;
    bool recoverResidualSurfaces = false;
    double relationDistanceMm = 0;
    double relationDistanceToleranceMultiplier = 4.0;
    double relationAngularToleranceDegrees = 5.0;
    double surfaceToleranceDiagonalFraction = 0.001;
    double relationToleranceDiagonalFraction = 0.01;
    double defaultAngleToleranceDegrees = 2.0;
    double adjacentConfidence = 0.75;
    double maximumConfidence = 0.95;
    double derivedConfidenceFraction = 0.8;
    double manualPatchOverlapFraction = 0.6;
    std::size_t optimizationMaximumAttempts = 32;
    std::size_t optimizationMaximumActiveSurfaces = 12;
    std::size_t optimizationResidualSampleTarget = 256;
    double constraintWeight = 5;
    double optimizationInlierFraction = 0.9;
    double maximumMeanErrorIncreaseMultiplier = 0.15;
    double validationToleranceFraction = 0.25;
    std::size_t optimizationIterations = 15;
    double initialDamping = 1e-3;
    double convergenceStep = 1e-7;
    double supportNormalCosine = 0.9;
    std::size_t minimumRecoveryRegionPoints = 20;
    double recoveryHaloToleranceMultiplier = 2;
    double recoveryHaloRelationMultiplier = 16;
    double stablePlaneOverlapFraction = 0.1;
    double stablePlaneNormalCosine = 0.95;
    double recoveryConnectivityToleranceMultiplier = 2;
    double recoveryConnectivityAdaptiveMultiplier = 4;
};

// Quotas currently hard-coded by recognitionAnalysisOptions live in
// SurfaceAnalysis.h as RecognitionAnalysisSettings; defaults reproduce the
// previous behaviour exactly.
struct RecognitionSettings {
    BoxRecognitionOptions box;
    ThroughHoleRecognitionOptions throughHole;
    CurvedSolidRecognitionOptions curved;
    CompoundRecognitionOptions compound;
    MixedFeatureRecognitionOptions mixed;
    GuidedRecognitionOptions guided;
    RecognitionAnalysisSettings analysis;
};

// AssemblyOptions is externalised directly; projectionOptions remains a
// runtime injection point and is not written to the configuration file.
using AssemblySettings = AssemblyOptions;

std::filesystem::path preprocessingConfigPath();
std::filesystem::path detectionConfigPath();
std::filesystem::path analysisConfigPath();
std::filesystem::path recognitionConfigPath();
std::filesystem::path assemblyConfigPath();

PreprocessingSettings loadPreprocessingSettings(const std::filesystem::path& path = {});
DetectionSettings loadDetectionSettings(const std::filesystem::path& path = {});
AnalysisSettings loadAnalysisSettings(const std::filesystem::path& path = {});
RecognitionSettings loadRecognitionSettings(const std::filesystem::path& path = {});
AssemblySettings loadAssemblySettings(const std::filesystem::path& path = {});

std::string serializePreprocessingSettings(const PreprocessingSettings& settings);
std::string serializeDetectionSettings(const DetectionSettings& settings);
std::string serializeAnalysisSettings(const AnalysisSettings& settings);
std::string serializeRecognitionSettings(const RecognitionSettings& settings);
std::string serializeAssemblySettings(const AssemblySettings& settings);

void setPreprocessingSetting(PreprocessingSettings& settings, std::string_view assignment);
void setDetectionSetting(DetectionSettings& settings, std::string_view assignment);
void setAnalysisSetting(AnalysisSettings& settings, std::string_view assignment);
void setRecognitionSetting(RecognitionSettings& settings, std::string_view assignment);
void setAssemblySetting(AssemblySettings& settings, std::string_view assignment);

void validatePreprocessingSettings(const PreprocessingSettings& settings);
void validateDetectionSettings(const DetectionSettings& settings);
void validateAnalysisSettings(const AnalysisSettings& settings);
void validateRecognitionSettings(const RecognitionSettings& settings);
void validateAssemblySettings(const AssemblySettings& settings);

settings::DocumentPtr makePreprocessingSettings();
settings::DocumentPtr makeDetectionSettings();
settings::DocumentPtr makeAnalysisSettings();
settings::DocumentPtr makeRecognitionSettings();
settings::DocumentPtr makeAssemblySettings();

}  // namespace reconstruction
