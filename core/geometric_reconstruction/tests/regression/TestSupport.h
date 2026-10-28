#pragma once
#include "modeling/StepExporter.h"
#include "reconstruction/recognition/AxisAlignedBoxRecognizer.h"
#include "reconstruction/recognition/BoxRecognizer.h"
#include "reconstruction/modeling/ModelingAdapter.h"
#include "reconstruction/recognition/GuidedSolidRecognizer.h"
#include "reconstruction/preprocessing/PointCloudPreprocessor.h"
#include "reconstruction/detection/PrimitiveDetector.h"
#include "reconstruction/testing/SyntheticPointCloud.h"
#include "reconstruction/recognition/ThroughHoleRecognizer.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <cmath>
#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <limits>

namespace regression {
void require(bool condition, const std::string& message);

void requireNear(
    double actual,
    double expected,
    double tolerance,
    const std::string& label);

double volumeOf(const TopoDS_Shape& shape);

void testSyntheticBoxToCad(const std::filesystem::path& outputDirectory);

void testPreprocessingPreservesNormalOrientation();

void testMissingNormalsFollowSignedObservations();

void testUnanchoredThinWallIsNotInvented();

void testDegenerateNeighborhoodsHaveNoNormals();

void testDistantOutliersCannotMaskNearerOutliers();

void testVoxelIndexOverflowPreservesObservationIdentity();

void testCompoundCommitFusesSolidsAtomically();

void testRecognitionRequiresKnownScale();

void testRotatedNoisyBoxWithOutliers(
    const std::filesystem::path& outputDirectory);

void testDuplicateDepthLayersDoNotHideBoxFaces();

void testScaleAwarePreprocessingAndNormalEstimation();

void testVoxelDownsamplingPreservesOriginalPointIds();

void testCylinderPrimitiveProposal();

void testThroughHoleRecognitionAndCadCommit();

void testBoxThroughHoleEndToEnd();

void testSpherePrimitiveProposal();

void testConePrimitiveProposal();

void testTorusPrimitiveProposal();

void testCurvedFeaturesEditAndRollback();
} // namespace regression
