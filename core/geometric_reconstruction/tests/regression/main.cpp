#include "TestSupport.h"
using namespace regression;

int main() {
    try {
        testSyntheticBoxToCad(RECONSTRUCTION_TEST_OUTPUT_DIR);
        testPreprocessingPreservesNormalOrientation();
        testMissingNormalsFollowSignedObservations();
        testUnanchoredThinWallIsNotInvented();
        testDegenerateNeighborhoodsHaveNoNormals();
        testDistantOutliersCannotMaskNearerOutliers();
        testVoxelIndexOverflowPreservesObservationIdentity();
        testCompoundCommitFusesSolidsAtomically();
        testRecognitionRequiresKnownScale();
        testRotatedNoisyBoxWithOutliers(RECONSTRUCTION_TEST_OUTPUT_DIR);
        testDuplicateDepthLayersDoNotHideBoxFaces();
        testScaleAwarePreprocessingAndNormalEstimation();
        testVoxelDownsamplingPreservesOriginalPointIds();
        testCylinderPrimitiveProposal();
        testThroughHoleRecognitionAndCadCommit();
        testBoxThroughHoleEndToEnd();
        testSpherePrimitiveProposal();
        testConePrimitiveProposal();
        testTorusPrimitiveProposal();
        testCurvedFeaturesEditAndRollback();
        std::cout << "Geometric reconstruction vertical-slice test passed.\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Geometric reconstruction test failed: "
                  << exception.what() << '\n';
        return 1;
    }
}
