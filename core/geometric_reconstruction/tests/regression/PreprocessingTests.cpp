#include "TestSupport.h"

namespace regression {
void testPreprocessingPreservesNormalOrientation() {
    reconstruction::SyntheticBoxRequest request;
    request.box.sizeX=30;request.box.sizeY=20;request.box.sizeZ=10;
    const auto source=reconstruction::makeSyntheticBoxSurface(request);
    reconstruction::PointCloudPreprocessingOptions options;
    options.removeStatisticalOutliers=false;
    reconstruction::PointStore processed({});
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(source,options,processed,report,error),error);
    require(processed.size()==source.size(),"Normal test must retain source observations");
    for(std::size_t i=0;i<source.size();++i) {
        const auto before=*source.points()[i].normal,after=*processed.points()[i].normal;
        require(before.x*after.x+before.y*after.y+before.z*after.z>0.999,
            "Preprocessing must preserve outward input normal orientation");
    }
}

void testMissingNormalsFollowSignedObservations() {
    std::vector<reconstruction::PointSample> samples;
    for (int x = 0; x < 11; ++x) for (int y = 0; y < 11; ++y) {
        reconstruction::PointSample point;
        point.id = samples.size() + 1;
        point.position = {double(x), double(y), 0};
        if (x == 5 && y == 5) point.normal = modeling::Vec3{0, 0, -2};
        samples.push_back(point);
    }
    reconstruction::PointCloudPreprocessingOptions options;
    options.removeStatisticalOutliers = false;
    options.normalNeighborCount = 8;
    const auto check = [&](const std::vector<reconstruction::PointSample>& observations) {
        reconstruction::PointStore output({});
        reconstruction::PointCloudPreprocessingReport report;
        std::string error;
        require(reconstruction::preprocessPointCloud(reconstruction::PointStore(observations),
            options, output, report, error), error);
        require(report.normalsEstimated == 120 && report.estimatedNormalsOriented == 120 &&
            report.normalsWithoutOrientation == 0, "One signed observation must orient its connected planar patch");
        for (const auto& point : output.points())
            require(point.normal && point.normal->z < -.999 && point.normalOrientationKnown,
                "Estimated sign must follow the observation, including negative outward normals");
    };
    check(samples);
    std::reverse(samples.begin(), samples.end());
    check(samples);
}

void testUnanchoredThinWallIsNotInvented() {
    std::vector<reconstruction::PointSample> samples;
    for (int layer = 0; layer < 2; ++layer)
        for (int x = 0; x < 11; ++x) for (int y = 0; y < 11; ++y) {
            reconstruction::PointSample point;
            point.id = samples.size() + 1;
            point.position = {double(x), double(y), layer * .1};
            if (!layer) point.normal = modeling::Vec3{0, 0, -1};
            samples.push_back(point);
        }
    reconstruction::PointCloudPreprocessingOptions options;
    options.removeStatisticalOutliers = false;
    reconstruction::PointStore output({});
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(reconstruction::PointStore(samples), options,
        output, report, error), error);
    require(report.normalsWithoutOrientation == 121, "A neighbouring wall must not supply an unknown wall's outward sign");
    reconstruction::CompoundCandidate candidate;
    require(!reconstruction::reconstructParametricSolid(output, {}, {}, candidate, error) &&
        error.find("orientation") != std::string::npos, "Unoriented PCA normals must not become material occupancy");
}

void testDegenerateNeighborhoodsHaveNoNormals() {
    for (const bool coincident : {false, true}) {
        std::vector<reconstruction::PointSample> samples;
        for (int i = 0; i < 25; ++i)
            samples.push_back({reconstruction::PointId(i + 1), {coincident ? 0.0 : double(i), 0, 0}, std::nullopt, 1});
        reconstruction::PointCloudPreprocessingOptions options;
        options.removeStatisticalOutliers = false;
        reconstruction::PointStore output({});
        reconstruction::PointCloudPreprocessingReport report;
        std::string error;
        require(reconstruction::preprocessPointCloud(reconstruction::PointStore(samples), options,
            output, report, error), error);
        require(report.degenerateNeighborhoods == samples.size() && report.normalsEstimated == 0,
            "Rank deficient observations must not manufacture surface normals");
        for (const auto& point : output.points())
            require(!point.normal && !point.curvature, "A line or coincident points cannot define a surface");
    }
}

void testDistantOutliersCannotMaskNearerOutliers() {
    std::vector<reconstruction::PointSample> samples;
    for (int x = 0; x < 21; ++x) for (int y = 0; y < 21; ++y)
        samples.push_back({reconstruction::PointId(samples.size() + 1), {double(x), double(y), 0}, modeling::Vec3{0, 0, 1}, 1});
    const auto surfaceCount = samples.size();
    samples.push_back({1000, {100, 100, 100}, std::nullopt, 1});
    samples.push_back({1001, {10000, 10000, 10000}, std::nullopt, 1});
    reconstruction::PointCloudPreprocessingOptions options;
    reconstruction::PointStore output({});
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(reconstruction::PointStore(samples), options,
        output, report, error), error);
    require(output.size() == surfaceCount && report.removedAsOutliers == 2,
        "Multiple distance scales must not leave a closer isolated outlier behind");
    options.protectedPointIds = {1000};
    require(reconstruction::preprocessPointCloud(reconstruction::PointStore(samples), options,
        output, report, error), error);
    require(output.size() == surfaceCount + 1 && report.removedAsOutliers == 1,
        "Explicit manual observations must survive robust outlier filtering");
}

void testVoxelIndexOverflowPreservesObservationIdentity() {
    const reconstruction::PointStore input({
        {1, {.1, .1, .1}, std::nullopt, 1},
        {2, {.2, .2, .2}, std::nullopt, 1},
        {3, {1.0e38, -1.0e38, 1.0e38}, std::nullopt, 1}});
    reconstruction::PointCloudPreprocessingOptions options;
    options.voxelSize = 1;
    options.removeStatisticalOutliers = false;
    options.estimateMissingNormals = false;
    reconstruction::PointStore output({});
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(input, options, output, report, error), error);
    require(output.size() == 2 && output.points().back().id == 3,
        "Finite observations outside the voxel index range must survive for downstream filtering");
    requireNear(output.points().back().position.x, 1.0e38, 0,
        "Extreme observation position must not be changed by voxel indexing");
}

void testScaleAwarePreprocessingAndNormalEstimation() {
    std::vector<reconstruction::PointSample> samples;
    reconstruction::PointId nextId = 1;
    for (int x = 0; x < 11; ++x) {
        for (int y = 0; y < 11; ++y) {
            samples.push_back({
                nextId++,
                {0.001 * x, 0.001 * y, 0.0},
                std::nullopt,
                1.0,
            });
        }
    }
    const reconstruction::PointId outlierId = nextId;
    samples.push_back({outlierId, {1.0, 1.0, 1.0}, std::nullopt, 1.0});
    const reconstruction::PointStore meters(
        std::move(samples), reconstruction::LengthUnit::Meter);

    reconstruction::PointCloudPreprocessingOptions options;
    options.voxelSize = 0.0;
    options.outlierNeighborCount = 8;
    options.outlierStandardDeviationMultiplier = 1.0;
    options.normalNeighborCount = 8;
    reconstruction::PointStore processed(
        {}, reconstruction::LengthUnit::Arbitrary);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(
                meters, options, processed, report, error),
            error);
    require(processed.unit() == reconstruction::LengthUnit::Millimeter,
            "Meter input must be converted to millimetres");
    requireNear(report.appliedScaleToMillimeters, 1000.0, 1.0e-12,
                "preprocessing metric scale");
    require(processed.size() == 121U,
            "The isolated point must be removed without removing the plane");
    require(report.removedAsOutliers == 1U,
            "Preprocessing must report the isolated outlier");
    for (const reconstruction::PointSample& point : processed.points()) {
        require(point.id != outlierId,
                "The isolated outlier ID must not survive preprocessing");
        require(point.normal.has_value(),
                "PCA preprocessing must estimate a plane normal");
        require(!point.normalOrientationKnown, "Unanchored PCA directions must remain explicitly unoriented");
        require(std::abs(point.normal->z) > 0.999,
                "Estimated normal must be perpendicular to the XY plane");
        require(point.curvature.has_value() && *point.curvature < 1.0e-10,
                "A noise-free plane must have near-zero curvature");
    }
    requireNear(processed.points().back().position.x, 10.0, 1.0e-12,
                "meter-to-millimetre position conversion");
}

void testVoxelDownsamplingPreservesOriginalPointIds() {
    std::vector<reconstruction::PointSample> samples{
        {10, {0.1, 0.1, 0.1}, std::nullopt, 1.0},
        {11, {0.2, 0.2, 0.2}, std::nullopt, 1.0},
        {12, {1.2, 0.2, 0.2}, std::nullopt, 1.0},
    };
    const reconstruction::PointStore input(
        std::move(samples), reconstruction::LengthUnit::Millimeter);
    reconstruction::PointCloudPreprocessingOptions options;
    options.voxelSize = 1.0;
    options.removeStatisticalOutliers = false;
    options.estimateMissingNormals = false;

    reconstruction::PointStore processed(
        {}, reconstruction::LengthUnit::Arbitrary);
    reconstruction::PointCloudPreprocessingReport report;
    std::string error;
    require(reconstruction::preprocessPointCloud(
                input, options, processed, report, error),
            error);
    require(processed.size() == 2U,
            "Two occupied voxels must produce two representatives");
    require(report.removedByVoxel == 1U,
            "Voxel report must count one removed observation");
    require(processed.points()[0].id == 11U && processed.points()[1].id == 12U,
            "Voxel representatives must retain deterministic original PointIds");
    options.protectedPointIds = {10};
    require(reconstruction::preprocessPointCloud(input, options, processed, report, error), error);
    require(processed.size() == 3 && processed.points()[0].id == 10,
            "Manual observation survives even when another sample represents the same voxel");

    std::vector<reconstruction::PointSample> dense;
    for (int x=0;x<10;++x) for (int y=0;y<10;++y)
        dense.push_back({static_cast<reconstruction::PointId>(dense.size()+1),
                         {double(x),double(y),0},modeling::Vec3{0,0,1},1});
    dense.push_back({1001,{1000,1000,0},modeling::Vec3{0,0,1},1});
    dense.push_back({1002,{0,0,0},modeling::Vec3{0,0,1},1});
    options.voxelSize = 2;
    options.removeStatisticalOutliers = true;
    options.protectedPointIds.clear();
    const reconstruction::PointStore observations(dense,reconstruction::LengthUnit::Millimeter);
    require(reconstruction::preprocessPointCloud(observations,options,processed,report,error),error);
    require(report.removedByVoxel > 0 && report.removedAsOutliers > 0,
            "Ordinary dense observations still use voxel and outlier filtering");
    options.protectedPointIds = {1001,1002};
    require(reconstruction::preprocessPointCloud(observations,options,processed,report,error),error);
    for (auto id : options.protectedPointIds)
        require(std::any_of(processed.points().begin(),processed.points().end(),
                           [id](const auto& p){return p.id==id;}),
                "Every explicit observation keeps its original ID despite voxel/outlier filtering");
    require(report.removedByVoxel > 0,
            "Protecting a manual group retains downsampling for the remaining scene");
}
} // namespace regression
