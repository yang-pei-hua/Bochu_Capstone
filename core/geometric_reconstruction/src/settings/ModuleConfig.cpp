#include "reconstruction/settings/ModuleConfig.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace reconstruction {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesPerRadian = 180.0 / kPi;
constexpr double kRadiansPerDegree = kPi / 180.0;

#define REAL(field, key, group, label, low, high) \
    {key, group, label, T::Type::Real, low, high, \
     [](const S& s) -> double { return double(s.field); }, \
     [](S& s, double v) { s.field = static_cast<decltype(s.field)>(v); }}
#define INT(field, key, group, label, low, high) \
    {key, group, label, T::Type::Integer, low, high, \
     [](const S& s) -> double { return double(s.field); }, \
     [](S& s, double v) { s.field = static_cast<decltype(s.field)>(std::llround(v)); }}
#define BOOL(field, key, group, label) \
    {key, group, label, T::Type::Boolean, 0, 1, \
     [](const S& s) -> double { return s.field ? 1.0 : 0.0; }, \
     [](S& s, double v) { s.field = v != 0; }}
#define DEG(field, key, group, label) \
    {key, group, label, T::Type::Real, 0, 180, \
     [](const S& s) -> double { return double(s.field) * kDegreesPerRadian; }, \
     [](S& s, double v) { s.field = v * kRadiansPerDegree; }}
#define RADIUS_MAX(field, key, group, label) \
    {key, group, label, T::Type::Real, 0, 1000000, \
     [](const S& s) -> double { const auto value = s.field; return std::isfinite(value) ? double(value) : 0.0; }, \
     [](S& s, double v) { s.field = v > 0 ? v : std::numeric_limits<double>::infinity(); }}

const std::vector<settings::Setting<PreprocessingSettings>>& preprocessingSchema()
{
    using S = PreprocessingSettings;
    using T = settings::Setting<S>;
    static const std::vector<T> values{
        REAL(voxelSizeMm, "voxel_size_mm", "降采样", "体素尺寸 mm，0 为自动", 0, 10000),
        REAL(voxelDiagonalFraction, "voxel_diagonal_fraction", "降采样", "自动体素尺寸占包围盒对角线比例", 0.000001, 0.5),
        BOOL(options.removeStatisticalOutliers, "remove_statistical_outliers", "离群过滤", "启用统计离群过滤"),
        INT(options.outlierNeighborCount, "outlier_neighbor_count", "离群过滤", "离群过滤近邻数", 1, 1000),
        REAL(options.outlierStandardDeviationMultiplier, "outlier_standard_deviation_multiplier", "离群过滤", "邻距标准差倍数", 0, 100),
        REAL(options.outlierRobustMedianMultiplier, "outlier_robust_median_multiplier", "离群过滤", "稳健中位数距离倍数上限", 0, 100),
        BOOL(options.estimateMissingNormals, "estimate_missing_normals", "法向与曲率", "估计缺失法向"),
        INT(options.normalNeighborCount, "normal_neighbor_count", "法向与曲率", "法向估计近邻数", 3, 1000),
        REAL(options.normalOrientationMinimumAlignment, "normal_orientation_minimum_alignment", "法向与曲率", "方向传播最小对齐度", 0, 1),
        REAL(options.normalOrientationMaximumTangentDeviation, "normal_orientation_maximum_tangent_deviation", "法向与曲率", "方向传播最大切向偏离", 0, 10),
        REAL(options.normalOrientationMaximumSpacingRatio, "normal_orientation_maximum_spacing_ratio", "法向与曲率", "方向传播最大相邻间距比", 0, 100),
        REAL(options.normalOrientationDistanceCostWeight, "normal_orientation_distance_cost_weight", "法向与曲率", "方向传播距离代价权重", 0, 100),
        BOOL(options.convertKnownUnitsToMillimeters, "convert_known_units_to_millimeters", "单位", "已知单位转换为毫米"),
    };
    return values;
}

const std::vector<settings::Setting<DetectionSettings>>& detectionSchema()
{
    using S = DetectionSettings;
    using T = settings::Setting<S>;
    static const std::vector<T> values{
        INT(options.randomSeed, "random_seed", "通用", "随机种子", 0, 4294967295),
        BOOL(options.detectPlanes, "detect_planes", "通用", "检测平面"),
        BOOL(options.detectCylinders, "detect_cylinders", "通用", "检测圆柱"),
        BOOL(options.detectSpheres, "detect_spheres", "通用", "检测球"),
        BOOL(options.detectCones, "detect_cones", "通用", "检测圆锥"),
        BOOL(options.detectTori, "detect_tori", "通用", "检测圆环"),
        INT(options.multiScaleRounds, "multi_scale_rounds", "多尺度", "残余多尺度轮数", 0, 16),
        REAL(options.multiScaleInitialAlignment, "multi_scale_initial_alignment", "多尺度", "首轮区域法向对齐余弦", 0, 1),
        REAL(options.multiScaleRecoveryAlignment, "multi_scale_recovery_alignment", "多尺度", "恢复轮区域法向对齐余弦", 0, 1),
        INT(options.multiScaleNeighborCount, "multi_scale_neighbor_count", "多尺度", "区域聚类近邻数", 1, 256),
        INT(options.multiScaleMinimumSupportFloor, "multi_scale_minimum_support_floor", "多尺度", "多尺度最低支持点数下限", 1, 100000),
        REAL(distanceToleranceDiagonalFraction, "distance_tolerance_diagonal_fraction", "尺度自适应", "自动距离容差占包围盒对角线比例", 0.000001, 0.5),
        INT(planeSupportFloor, "plane_support_floor", "尺度自适应", "平面最低支持点下限", 6, 100000),
        INT(planeSupportCeiling, "plane_support_ceiling", "尺度自适应", "平面最低支持点上限", 6, 1000000),
        INT(planeSupportDivisor, "plane_support_divisor", "尺度自适应", "平面支持点采样除数", 1, 1000000),
        INT(cylinderSupportFloor, "cylinder_support_floor", "尺度自适应", "圆柱最低支持点下限", 6, 100000),
        INT(cylinderSupportCeiling, "cylinder_support_ceiling", "尺度自适应", "圆柱最低支持点上限", 6, 1000000),
        INT(cylinderSupportDivisor, "cylinder_support_divisor", "尺度自适应", "圆柱支持点采样除数", 1, 1000000),
        REAL(cylinderMinimumRadiusToleranceMultiplier, "cylinder_minimum_radius_tolerance_multiplier", "尺度自适应", "圆柱最小半径与容差之比", 0, 100),
        REAL(cylinderMaximumRadiusDiagonalFraction, "cylinder_maximum_radius_diagonal_fraction", "尺度自适应", "圆柱最大半径占对角线比例", 0.000001, 1),
        INT(curvedSupportFloor, "curved_support_floor", "尺度自适应", "球/圆锥/圆环支持点下限", 6, 100000),
        INT(curvedSupportCeiling, "curved_support_ceiling", "尺度自适应", "球/圆锥/圆环支持点上限", 6, 1000000),
        INT(curvedSupportDivisor, "curved_support_divisor", "尺度自适应", "球/圆锥/圆环支持点采样除数", 1, 1000000),
        REAL(options.plane.distanceThreshold, "plane_distance_threshold_mm", "平面", "距离容差 mm，0 为自动", 0, 10000),
        INT(options.plane.minimumSupportPoints, "plane_minimum_support_points", "平面", "最低支持点数，0 为自动", 0, 2000000),
        INT(options.plane.maximumPlanes, "plane_maximum_planes", "平面", "最多平面数", 1, 4096),
        REAL(options.plane.probability, "plane_probability", "平面", "RANSAC 概率", 0.000001, 1),
        REAL(options.plane.minimumNormalAlignment, "plane_minimum_normal_alignment", "平面", "最低法向一致度", 0, 1),
        REAL(options.plane.clusterEpsilon, "plane_cluster_epsilon", "平面", "聚类半径，0 为自动", 0, 10000),
        DEG(options.plane.duplicatePlaneAngularToleranceRadians, "plane_duplicate_angular_tolerance_deg", "平面", "重复平面角度容差"),
        REAL(options.plane.duplicatePlaneDistanceMultiplier, "plane_duplicate_distance_multiplier", "平面", "重复平面距离容差倍数", 0, 100),
        INT(options.plane.randomSeed, "plane_random_seed", "平面", "平面随机种子", 0, 4294967295),
        REAL(options.cylinder.distanceThreshold, "cylinder_distance_threshold_mm", "圆柱", "距离容差 mm，0 为自动", 0, 10000),
        INT(options.cylinder.minimumSupportPoints, "cylinder_minimum_support_points", "圆柱", "最低支持点数，0 为自动", 0, 2000000),
        INT(options.cylinder.maximumCylinders, "cylinder_maximum_cylinders", "圆柱", "最多圆柱数", 1, 4096),
        REAL(options.cylinder.probability, "cylinder_probability", "圆柱", "RANSAC 概率", 0.000001, 1),
        REAL(options.cylinder.minimumNormalAlignment, "cylinder_minimum_normal_alignment", "圆柱", "最低法向一致度", 0, 1),
        REAL(options.cylinder.clusterEpsilon, "cylinder_cluster_epsilon", "圆柱", "聚类半径，0 为自动", 0, 10000),
        REAL(options.cylinder.minimumRadius, "cylinder_minimum_radius_mm", "圆柱", "最小半径 mm，0 为自动", 0, 10000),
        RADIUS_MAX(options.cylinder.maximumRadius, "cylinder_maximum_radius_mm", "圆柱", "最大半径 mm，0 为自动"),
        INT(options.cylinder.randomSeed, "cylinder_random_seed", "圆柱", "圆柱随机种子", 0, 4294967295),
        REAL(options.sphere.distanceThreshold, "sphere_distance_threshold_mm", "球", "距离容差 mm，0 为自动", 0, 10000),
        INT(options.sphere.minimumSupportPoints, "sphere_minimum_support_points", "球", "最低支持点数，0 为自动", 0, 2000000),
        INT(options.sphere.maximumSpheres, "sphere_maximum_spheres", "球", "最多球数", 1, 4096),
        REAL(options.sphere.probability, "sphere_probability", "球", "RANSAC 概率", 0.000001, 1),
        REAL(options.sphere.minimumNormalAlignment, "sphere_minimum_normal_alignment", "球", "最低法向一致度", 0, 1),
        REAL(options.sphere.clusterEpsilon, "sphere_cluster_epsilon", "球", "聚类半径，0 为自动", 0, 10000),
        REAL(options.sphere.minimumRadius, "sphere_minimum_radius_mm", "球", "最小半径 mm，0 为自动", 0, 10000),
        RADIUS_MAX(options.sphere.maximumRadius, "sphere_maximum_radius_mm", "球", "最大半径 mm，0 为自动"),
        REAL(options.cone.distanceThreshold, "cone_distance_threshold_mm", "圆锥", "距离容差 mm，0 为自动", 0, 10000),
        INT(options.cone.minimumSupportPoints, "cone_minimum_support_points", "圆锥", "最低支持点数，0 为自动", 0, 2000000),
        INT(options.cone.maximumCones, "cone_maximum_cones", "圆锥", "最多圆锥数", 1, 4096),
        REAL(options.cone.probability, "cone_probability", "圆锥", "RANSAC 概率", 0.000001, 1),
        REAL(options.cone.minimumNormalAlignment, "cone_minimum_normal_alignment", "圆锥", "最低法向一致度", 0, 1),
        REAL(options.cone.clusterEpsilon, "cone_cluster_epsilon", "圆锥", "聚类半径，0 为自动", 0, 10000),
        DEG(options.cone.minimumOpeningAngleRadians, "cone_minimum_opening_angle_deg", "圆锥", "最小开口角"),
        DEG(options.cone.maximumOpeningAngleRadians, "cone_maximum_opening_angle_deg", "圆锥", "最大开口角"),
        REAL(options.torus.distanceThreshold, "torus_distance_threshold_mm", "圆环", "距离容差 mm，0 为自动", 0, 10000),
        INT(options.torus.minimumSupportPoints, "torus_minimum_support_points", "圆环", "最低支持点数，0 为自动", 0, 2000000),
        INT(options.torus.maximumTori, "torus_maximum_tori", "圆环", "最多圆环数", 1, 4096),
        REAL(options.torus.probability, "torus_probability", "圆环", "RANSAC 概率", 0.000001, 1),
        REAL(options.torus.minimumNormalAlignment, "torus_minimum_normal_alignment", "圆环", "最低法向一致度", 0, 1),
        REAL(options.torus.clusterEpsilon, "torus_cluster_epsilon", "圆环", "聚类半径，0 为自动", 0, 10000),
        REAL(options.torus.minimumMajorRadius, "torus_minimum_major_radius_mm", "圆环", "最小主半径 mm", 0, 10000),
        RADIUS_MAX(options.torus.maximumMajorRadius, "torus_maximum_major_radius_mm", "圆环", "最大主半径 mm，0 为不限制"),
        REAL(options.torus.minimumMinorRadius, "torus_minimum_minor_radius_mm", "圆环", "最小次半径 mm", 0, 10000),
        RADIUS_MAX(options.torus.maximumMinorRadius, "torus_maximum_minor_radius_mm", "圆环", "最大次半径 mm，0 为不限制"),
    };
    return values;
}

const std::vector<settings::Setting<AnalysisSettings>>& analysisSchema()
{
    using S = AnalysisSettings;
    using T = settings::Setting<S>;
    static const std::vector<T> values{
        REAL(connectivityDistance, "connectivity_distance_mm", "连接", "连通距离 mm，0 为自动", 0, 10000),
        BOOL(independentFamilies, "independent_families", "连接", "曲面族独立检测"),
        BOOL(recoverResidualSurfaces, "recover_residual_surfaces", "连接", "恢复残余曲面"),
        REAL(relationDistanceMm, "relation_distance_mm", "关系推断", "关系距离容差 mm，0 为自动", 0, 10000),
        REAL(relationDistanceToleranceMultiplier, "relation_distance_tolerance_multiplier", "关系推断", "关系容差与拟合容差之比", 0.01, 100),
        REAL(relationAngularToleranceDegrees, "relation_angular_tolerance_deg", "关系推断", "关系角度容差", 0.001, 180),
        REAL(surfaceToleranceDiagonalFraction, "surface_tolerance_diagonal_fraction", "界面默认容差", "曲面拟合默认容差占对角线比例", 0.000001, 0.5),
        REAL(relationToleranceDiagonalFraction, "relation_tolerance_diagonal_fraction", "界面默认容差", "关系默认容差占对角线比例", 0.000001, 0.5),
        REAL(defaultAngleToleranceDegrees, "default_angle_tolerance_deg", "界面默认容差", "关系默认角度容差", 0.01, 30),
        REAL(adjacentConfidence, "relation_adjacent_confidence", "关系推断", "相邻关系置信度", 0, 1),
        REAL(maximumConfidence, "relation_maximum_confidence", "关系推断", "关系置信度上限", 0, 1),
        REAL(derivedConfidenceFraction, "relation_derived_confidence_fraction", "关系推断", "派生关系置信度比例", 0, 1),
        REAL(manualPatchOverlapFraction, "relation_manual_patch_overlap_fraction", "关系推断", "人工关系面片最低重叠比例", 0, 1),
        INT(optimizationMaximumAttempts, "relation_optimization_maximum_attempts", "关系优化", "关系优化最大尝试数", 1, 100000),
        INT(optimizationMaximumActiveSurfaces, "relation_optimization_maximum_active_surfaces", "关系优化", "关系优化最多活跃曲面数", 1, 1000),
        INT(optimizationResidualSampleTarget, "relation_optimization_residual_sample_target", "关系优化", "残差采样目标数", 1, 100000),
        REAL(constraintWeight, "relation_constraint_weight", "关系优化", "约束残差权重", 0, 100),
        REAL(optimizationInlierFraction, "relation_optimization_inlier_fraction", "关系优化", "优化最低内点比例", 0, 1),
        REAL(maximumMeanErrorIncreaseMultiplier, "relation_maximum_mean_error_increase_multiplier", "关系优化", "平均误差允许增加倍率", 0, 100),
        REAL(validationToleranceFraction, "relation_validation_tolerance_fraction", "关系优化", "关系复验容差比例", 0, 1),
        INT(optimizationIterations, "relation_optimization_iterations", "关系优化", "关系优化迭代次数", 1, 1000),
        REAL(initialDamping, "relation_initial_damping", "关系优化", "初始阻尼", 0.000000001, 100),
        REAL(convergenceStep, "relation_convergence_step", "关系优化", "收敛步长", 0.000000000001, 1),
        REAL(supportNormalCosine, "relation_support_normal_cosine", "关系优化", "支持点法向余弦", 0, 1),
        INT(minimumRecoveryRegionPoints, "minimum_recovery_region_points", "局部恢复", "局部恢复区域最低点数", 1, 1000000),
        REAL(recoveryHaloToleranceMultiplier, "recovery_halo_tolerance_multiplier", "局部恢复", "恢复搜索容差倍率", 0, 100),
        REAL(recoveryHaloRelationMultiplier, "recovery_halo_relation_multiplier", "局部恢复", "恢复搜索关系容差倍率", 0, 100),
        REAL(stablePlaneOverlapFraction, "stable_plane_overlap_fraction", "局部恢复", "稳定平面最大重叠比例", 0, 1),
        REAL(stablePlaneNormalCosine, "stable_plane_normal_cosine", "局部恢复", "稳定平面法向余弦", 0, 1),
        REAL(recoveryConnectivityToleranceMultiplier, "recovery_connectivity_tolerance_multiplier", "局部恢复", "恢复连通距离倍率", 0, 100),
        REAL(recoveryConnectivityAdaptiveMultiplier, "recovery_connectivity_adaptive_multiplier", "局部恢复", "恢复自适应连通倍率", 0, 100),
    };
    return values;
}

const std::vector<settings::Setting<RecognitionSettings>>& recognitionSchema()
{
    using S = RecognitionSettings;
    using T = settings::Setting<S>;
    static const std::vector<T> values{
        INT(analysis.maximumPlanes, "analysis_maximum_planes", "分析配额", "平面候选上限", 1, 4096),
        INT(analysis.maximumCylinders, "analysis_maximum_cylinders", "分析配额", "圆柱候选上限", 1, 4096),
        INT(analysis.planeSupportCap, "analysis_plane_support_cap", "分析配额", "平面支持点合并上限", 1, 1000000),
        INT(analysis.cylinderSupportCap, "analysis_cylinder_support_cap", "分析配额", "圆柱支持点合并上限", 1, 1000000),
        INT(analysis.curvedSupportFloor, "analysis_curved_support_floor", "分析配额", "曲面支持点下限", 1, 1000000),
        INT(analysis.curvedSupportCeiling, "analysis_curved_support_ceiling", "分析配额", "曲面支持点上限", 1, 1000000),
        INT(analysis.curvedMaximumCandidates, "analysis_curved_maximum_candidates", "分析配额", "曲面候选上限", 1, 4096),
        REAL(analysis.curvedProbability, "analysis_curved_probability", "分析配额", "曲面 RANSAC 概率", 0.000001, 1),
        REAL(analysis.relationDistanceToleranceMultiplier, "analysis_relation_distance_tolerance_multiplier", "分析配额", "关系容差与识别容差之比", 0.01, 100),
        DEG(box.angularToleranceRadians, "box_angular_tolerance_deg", "箱体", "平行/垂直角度容差"),
        DEG(throughHole.angularToleranceRadians, "through_hole_angular_tolerance_deg", "通孔", "轴线对齐角度容差"),
        REAL(throughHole.minimumAngularCoverage, "through_hole_minimum_angular_coverage", "通孔", "最低角度覆盖率", 0, 1),
        REAL(throughHole.minimumAxialCoverage, "through_hole_minimum_axial_coverage", "通孔", "最低轴向覆盖率", 0, 1),
        REAL(throughHole.maximumEndpointGapFraction, "through_hole_maximum_endpoint_gap_fraction", "通孔", "端部与外表最大间隙比例", 0, 1),
        REAL(throughHole.minimumBoundaryClearance, "through_hole_minimum_boundary_clearance", "通孔", "边界最小余量 mm", 0, 10000),
        REAL(curved.distanceTolerance, "curved_distance_tolerance_mm", "曲面实体", "拟合距离容差 mm", 0, 10000),
        REAL(curved.minimumSurfaceCoverage, "curved_minimum_surface_coverage", "曲面实体", "最低曲面覆盖率", 0, 1),
        REAL(curved.minimumBoundaryCoverage, "curved_minimum_boundary_coverage", "曲面实体", "最低边界覆盖率", 0, 1),
        REAL(curved.minimumNormalAlignment, "curved_minimum_normal_alignment", "曲面实体", "最低法向一致度", 0, 1),
        INT(curved.randomSeed, "curved_random_seed", "曲面实体", "随机种子", 0, 4294967295),
        REAL(curved.boundaryNormalCosine, "curved_boundary_normal_cosine", "曲面实体", "边界法向余弦", 0, 1),
        REAL(curved.capNormalCosine, "curved_cap_normal_cosine", "曲面实体", "端面法向余弦", 0, 1),
        REAL(curved.assessDistanceMultiplier, "curved_assess_distance_multiplier", "曲面实体", "拟合判定距离容差倍率", 0, 100),
        REAL(curved.apexToleranceMultiplier, "curved_apex_tolerance_multiplier", "曲面实体", "锥顶观测容差倍率", 0, 100),
        REAL(curved.capRadialToleranceMultiplier, "curved_cap_radial_tolerance_multiplier", "曲面实体", "端面径向容差倍率", 0, 100),
        REAL(curved.capGroupToleranceMultiplier, "curved_cap_group_tolerance_multiplier", "曲面实体", "端面分组容差倍率", 0, 100),
        INT(curved.minimumCapPoints, "curved_minimum_cap_points", "曲面实体", "端面最低点数", 1, 1000000),
        INT(curved.capPointDivisor, "curved_cap_point_divisor", "曲面实体", "端面点数采样除数", 1, 1000000),
        INT(curved.sphereAngularBins, "curved_sphere_angular_bins", "曲面实体", "球角度分箱数", 1, 512),
        INT(curved.sphereAxialBins, "curved_sphere_axial_bins", "曲面实体", "球轴向分箱数", 1, 128),
        INT(curved.torusAngularBins, "curved_torus_angular_bins", "曲面实体", "圆环角度分箱数", 1, 512),
        INT(curved.torusTubeBins, "curved_torus_tube_bins", "曲面实体", "圆环管向分箱数", 1, 128),
        INT(curved.curvedSideAngularBins, "curved_side_angular_bins", "曲面实体", "侧壁角度分箱数", 1, 512),
        INT(curved.curvedSideAxialBins, "curved_side_axial_bins", "曲面实体", "侧壁轴向分箱数", 1, 128),
        INT(curved.capAngularBins, "curved_cap_angular_bins", "曲面实体", "端面角度分箱数", 1, 512),
        INT(curved.capRadialBins, "curved_cap_radial_bins", "曲面实体", "端面径向分箱数", 1, 128),
        INT(curved.curvedCylinderSupportFloor, "curved_cylinder_support_floor", "曲面实体", "圆柱检测支持点下限", 1, 1000000),
        INT(curved.curvedCylinderSupportDivisor, "curved_cylinder_support_divisor", "曲面实体", "圆柱检测支持点采样除数", 1, 1000000),
        REAL(curved.curvedCylinderProbability, "curved_cylinder_probability", "曲面实体", "圆柱检测 RANSAC 概率", 0.000001, 1),
        REAL(curved.coneStartToleranceMultiplier, "curved_cone_start_tolerance_multiplier", "曲面实体", "圆锥起点容差倍率", 0, 100),
        REAL(curved.residualScoreWeight, "curved_residual_score_weight", "曲面实体", "残差评分权重", 0, 100),
        REAL(compound.distanceTolerance, "compound_distance_tolerance_mm", "组合体", "距离容差 mm", 0, 10000),
        REAL(compound.normalAlignment, "compound_normal_alignment", "组合体", "法向一致度", 0, 1),
        REAL(compound.maximumCellError, "compound_maximum_cell_error", "组合体", "材料单元最大误差 mm", 0, 10000),
        INT(compound.maximumGridCells, "compound_maximum_grid_cells", "组合体", "材料网格单元上限", 1, 10000000),
        INT(compound.maximumSearchNodes, "compound_maximum_search_nodes", "组合体", "覆盖搜索节点上限", 1, 100000000),
        INT(compound.minimumPoints, "compound_minimum_points", "组合体", "最低输入点数", 1, 1000000),
        INT(compound.minimumSupportPoints, "compound_minimum_support_points", "组合体", "方向/层最低支持点数", 1, 1000000),
        INT(compound.supportDivisor, "compound_support_divisor", "组合体", "支持点采样除数", 1, 1000000),
        REAL(compound.frameDistanceToleranceMultiplier, "compound_frame_distance_tolerance_multiplier", "组合体", "框架距离容差倍率", 0, 100),
        REAL(compound.levelMergeToleranceMultiplier, "compound_level_merge_tolerance_multiplier", "组合体", "层合并容差倍率", 0, 100),
        REAL(compound.orthogonalCosine, "compound_orthogonal_cosine", "组合体", "正交方向最大余弦", 0, 1),
        INT(compound.maximumBoundaries, "compound_maximum_boundaries", "组合体", "每轴最大边界层数", 2, 64),
        REAL(compound.extrusionAxisCosine, "compound_extrusion_axis_cosine", "组合体", "斜切拉伸轴最大余弦", 0, 1),
        INT(compound.maximumObliques, "compound_maximum_obliques", "组合体", "最大斜切面数", 0, 64),
        REAL(compound.coverageShiftToleranceMultiplier, "compound_coverage_shift_tolerance_multiplier", "组合体", "覆盖率采样偏移容差倍率", 0, 100),
        REAL(compound.minimumSurfaceCoverage, "compound_minimum_surface_coverage", "组合体", "最低表面覆盖率", 0, 1),
        INT(compound.cellSamplesPerAxis, "compound_cell_samples_per_axis", "组合体", "材料单元每轴采样数", 1, 16),
        REAL(mixed.hostOpposingPlaneCosine, "mixed_host_opposing_plane_cosine", "混合特征", "宿主平行面对向余弦", 0, 1),
        REAL(mixed.hostDuplicateSupportFraction, "mixed_host_duplicate_support_fraction", "混合特征", "宿主重复面最低支持点比例", 0, 1),
        REAL(mixed.hostPlaneDistanceMultiplier, "mixed_host_plane_distance_multiplier", "混合特征", "宿主平行面距离容差倍率", 0, 100),
        REAL(mixed.hostOutsidePlaneMultiplier, "mixed_host_outside_plane_multiplier", "混合特征", "宿主外侧点判定容差倍率", 0, 100),
        REAL(mixed.hostSideBandMultiplier, "mixed_host_side_band_multiplier", "混合特征", "宿主侧面包带容差倍率", 0, 100),
        INT(mixed.minimumHostPoints, "mixed_minimum_host_points", "混合特征", "宿主最低点数", 1, 1000000),
        REAL(mixed.maximumOpposingPointFraction, "mixed_maximum_opposing_point_fraction", "混合特征", "宿主反向点最大比例", 0, 1),
        REAL(mixed.hostSideNormalCosine, "mixed_host_side_normal_cosine", "混合特征", "宿主侧面法向最大轴向余弦", 0, 1),
        INT(mixed.minimumSidePoints, "mixed_minimum_side_points", "混合特征", "侧面最低点数", 1, 1000000),
        REAL(mixed.minimumSideFraction, "mixed_minimum_side_fraction", "混合特征", "侧面最低支持点比例", 0, 1),
        REAL(mixed.hostSideAxisCosine, "mixed_host_side_axis_cosine", "混合特征", "宿主侧面轴对齐余弦", 0, 1),
        REAL(mixed.maximumThicknessRatio, "mixed_maximum_thickness_ratio", "混合特征", "宿主最大宽厚比", 0, 10000),
        REAL(mixed.hostConfidence, "mixed_host_confidence", "混合特征", "宿主置信度", 0, 1),
        INT(mixed.minimumEvidenceSupportPoints, "mixed_minimum_evidence_support_points", "混合特征", "局部证据最低支持点数", 1, 1000000),
        REAL(mixed.holeInwardNormalFraction, "mixed_hole_inward_normal_fraction", "混合特征", "孔壁内向法向最低比例", 0, 1),
        REAL(mixed.aboveSideToleranceFraction, "mixed_above_side_tolerance_fraction", "混合特征", "宿主上方侧面判定容差比例", 0, 1),
        REAL(mixed.aboveSideNormalCosine, "mixed_above_side_normal_cosine", "混合特征", "宿主上方侧面法向余弦", 0, 1),
        REAL(mixed.knownSurfaceToleranceMultiplier, "mixed_known_surface_tolerance_multiplier", "混合特征", "已知曲面判定容差倍率", 0, 100),
        REAL(mixed.knownSurfaceNormalCosine, "mixed_known_surface_normal_cosine", "混合特征", "已知曲面法向余弦", 0, 1),
        REAL(mixed.connectivityToleranceMultiplier, "mixed_connectivity_tolerance_multiplier", "混合特征", "连通距离容差倍率", 0, 100),
        REAL(mixed.connectivitySpacingMultiplier, "mixed_connectivity_spacing_multiplier", "混合特征", "连通距离点间距倍率", 0, 100),
        REAL(mixed.connectivityAdaptiveMultiplier, "mixed_connectivity_adaptive_multiplier", "混合特征", "自适应连通距离倍率", 0, 100),
        REAL(mixed.coverageShiftToleranceMultiplier, "mixed_coverage_shift_tolerance_multiplier", "混合特征", "覆盖率采样偏移容差倍率", 0, 100),
        INT(mixed.minimumLocalPoints, "mixed_minimum_local_points", "混合特征", "局部特征最低点数", 1, 1000000),
        REAL(mixed.capNormalCosine, "mixed_cap_normal_cosine", "混合特征", "端面法向余弦", 0, 1),
        INT(mixed.minimumCapPoints, "mixed_minimum_cap_points", "混合特征", "端面最低点数", 1, 1000000),
        REAL(mixed.minimumSurfaceCoverage, "mixed_minimum_surface_coverage", "混合特征", "混合模型最低表面覆盖率", 0, 1),
        REAL(mixed.bossAxisCosine, "mixed_boss_axis_cosine", "混合特征", "凸台轴线对齐余弦", 0, 1),
        REAL(mixed.bossRadiusToleranceMultiplier, "mixed_boss_radius_tolerance_multiplier", "混合特征", "凸台半径容差倍率", 0, 100),
        REAL(mixed.bossBandToleranceMultiplier, "mixed_boss_band_tolerance_multiplier", "混合特征", "凸台匹配带容差倍率", 0, 100),
        REAL(mixed.bossCapNormalCosine, "mixed_boss_cap_normal_cosine", "混合特征", "凸台端面法向余弦", 0, 1),
        INT(mixed.bossMinimumCapPoints, "mixed_boss_minimum_cap_points", "混合特征", "凸台端面最低点数", 1, 1000000),
        REAL(mixed.bossMinimumHeightMultiplier, "mixed_boss_minimum_height_multiplier", "混合特征", "凸台最低高度容差倍率", 0, 100),
        REAL(mixed.bossSideNormalCosine, "mixed_boss_side_normal_cosine", "混合特征", "凸台侧壁法向余弦", 0, 1),
        REAL(mixed.bossReferenceAxisCosine, "mixed_boss_reference_axis_cosine", "混合特征", "凸台参考轴选择余弦", 0, 1),
        INT(mixed.bossDepthBins, "mixed_boss_depth_bins", "混合特征", "凸台深度分箱数", 1, 128),
        INT(mixed.bossAngularBins, "mixed_boss_angular_bins", "混合特征", "凸台角度分箱数", 1, 512),
        INT(mixed.bossCapSectors, "mixed_boss_cap_sectors", "混合特征", "凸台端面扇区数", 1, 512),
        INT(mixed.bossRadialBins, "mixed_boss_radial_bins", "混合特征", "凸台端面径向分箱数", 1, 128),
        REAL(mixed.minimumBossCoverage, "mixed_minimum_boss_coverage", "混合特征", "凸台最低覆盖率", 0, 1),
        REAL(mixed.minimumBossBoundary, "mixed_minimum_boss_boundary", "混合特征", "凸台最低边界覆盖率", 0, 1),
        REAL(mixed.coplanarNormalCosine, "mixed_coplanar_normal_cosine", "混合特征", "共面法向最大轴向余弦", 0, 1),
        REAL(mixed.coplanarAxisCosine, "mixed_coplanar_axis_cosine", "混合特征", "共面轴对齐余弦", 0, 1),
        REAL(mixed.cutWallToleranceMultiplier, "mixed_cut_wall_tolerance_multiplier", "混合特征", "切槽壁面容差倍率", 0, 100),
        INT(mixed.minimumWalls, "mixed_minimum_walls", "混合特征", "切槽最低壁面数", 1, 64),
        INT(mixed.maximumWalls, "mixed_maximum_walls", "混合特征", "切槽最多壁面数", 1, 256),
        REAL(mixed.cutLevelAxisCosine, "mixed_cut_level_axis_cosine", "混合特征", "切槽层轴对齐余弦", 0, 1),
        REAL(mixed.cutMinimumSizeMultiplier, "mixed_cut_minimum_size_multiplier", "混合特征", "切槽最小尺寸容差倍率", 0, 100),
        REAL(mixed.cutSideNormalCosine, "mixed_cut_side_normal_cosine", "混合特征", "切槽侧壁法向余弦", 0, 1),
        REAL(mixed.cutSpanToleranceMultiplier, "mixed_cut_span_tolerance_multiplier", "混合特征", "切槽跨度容差倍率", 0, 100),
        INT(guided.minimumTemplatePoints, "guided_minimum_template_points", "引导模板", "模板选区最低点数", 1, 1000000),
        INT(guided.templateBoxMinimumSupportPoints, "guided_template_box_minimum_support_points", "引导模板", "箱体模板最低支持点数", 1, 1000000),
        REAL(guided.boundaryToleranceMultiplier, "guided_boundary_tolerance_multiplier", "引导模板", "边界匹配容差倍率", 0, 100),
        REAL(guided.boundaryNormalCosine, "guided_boundary_normal_cosine", "引导模板", "边界法向余弦", 0, 1),
        REAL(guided.minimumSupportFraction, "guided_minimum_support_fraction", "引导模板", "模板最低支持比例", 0, 1),
        REAL(guided.relationToleranceMultiplier, "guided_relation_tolerance_multiplier", "引导模板", "局部关系容差倍率", 0, 100),
        INT(guided.minimumExpandedPoints, "guided_minimum_expanded_points", "引导模板", "曲面扩展最低点数", 1, 1000000),
        REAL(guided.minimumSurfaceSupportFraction, "guided_minimum_surface_support_fraction", "引导模板", "曲面模板最低支持比例", 0, 1),
        INT(guided.templateDetectionMinimumSupportPoints, "guided_template_detection_minimum_support_points", "引导模板", "模板检测最低支持点数", 1, 1000000),
        REAL(guided.capNormalCosine, "guided_cap_normal_cosine", "引导模板", "扩展端面法向余弦", 0, 1),
        REAL(guided.sideNormalCosine, "guided_side_normal_cosine", "引导模板", "扩展侧壁法向余弦", 0, 1),
        REAL(guided.capMarginFraction, "guided_cap_margin_fraction", "引导模板", "端面扩展余量比例", 0, 1),
        REAL(guided.minimumMatchFraction, "guided_minimum_match_fraction", "引导模板", "模板复验最低匹配比例", 0, 1),
        INT(guided.minimumSeedPoints, "guided_minimum_seed_points", "引导模板", "剩余种子最低点数", 1, 1000000),
    };
    return values;
}

const std::vector<settings::Setting<AssemblySettings>>& assemblySchema()
{
    using S = AssemblySettings;
    using T = settings::Setting<S>;
    static const std::vector<T> values{
        INT(maximumWorkingPoints, "maximum_working_points", "采样", "工作采样点上限", 6, 100000000),
        INT(neighbors, "neighbors", "采样", "邻域点数", 6, 128),
        INT(minimumPatchPoints, "minimum_patch_points", "采样", "面片最低点数", 6, 1000000),
        REAL(distanceTolerance, "distance_tolerance_mm", "容差", "距离容差 mm，0 为自动", 0, 10000),
        DEG(normalAngleRadians, "normal_angle_deg", "容差", "法向夹角容差"),
        REAL(connectivityFactor, "connectivity_factor", "容差", "连通距离系数", 0.000001, 100),
        REAL(alphaFactor, "alpha_factor", "容差", "Alpha 半径系数", 0.000001, 100),
        INT(maximumPrimitiveAttempts, "maximum_primitive_attempts", "面片与闭合", "基元尝试上限", 1, 1000000),
        BOOL(buildSolids, "build_solids", "面片与闭合", "构建并验证实体"),
        INT(maximumLocalProfileAttempts, "maximum_local_profile_attempts", "局部恢复", "局部轮廓尝试上限", 0, 1000000),
        BOOL(enableSpecializedRecovery, "enable_specialized_recovery", "局部恢复", "启用专用局部恢复"),
        INT(maximumProjectionParts, "maximum_projection_parts", "投影回退", "投影回退部件上限，0 为关闭", 0, 1000),
        INT(minimumPartPatches, "minimum_part_patches", "采样", "部件最低面片数", 1, 128),
        INT(maximumPartPatches, "maximum_part_patches", "采样", "部件最多面片数", 1, 128),
        INT(minimumProjectionPartPoints, "minimum_projection_part_points", "投影回退", "投影回退部件最低点数", 1, 1000000),
        INT(minimumPartPointsForLocalRecovery, "minimum_part_points_for_local_recovery", "局部恢复", "局部恢复部件最低点数", 1, 1000000),
        REAL(patchSpacingToleranceMultiplier, "patch_spacing_tolerance_multiplier", "容差", "面片点间距容差倍率", 0, 100),
        REAL(connectionToleranceMultiplier, "connection_tolerance_multiplier", "容差", "连接判定容差倍率", 0, 100),
        REAL(minimumConvexLinkFraction, "minimum_convex_link_fraction", "容差", "凸连接最低比例", 0, 1),
        INT(minimumLinkCount, "minimum_link_count", "容差", "最低连接数", 1, 1000),
        INT(maximumClosurePatches, "maximum_closure_patches", "面片与闭合", "闭合面片上限", 1, 100000),
        INT(maximumClosureVertices, "maximum_closure_vertices", "面片与闭合", "闭合顶点上限", 1, 1000000),
        REAL(weldToleranceMultiplier, "weld_tolerance_multiplier", "面片与闭合", "焊接容差倍率", 0, 100),
        REAL(minimumPartSpanMultiplier, "minimum_part_span_multiplier", "面片与闭合", "部件最小跨度容差倍率", 0, 100),
        INT(minimumClosureSamples, "minimum_closure_samples", "面片与闭合", "闭合最低采样点数", 1, 1000000),
        INT(maximumClosureSamples, "maximum_closure_samples", "面片与闭合", "闭合最多采样点数", 1, 1000000),
        REAL(closureSampleToleranceMultiplier, "closure_sample_tolerance_multiplier", "面片与闭合", "闭合采样容差倍率", 0, 100),
        REAL(curvedClosureCoverage, "curved_closure_coverage", "面片与闭合", "曲面闭合最低覆盖率", 0, 1),
        INT(minimumClosureConflicts, "minimum_closure_conflicts", "面片与闭合", "闭合冲突最低计数", 1, 1000000),
        INT(closureConflictPointDivisor, "closure_conflict_point_divisor", "面片与闭合", "闭合冲突点数除数", 1, 1000000),
        INT(minimumProfileSplitPoints, "minimum_profile_split_points", "局部恢复", "轮廓拆分最低点数", 1, 1000000),
        INT(minimumParentSplitPoints, "minimum_parent_split_points", "局部恢复", "父部件拆分最低点数", 1, 1000000),
        INT(minimumLargeParentPoints, "minimum_large_parent_points", "局部恢复", "大部件最低点数", 1, 1000000),
        INT(minimumHugeParentPoints, "minimum_huge_parent_points", "局部恢复", "超大部件最低点数", 1, 1000000),
        INT(minimumProfileSamples, "minimum_profile_samples", "局部恢复", "轮廓恢复最低采样点数", 1, 1000000),
        INT(minimumLargeProfileSamples, "minimum_large_profile_samples", "局部恢复", "大轮廓恢复最低采样点数", 1, 1000000),
        REAL(minimumRetainedFraction, "minimum_retained_fraction", "局部恢复", "裁剪保留最低比例", 0, 1),
        INT(minimumResidualSamples, "minimum_residual_samples", "局部恢复", "残余最低新增点数", 1, 1000000),
        REAL(minimumParentSupportFraction, "minimum_parent_support_fraction", "局部恢复", "父部件最低支持比例", 0, 1),
        INT(minimumPlaneCount, "minimum_plane_count", "局部恢复", "通道分析最低平面数", 1, 1000),
        INT(maximumPlaneCount, "maximum_plane_count", "局部恢复", "通道分析最多平面数", 1, 1000),
        INT(maximumBroadPairs, "maximum_broad_pairs", "局部恢复", "宽面配对上限", 1, 1000),
        INT(maximumProfilePairs, "maximum_profile_pairs", "局部恢复", "轮廓配对上限", 1, 1000),
    };
    return values;
}

void validatePreprocessing(const PreprocessingSettings& settings, std::string& reason)
{
    if (settings.voxelSizeMm > 0 && settings.voxelDiagonalFraction <= 0)
        reason = "voxel_diagonal_fraction must be positive";
}

void validateDetection(const DetectionSettings& settings, std::string& reason)
{
    if (settings.planeSupportFloor > settings.planeSupportCeiling)
        reason = "plane_support_floor cannot exceed plane_support_ceiling";
    else if (settings.cylinderSupportFloor > settings.cylinderSupportCeiling)
        reason = "cylinder_support_floor cannot exceed cylinder_support_ceiling";
    else if (settings.curvedSupportFloor > settings.curvedSupportCeiling)
        reason = "curved_support_floor cannot exceed curved_support_ceiling";
    else if (settings.options.cone.minimumOpeningAngleRadians >= settings.options.cone.maximumOpeningAngleRadians)
        reason = "cone_minimum_opening_angle_deg must be below cone_maximum_opening_angle_deg";
    else if (settings.options.torus.minimumMajorRadius > settings.options.torus.maximumMajorRadius)
        reason = "torus_minimum_major_radius_mm cannot exceed torus_maximum_major_radius_mm";
    else if (settings.options.torus.minimumMinorRadius > settings.options.torus.maximumMinorRadius)
        reason = "torus_minimum_minor_radius_mm cannot exceed torus_maximum_minor_radius_mm";
}

void validateAnalysis(const AnalysisSettings&, std::string&) {}

void validateRecognition(const RecognitionSettings& settings, std::string& reason)
{
    if (settings.analysis.curvedSupportFloor > settings.analysis.curvedSupportCeiling)
        reason = "analysis_curved_support_floor cannot exceed analysis_curved_support_ceiling";
    else if (settings.mixed.minimumWalls > settings.mixed.maximumWalls)
        reason = "mixed_minimum_walls cannot exceed mixed_maximum_walls";
    else if (settings.mixed.bossCapSectors > settings.mixed.bossAngularBins)
        reason = "mixed_boss_cap_sectors cannot exceed mixed_boss_angular_bins";
}

void validateAssembly(const AssemblySettings& settings, std::string& reason)
{
    if (settings.minimumPatchPoints < 6)
        reason = "minimum_patch_points must be at least 6";
    else if (settings.neighbors < 6 || settings.neighbors > 128)
        reason = "neighbors must be between 6 and 128";
    else if (settings.maximumWorkingPoints < settings.minimumPatchPoints)
        reason = "maximum_working_points cannot be below minimum_patch_points";
    else if (!std::isfinite(settings.distanceTolerance) || settings.distanceTolerance < 0)
        reason = "distance_tolerance_mm must be finite and non-negative";
    else if (!std::isfinite(settings.normalAngleRadians) || settings.normalAngleRadians <= 0
             || settings.normalAngleRadians > 0.7854)
        reason = "normal_angle_deg must be between 0 and 45 degrees";
    else if (!std::isfinite(settings.connectivityFactor) || settings.connectivityFactor <= 0)
        reason = "connectivity_factor must be positive";
    else if (!std::isfinite(settings.alphaFactor) || settings.alphaFactor <= 0)
        reason = "alpha_factor must be positive";
}

#undef REAL
#undef INT
#undef BOOL
#undef DEG
#undef RADIUS_MAX

}  // namespace

std::filesystem::path preprocessingConfigPath() { return settings::configPath("preprocessing", "preprocessing.conf"); }
std::filesystem::path detectionConfigPath() { return settings::configPath("detection", "detection.conf"); }
std::filesystem::path analysisConfigPath() { return settings::configPath("analysis", "analysis.conf"); }
std::filesystem::path recognitionConfigPath() { return settings::configPath("recognition", "recognition.conf"); }
std::filesystem::path assemblyConfigPath() { return settings::configPath("assembly", "assembly.conf"); }

PreprocessingSettings loadPreprocessingSettings(const std::filesystem::path& path)
{
    return settings::loadOptions(preprocessingSchema(), "preprocessing", "preprocessing.conf", path, validatePreprocessing);
}

DetectionSettings loadDetectionSettings(const std::filesystem::path& path)
{
    return settings::loadOptions(detectionSchema(), "detection", "detection.conf", path, validateDetection);
}

AnalysisSettings loadAnalysisSettings(const std::filesystem::path& path)
{
    return settings::loadOptions(analysisSchema(), "analysis", "analysis.conf", path, validateAnalysis);
}

RecognitionSettings loadRecognitionSettings(const std::filesystem::path& path)
{
    return settings::loadOptions(recognitionSchema(), "recognition", "recognition.conf", path, validateRecognition);
}

AssemblySettings loadAssemblySettings(const std::filesystem::path& path)
{
    return settings::loadOptions(assemblySchema(), "assembly", "assembly.conf", path, validateAssembly);
}

std::string serializePreprocessingSettings(const PreprocessingSettings& value)
{
    validatePreprocessingSettings(value);
    return settings::serializeOptions(preprocessingSchema(), value, "Point cloud preprocessing");
}

std::string serializeDetectionSettings(const DetectionSettings& value)
{
    validateDetectionSettings(value);
    return settings::serializeOptions(detectionSchema(), value, "Surface detection");
}

std::string serializeAnalysisSettings(const AnalysisSettings& value)
{
    validateAnalysisSettings(value);
    return settings::serializeOptions(analysisSchema(), value, "Surface analysis");
}

std::string serializeRecognitionSettings(const RecognitionSettings& value)
{
    validateRecognitionSettings(value);
    return settings::serializeOptions(recognitionSchema(), value, "Parametric recognition");
}

std::string serializeAssemblySettings(const AssemblySettings& value)
{
    validateAssemblySettings(value);
    return settings::serializeOptions(assemblySchema(), value, "Assembly reconstruction");
}

void setPreprocessingSetting(PreprocessingSettings& value, std::string_view assignment)
{
    settings::setOption(preprocessingSchema(), value, assignment, "preprocessing");
}

void setDetectionSetting(DetectionSettings& value, std::string_view assignment)
{
    settings::setOption(detectionSchema(), value, assignment, "detection");
}

void setAnalysisSetting(AnalysisSettings& value, std::string_view assignment)
{
    settings::setOption(analysisSchema(), value, assignment, "analysis");
}

void setRecognitionSetting(RecognitionSettings& value, std::string_view assignment)
{
    settings::setOption(recognitionSchema(), value, assignment, "recognition");
}

void setAssemblySetting(AssemblySettings& value, std::string_view assignment)
{
    settings::setOption(assemblySchema(), value, assignment, "assembly");
}

void validatePreprocessingSettings(const PreprocessingSettings& value)
{
    settings::validateOptions(preprocessingSchema(), value, "preprocessing", validatePreprocessing);
}

void validateDetectionSettings(const DetectionSettings& value)
{
    settings::validateOptions(detectionSchema(), value, "detection", validateDetection);
}

void validateAnalysisSettings(const AnalysisSettings& value)
{
    settings::validateOptions(analysisSchema(), value, "analysis", validateAnalysis);
}

void validateRecognitionSettings(const RecognitionSettings& value)
{
    settings::validateOptions(recognitionSchema(), value, "recognition", validateRecognition);
}

void validateAssemblySettings(const AssemblySettings& value)
{
    settings::validateOptions(assemblySchema(), value, "assembly", validateAssembly);
}

settings::DocumentPtr makePreprocessingSettings()
{
    return settings::makeDocument(preprocessingSchema(), "preprocessing", "点云预处理", "preprocessing.conf", validatePreprocessing);
}

settings::DocumentPtr makeDetectionSettings()
{
    return settings::makeDocument(detectionSchema(), "detection", "曲面检测", "detection.conf", validateDetection);
}

settings::DocumentPtr makeAnalysisSettings()
{
    return settings::makeDocument(analysisSchema(), "analysis", "曲面关系", "analysis.conf", validateAnalysis);
}

settings::DocumentPtr makeRecognitionSettings()
{
    return settings::makeDocument(recognitionSchema(), "recognition", "常规识别", "recognition.conf", validateRecognition);
}

settings::DocumentPtr makeAssemblySettings()
{
    return settings::makeDocument(assemblySchema(), "assembly", "装配分解", "assembly.conf", validateAssembly);
}

}  // namespace reconstruction
