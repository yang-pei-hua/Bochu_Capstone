#include "reconstruction/recognition/ProjectionSettings.h"

#include <string>

namespace reconstruction {
namespace {

void validateProjection(const ProjectionExtrusionOptions& options, std::string& reason)
{
    if (options.minimumCapPoints > options.maximumFitPoints / 2)
        reason = "maximum_fit_points must be at least twice minimum_cap_points";
    else if (options.innerWallMinimumBands > options.innerWallDepthBands)
        reason = "inner_wall_minimum_bands cannot exceed inner_wall_depth_bands";
}

const std::vector<ProjectionSetting>& projectionSchema()
{
#define SETTING(field, key, group, label, type, low, high) \
    {key, group, label, ProjectionSetting::Type::type, low, high, \
     [](const ProjectionExtrusionOptions& options) { return double(options.field); }, \
     [](ProjectionExtrusionOptions& options, double value) { options.field = static_cast<decltype(options.field)>(value); }}
    static const std::vector<ProjectionSetting> values{
        SETTING(distanceTolerance, "distance_tolerance_mm", "输入与候选轴", "绝对距离容差 mm，0 为自动", Real, 0, 10000),
        SETTING(relativeDistanceTolerance, "relative_distance_tolerance", "输入与候选轴", "自动容差占包围盒对角线比例", Real, 0.000001, 0.5),
        SETTING(maximumAxes, "maximum_axes", "输入与候选轴", "候选轴上限", Integer, 1, 128),
        SETTING(maximumFitPoints, "maximum_fit_points", "输入与候选轴", "拟合采样点上限", Integer, 6, 2000000),
        SETTING(maximumScorePoints, "maximum_score_points", "输入与候选轴", "评分采样点上限", Integer, 1, 200000),
        SETTING(scoreVoxelScale, "score_voxel_scale", "输入与候选轴", "评分体素密度倍率", Real, 0.01, 100),
        SETTING(includeAreaDirections, "include_area_directions", "输入与候选轴", "启用投影面积搜索方向", Boolean, 0, 1),
        SETTING(maximumEvidencePlanes, "maximum_evidence_planes", "输入与候选轴", "平面证据上限", Integer, 1, 128),
        SETTING(maximumEvidenceCylinders, "maximum_evidence_cylinders", "输入与候选轴", "圆柱证据上限", Integer, 1, 128),
        SETTING(evidenceSupportDivisor, "evidence_support_divisor", "输入与候选轴", "曲面证据最低支持点数的采样除数", Integer, 1, 1000000),
        SETTING(axisAlignmentCosine, "axis_alignment_cosine", "输入与候选轴", "轴对齐余弦阈值", Real, 0.01, 1),
        SETTING(sideMomentNormalCosine, "side_moment_normal_cosine", "输入与候选轴", "侧壁法向矩允许的最大轴向余弦", Real, 0, 1),
        SETTING(sideMomentEigenGap, "side_moment_eigen_gap", "输入与候选轴", "侧壁法向矩的最小特征值差比例", Real, 0, 1),
        SETTING(areaSamplePoints, "area_sample_points", "输入与候选轴", "投影面积计算的采样点上限", Integer, 3, 200000),
        SETTING(areaSampleDirections, "area_sample_directions", "输入与候选轴", "投影面积搜索的球面方向数", Integer, 1, 1024),
        SETTING(maximumAreaDirections, "maximum_area_directions", "输入与候选轴", "最多追加的面积搜索方向数", Integer, 0, 128),
        SETTING(extentTrimFraction, "extent_trim_fraction", "输入与候选轴", "每端裁掉的深度样本比例", Real, 0, 0.2),
        SETTING(minimumCapPoints, "minimum_cap_points", "端面与草图", "端面最低点数", Integer, 3, 200000),
        SETTING(capBandMultiplier, "cap_band_multiplier", "端面与草图", "端面深度带宽与距离容差之比", Real, 0.01, 100),
        SETTING(capNormalCosine, "cap_normal_cosine", "端面与草图", "端面法向最小轴向余弦", Real, 0, 1),
        SETTING(minimumDepthMultiplier, "minimum_depth_multiplier", "端面与草图", "拉伸最低深度与距离容差之比", Real, 0.01, 100),
        SETTING(capAlphaSpacingMultiplier, "cap_alpha_spacing_multiplier", "端面与草图", "端面 Alpha 的点间距倍率", Real, 0.01, 100),
        SETTING(capAlphaToleranceMultiplier, "cap_alpha_tolerance_multiplier", "端面与草图", "端面 Alpha 的容差倍率", Real, 0.01, 100),
        SETTING(capSimplificationTolerance, "cap_simplification_mm", "端面与草图", "端面简化容差 mm，0 为自动", Real, 0, 10000),
        SETTING(capSpacingNeighbors, "cap_spacing_neighbors", "端面与草图", "端面点间距的近邻数", Integer, 1, 128),
        SETTING(capSpacingSamples, "cap_spacing_samples", "端面与草图", "估计端面间距的目标采样数", Integer, 1, 100000),
        SETTING(maximumSketchEdges, "maximum_sketch_edges", "端面与草图", "每个轮廓的折线边数上限", Integer, 3, 10000),
        SETTING(minimumCircleVertices, "minimum_circle_vertices", "端面与草图", "圆拟合最低顶点数", Integer, 3, 10000),
        SETTING(minimumCircleRadiusMultiplier, "minimum_circle_radius_multiplier", "端面与草图", "圆半径最低值与容差之比", Real, 0.01, 100),
        SETTING(maximumCircleGapDegrees, "maximum_circle_gap_deg", "端面与草图", "圆拟合最大采样角间隙", Real, 1, 180),
        SETTING(sideSnapNormalCosine, "side_snap_normal_cosine", "端面与草图", "侧平面贴合的最大轴向余弦", Real, 0, 1),
        SETTING(sideSnapReachMultiplier, "side_snap_reach_multiplier", "端面与草图", "侧平面贴合的搜索距离倍率", Real, 0, 100),
        SETTING(minimumRingAreaMultiplier, "minimum_ring_area_multiplier", "端面与草图", "最小轮廓面积与简化容差平方之比", Real, 0, 100),
        SETTING(intersectionAlphaRadius, "intersection_alpha_mm", "两轴求交", "整件投影 Alpha 半径 mm，0 为自动", Real, 0, 10000),
        SETTING(intersectionAlphaScale, "intersection_alpha_scale", "两轴求交", "自动 Alpha 半径倍率", Real, 0.01, 100),
        SETTING(intersectionSimplificationTolerance, "intersection_simplification_mm", "两轴求交", "整件投影简化容差 mm，0 为自动", Real, 0, 10000),
        SETTING(intersectionRegionSupport.minimumPoints, "intersection_minimum_region_points", "两轴求交", "外环最低支持点数", Integer, 0, 2000000),
        SETTING(intersectionRegionSupport.minimumFraction, "intersection_minimum_region_fraction", "两轴求交", "外环最低支持点占比", Real, 0, 1),
        SETTING(intersectionMaximumAxisCosine, "intersection_maximum_axis_cosine", "两轴求交", "两轴夹角余弦绝对值上限", Real, 0, 1),
        SETTING(projectionDepthPaddingMultiplier, "projection_depth_padding_multiplier", "两轴求交", "投影输入深度范围的容差扩展倍率", Real, 0, 100),
        SETTING(innerWallNormalCosine, "inner_wall_normal_cosine", "孔与分段", "孔壁允许的最大轴向余弦", Real, 0, 1),
        SETTING(innerWallEndTrimFraction, "inner_wall_end_trim_fraction", "孔与分段", "孔壁证据忽略的每端深度比例", Real, 0, 0.49),
        SETTING(innerWallDepthBands, "inner_wall_depth_bands", "孔与分段", "孔壁深度分带数", Integer, 1, 64),
        SETTING(innerWallMinimumBands, "inner_wall_minimum_bands", "孔与分段", "孔壁最低支持带数", Integer, 1, 64),
        SETTING(innerWallMinimumPointsPerBand, "inner_wall_minimum_points_per_band", "孔与分段", "每个孔壁带的最低点数", Integer, 1, 10000),
        SETTING(innerWallReachMultiplier, "inner_wall_reach_multiplier", "孔与分段", "孔壁距轮廓的搜索容差倍率", Real, 0, 100),
        SETTING(capWallSpacingMultiplier, "cap_wall_spacing_multiplier", "孔与分段", "端面孔壁搜索距离的点间距倍率", Real, 0, 100),
        SETTING(cappedCylinderCoverage, "capped_cylinder_coverage", "孔与分段", "端面圆柱证据最低角覆盖率", Real, 0, 1),
        SETTING(cutCylinderCoverage, "cut_cylinder_coverage", "孔与分段", "圆柱切除证据最低角覆盖率", Real, 0, 1),
        SETTING(cylinderEndBandMultiplier, "cylinder_end_band_multiplier", "孔与分段", "圆柱端点匹配带的容差倍率", Real, 0.01, 100),
        SETTING(cylinderRadialNormalCosine, "cylinder_radial_normal_cosine", "孔与分段", "圆柱壁径向法向余弦阈值", Real, 0, 1),
        SETTING(cylinderOpposingNormalFraction, "cylinder_opposing_normal_fraction", "孔与分段", "圆柱反向法向的允许比例", Real, 0, 1),
        SETTING(exteriorCircleOutsideFraction, "exterior_circle_outside_fraction", "孔与分段", "圆形端面允许的外部点比例", Real, 0, 1),
        SETTING(minimumOpeningPoints, "minimum_opening_points", "孔与分段", "开口或盲孔底的最低支持点数", Integer, 1, 100000),
        SETTING(blindHoleNormalCosine, "blind_hole_normal_cosine", "孔与分段", "盲孔端部法向余弦阈值", Real, 0, 1),
        SETTING(throughHoleOpeningNormalCosine, "through_hole_opening_normal_cosine", "孔与分段", "通孔开口法向余弦阈值", Real, 0, 1),
        SETTING(throughHoleToleranceSpanFraction, "through_hole_tolerance_span_fraction", "孔与分段", "通孔局部容差占孔壁深度的上限", Real, 0.001, 0.5),
        SETTING(throughHoleOpeningBandMultiplier, "through_hole_opening_band_multiplier", "孔与分段", "通孔开口带宽的局部容差倍率", Real, 0.01, 100),
        SETTING(throughHoleOpeningBandFraction, "through_hole_opening_band_fraction", "孔与分段", "通孔开口带占孔壁深度的上限", Real, 0.001, 0.5),
        SETTING(throughHoleCutMarginToleranceFraction, "through_hole_cut_margin_tolerance_fraction", "孔与分段", "通孔切除余量占距离容差的上限", Real, 0, 1),
        SETTING(throughHoleCutMarginDepthFraction, "through_hole_cut_margin_depth_fraction", "孔与分段", "通孔切除余量占孔深的上限", Real, 0, 0.5),
        SETTING(slotWallNormalCosine, "slot_wall_normal_cosine", "孔与分段", "槽壁法向余弦阈值", Real, 0, 1),
        SETTING(slotWallMinimumDepthFraction, "slot_wall_minimum_depth_fraction", "孔与分段", "槽壁最低深度覆盖比例", Real, 0, 1),
        SETTING(slotWallMinimumPoints, "slot_wall_minimum_points", "孔与分段", "槽壁最低支持点数", Integer, 1, 100000),
        SETTING(slotWallDistanceMultiplier, "slot_wall_distance_multiplier", "孔与分段", "槽壁距草图边的搜索容差倍率", Real, 0, 100),
        SETTING(minimumSlotSupportedEdges, "minimum_slot_supported_edges", "孔与分段", "槽的最低受支持边数", Integer, 1, 4),
        SETTING(maximumInteriorSections, "maximum_interior_sections", "孔与分段", "中间候选截面上限", Integer, 0, 100),
        SETTING(interiorSectionEndMultiplier, "interior_section_end_multiplier", "孔与分段", "中间截面离端面最低距离的容差倍率", Real, 0, 100),
        SETTING(interiorSectionMergeMultiplier, "interior_section_merge_multiplier", "孔与分段", "中间截面去重距离的容差倍率", Real, 0, 100),
        SETTING(cylinderOpeningClearanceMultiplier, "cylinder_opening_clearance_multiplier", "孔与分段", "圆柱开口验证的容差余量倍率", Real, 0, 100),
        SETTING(cylinderMergeToleranceMultiplier, "cylinder_merge_tolerance_multiplier", "孔与分段", "重合圆孔的容差倍率", Real, 0, 100),
        SETTING(cylinderMergeSpacingMultiplier, "cylinder_merge_spacing_multiplier", "孔与分段", "重合圆孔的点间距倍率", Real, 0, 100),
        SETTING(throughHoleMinimumRadiusMultiplier, "through_hole_minimum_radius_multiplier", "孔与分段", "通孔最低半径与局部容差之比", Real, 0, 100),
        SETTING(throughHoleMaximumResidualMultiplier, "through_hole_maximum_residual_multiplier", "孔与分段", "通孔证据残差与局部容差比值上限", Real, 0, 100),
        SETTING(holeOpeningOuterRadiusMultiplier, "hole_opening_outer_radius_multiplier", "孔与分段", "开口外围取证的半径倍率", Real, 1, 100),
        SETTING(cutOvertravelMultiplier, "cut_overtravel_multiplier", "孔与分段", "槽和盲孔切除的外侧余量倍率", Real, 0, 100),
        SETTING(maximumOperations, "maximum_operations", "评分与搜索", "候选程序操作数上限", Integer, 1, 16),
        SETTING(searchBeamWidth, "search_beam_width", "评分与搜索", "每轮保留的候选数", Integer, 1, 128),
        SETTING(minimumScoreImprovement, "minimum_score_improvement", "评分与搜索", "添加操作所需的最低分数改善", Real, 0, 100),
        SETTING(minimumInlierFraction, "minimum_inlier_fraction", "评分与搜索", "通过所需的最低内点率", Real, 0, 1),
        SETTING(maximumP95Multiplier, "maximum_p95_multiplier", "评分与搜索", "通过所需的 P95 距离容差倍率上限", Real, 0, 100),
        SETTING(maximumUnsupportedFaceFraction, "maximum_unsupported_face_fraction", "评分与搜索", "通过允许的最大无支持面面积比例", Real, 0, 1),
        SETTING(scoreMaximumDistanceLoss, "score_maximum_distance_loss", "评分与搜索", "评分中每点的最大距离损失", Real, 0, 100),
        SETTING(scoreNormalWeight, "score_normal_weight", "评分与搜索", "法向误差权重", Real, 0, 100),
        SETTING(scoreFaceSupportNormalCosine, "score_face_support_normal_cosine", "评分与搜索", "评分中面支持所需的法向余弦", Real, 0, 1),
        SETTING(scoreMinimumFacePoints, "score_minimum_face_points", "评分与搜索", "评分中面的最低支持点数", Integer, 1, 100000),
        SETTING(scoreOperationPenalty, "score_operation_penalty", "评分与搜索", "每个操作的评分惩罚", Real, 0, 100),
        SETTING(scoreCurvePenalty, "score_curve_penalty", "评分与搜索", "每条曲线的评分惩罚", Real, 0, 100),
        SETTING(minimumBooleanVolumeMultiplier, "minimum_boolean_volume_multiplier", "评分与搜索", "布尔体积变化门槛与容差立方之比", Real, 0, 100),
    };
#undef SETTING
    return values;
}

}  // namespace

const std::vector<ProjectionSetting>& projectionSettings() { return projectionSchema(); }

std::filesystem::path projectionConfigPath()
{
    return settings::configPath("projection", "projection.conf");
}

void setProjectionOption(ProjectionExtrusionOptions& options, std::string_view assignment)
{
    settings::setOption(projectionSchema(), options, assignment, "projection");
}

void validateProjectionOptions(const ProjectionExtrusionOptions& options)
{
    settings::validateOptions(projectionSchema(), options, "projection", validateProjection);
}

ProjectionExtrusionOptions loadProjectionOptions(const std::filesystem::path& path)
{
    return settings::loadOptions(projectionSchema(), "projection", "projection.conf", path, validateProjection);
}

std::string serializeProjectionOptions(const ProjectionExtrusionOptions& options)
{
    validateProjectionOptions(options);
    return settings::serializeOptions(projectionSchema(), options, "Projection");
}

settings::DocumentPtr makeProjectionSettings()
{
    return settings::makeDocument(projectionSchema(), "projection", "投影拟合", "projection.conf", validateProjection);
}

}  // namespace reconstruction
