# 点云到参数化 CAD：几何重建实施流程

## 1. 目标与边界

本流程把当前“完整六面长方体”的验证链扩展为面向机械零件的通用几何重建链。核心状态仍然是项目自己的 `PointStore`、`PrimitiveEvidence`、关系图和参数化 Feature Graph；CGAL、数值优化器与 OpenCASCADE 都是可替换后端，不向模块边界暴露第三方类型。

第一阶段不引入 CGAL。先完成尺度、点属性和预处理契约，避免后续 primitive detector 建立在单位不明、法向缺失或点密度失控的数据上。

## 2. 总体流水线

```text
PointCloud + Unit + Provenance
    ↓
Scale Validation / Unit Conversion
    ↓
Voxel Downsampling
Statistical Outlier Removal
    ↓
Normal Estimation + Curvature
    ↓
Object Connected Components
    ↓
Primitive Proposal
 ├─ CGAL Efficient RANSAC
 └─ Region Growing（补充、边界细化或失败回退）
    ↓
Primitive-specific Robust Refinement
    ↓
Support Reassignment / Merge / Split / Deduplication
    ↓
Primitive Evidence
    ↓
Relation Candidate Detection
    ↓
Consistent Constraint Selection
    ↓
Global Constrained Optimization
    ↓
Evidence Graph
    ↓
Feature Recognition
    ↓
Parametric Feature Graph / ModelPatch
    ↓
OpenCASCADE B-Rep / STEP
```

Region Growing 不作为 Efficient RANSAC 的固定前置步骤。全局 RANSAC 可以看到完整 primitive，Region Growing 主要用于小曲面补充、边界细化和确定性回退，避免先验过度分割把一个圆柱或球面切碎。

## 3. 数据契约

### 3.1 PointStore

每个点至少保存：

- 稳定 `PointId`；
- 三维位置；
- 可选法向量；
- 可选曲率；
- 点置信度；
- 长度单位。

长度单位至少区分 `Millimeter`、`Meter` 和 `Arbitrary`。进入 CAD 前，已知单位统一转换为毫米；未知尺度不得静默宣称为毫米。体素降采样选择原始点作为代表点，继续使用原始 `PointId`，以保留可追踪来源。

### 3.2 Primitive Evidence

后续统一为以下变体：

```cpp
using PrimitiveEvidence = std::variant<
    PlaneEvidence,
    CylinderEvidence,
    SphereEvidence,
    ConeEvidence,
    TorusEvidence>;
```

每个 evidence 保存参数、支持点 ID、残差、覆盖率、置信度、有限边界、来源算法和参数版本。检测结果不能只保存最终 OCCT Shape。

### 3.3 Relation Evidence

第一批关系为：

```text
Parallel / Perpendicular / Coplanar
Coaxial / Concentric / EqualRadius
```

关系先作为带误差和置信度的候选存在；只有选出互相一致的约束子集后才能进入全局优化。不能把所有“近似平行”直接强制为精确平行。

## 4. 分阶段实施

### Phase 1：尺度感知预处理（本轮）

交付项：

1. PLY 保留 `nx/ny/nz` 法向量；
2. 重建清单中的 `mm` / `m` / `arbitrary` 进入 `PointStore`；
3. 米制输入在几何处理前转换为毫米；
4. 体素降采样；
5. 基于 K 近邻平均距离的统计离群点过滤；
6. 对缺少法向的点执行 PCA 法向估计；
7. 由局部协方差特征值计算曲率；
8. 提供预处理报告和确定性单元测试；
9. 客户端的点云转 CAD 入口使用预处理后的点集。

验收条件：

- 已有长方体端到端测试继续通过；
- 米输入产生的 CAD 尺寸正确转换为毫米；
- 降采样点仍使用原始非零 ID；
- 明显孤立点被移除；
- 平面样本的估计法向与真实法向一致（忽略正负号），曲率接近零；
- Release 模式下核心、客户端均能构建。

### Phase 2：Primitive Proposal

- 集成 Plane、Cylinder、Sphere、Cone 与 Torus；
- 直接使用 CGAL Efficient RANSAC，不保留自研 RANSAC 运行时回退；
- 用真实数据持续校准五类图元的阈值和误检率；
- CGAL 仅存在于 adapter 实现中；
- 正式分发前确认 Shape Detection 的 GPL/商业许可策略。

当前状态：CGAL 6.2.1 Efficient RANSAC adapter 已启用并成为唯一 proposal
后端。Plane、Cylinder、Sphere、Cone 与 Torus 在同一次检测中竞争支持点，
结果全部映射回项目自有的 typed evidence；支持稳定 `PointId`、残差、覆盖率、
置信度及相应的有限范围。平面执行全支持点 PCA 精修及重复层抑制，圆柱和球面
半径使用全支持点精修。五类解析合成测试、完整客户端构建和真实点云探针均已通过。

### Phase 3：Evidence 与局部精修

- primitive 专用最小二乘或鲁棒 IRLS；
- 支持点重新分配；
- patch 合并、拆分和重复抑制；
- 恢复有限范围和边界，而不是只保留无限解析曲面。

### Phase 4：关系与全局优化

- 生成带容差的关系候选；
- 检测互斥、循环和过约束；
- 选择一致约束子集；
- 联合优化 primitive 参数与数据残差；
- 保存优化前后参数以及每条约束的来源。

### Phase 5：有限特征识别

按价值依次实现：

1. 六平面长方体；
2. 通孔；
3. 盲孔；
4. 圆柱凸台；
5. 阶梯轴；
6. 矩形口袋。

识别器输出 `ModelPatch`，不得直接修改 OCCT Shape。

## 5. 参数策略

所有距离参数以物理单位或点云局部尺度表达，禁止散落固定魔数。未知单位时允许使用包围盒对角线、平均点间距或中位近邻距离生成相对阈值，但最终进入 CAD 前必须由用户或采集元数据提供毫米换算关系。

每次运行记录：

- 输入点数、输出点数；
- 单位和毫米换算比例；
- 体素尺寸；
- 邻域大小；
- 离群阈值；
- 有法向点比例；
- 曲率分布；
- primitive 覆盖率和未解释点比例。

## 6. 评测策略

测试集分三层：

1. 解析合成数据：精确验证尺寸、方向、半径和关系；
2. 带噪合成数据：验证噪声、离群点、缺面和非均匀采样；
3. 真实扫描数据：固定输入与人工标注尺寸，作为非回归基准。

核心指标为参数误差、点覆盖率、错误 primitive 数、关系准确率、Feature 识别准确率和运行时间。只验证“能生成 STEP”不足以证明重建正确。
