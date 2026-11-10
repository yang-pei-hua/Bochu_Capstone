# 后台 CAD 重建模块算法边界

本文从算法角度逐个说明后台 CAD 重建链路中每个模块“能做什么、不能做什么、在什么条件下会拒绝”。

与[后台 CAD 重建流程](Background_CAD_Reconstruction_Flow.md)不同，本文不描述调用顺序，而是记录每个模块的算法假设、输入条件、拒绝条件和参数边界。

## 0. 范围与判定口径

- 覆盖模块 01–31 和 34，编号沿用[点云到 CAD 当前实现](Point_Cloud_to_CAD_Current_Implementation.md)。
- 模块 32（PCAD 保存/重开）和 33（STEP 导出/回读）不属于后台重建计算，本文不展开。
- 图像拍摄、COLMAP 点云生成、桌面交互和文本参数编辑不在本文范围。
- “支持”指当前代码在给定输入和参数下可以产出有效结果，不代表对所有同类零件都成立。
- “不支持”指代码中有明确拒绝、前置校验或算法假设；不是产品路线图。
- 文中数值来自 `apps/ParamCAD Studio/config/*.conf` 的当前默认值，运行时可以被超参数配置覆盖。

## 1. 总览

| 模块 | 核心算法 | 主要支持 | 主要边界 |
| --- | --- | --- | --- |
| 01 快照/单位 | 行号映射、单位换算、无效点跳过 | PLY/内存点云、mm/m、稳定 PointId | 不做尺度推断；未知单位需用户确认；不做点置信度 |
| 02 体素降采样 | 均匀体素网格、中心最近代表点 | 大规模点云、保护点 | 非特征保持；体素过大会吞掉小特征 |
| 03 离群过滤 | kNN 平均距离 + 全局统计 + MAD 稳健上限 | 稀疏离群点 | 不去除密集离群块；可能误删合法稀疏边缘 |
| 04 法向/定向/曲率 | PCA + 多源最短路径方向传播 | 有定向种子、局部光滑曲面 | 无种子连通分量无法定向；尖锐边/薄壁歧义 |
| 05 单次基元检测 | CGAL Efficient RANSAC | 平面/圆柱/球/圆锥/圆环 | 依赖有效法向；随机采样；不保证完整性 |
| 06 多轮检测 | 多尺度残差 + 法向连通分组 + 族竞争 | 小表面、相互竞争的五类基元 | 非全局最优；极小/遮挡表面可能漏检 |
| 07 证据索引 | 指纹校验 + 网格连通区域生长 | 将基元支持点转成有限面片 | 连通半径启发式；曲面面片不是精确裁剪面 |
| 08 关系推断 | 包围盒预筛 + 点邻接 + 法向/距离判据 | 平行、垂直、共面、同轴、相切等 | 仅成对面片关系；人工关系不能凭空创建 |
| 09 关系优化 | 带阻尼加权最小二乘 + 接受门槛 | 局部参数精修 | 非全局优化；误差上升或内点不足会被拒绝 |
| 10 局部恢复 | 失败区域取点 + 光晕 + 有界重检测 | 局部缺面/漏检 | 必须能取到原始观测；不放松距离/完整性阈值 |
| 11 修复合并 | 局部证据合并 + 重新索引 + 覆盖率复验 | 单次可修复失败 | 需要二次修复、覆盖下降或映射缺失时拒绝 |
| 12 箱体识别 | 六平面配对 + 三组正交平行面 | 完整六面长方体、带朝向长方体 | 不支持缺面、圆角、倒角、非正交箱体 |
| 13 贯穿孔识别 | 圆柱轴与箱体轴对齐 + 角/轴向覆盖 | 单箱体上的轴向圆通孔 | 不支持盲孔、沉孔、螺纹、斜孔、异形孔 |
| 14 曲面实体 | 解析曲面精修 + 端面/角度覆盖评估 | 完整球、带端面圆柱、圆锥/圆台、完整圆环 | 要求外向法向和完整端面；部分扫描拒绝 |
| 15 组合体 | 平面框架 + 材料单元 + 箱体覆盖 + 边界验证 | 正交框架下的长方体/棱柱组合、少量斜面 | 网格/边界/斜面数量有限；非通用机械零件 |
| 16 混合特征 | 薄矩形宿主 + 通孔 + 附着体 + 矩形切除 | 薄板加凸台/曲面附件/矩形槽 | 必须有薄矩形宿主；不支持任意口袋/自由曲面 |
| 17 模板引导 | 模板拟合 + 剩余材料识别 + 边界复验 | 用户指定长方体/球/圆柱/圆锥/圆环/平面 | 仅六类模板；选区覆盖不足或与最终边界冲突会失败 |
| 18 装配面片 | 坐标哈希采样 + 区域生长 | 多体点云的面片分解 | 非特征保持采样；薄壁/尖锐边仍可能泄漏 |
| 19 分组/回填 | 邻接图 + 凸性 + 并查集 + 最近邻回填 | 面片合并为部件候选 | 凹接触/粘连可能合并或过分割；不保证真实零件实例 |
| 20 边界恢复 | CGAL Delaunay + Alpha Shape + 折线简化/吸附 | 平面面片的有限轮廓、孔、分离岛 | 仅平面面片；稀疏采样和拓扑回退会降低精度 |
| 21 壳闭合 | 半空间求交/凸多面体 + 边图检查 + OCCT 试建 | 凸平面壳、闭合曲面体 | 要求闭合、流形、方向已知、无内部矛盾观测 |
| 21S 专用恢复 | 解析轮廓/环形管/圆盘/局部轮廓 | 特定几何族和局部窗口 | 需要高覆盖率；通用性有限 |
| 22 候选轴 | 圆柱轴/平面法向/PCA/侧壁叉积/法向矩 | 单轴拉伸、两轴求交、分段 | 最多 12 轴；不保证真实轴一定在候选中 |
| 23 区间/端面 | 深度分位数 + 端面法向余弦 | 有明确端面的拉伸体 | 缺端面会跳过该轴；不处理任意斜端面 |
| 24 二维截面 | 轮廓拟合 + 圆拟合 + 线段化 | 一个外环 + 有壁证据的孔 | 仅圆和直线；多外环、复杂曲线失败 |
| 25 操作提案 | 基体/台阶/槽/盲孔/通孔 | 沿轴的拉伸、矩形槽、圆孔 | 仅挤出类操作；任意口袋/非圆孔不支持 |
| 26 评分 | 点到面距离 + 法向 + 无支持面面积 | 已试建候选的质量评估 | 采样评分；依赖覆盖；不是物理精度指标 |
| 27 序列搜索 | Beam Search + 操作扩展/剪枝 | 最多 3 步操作序列 | 非穷举；无有效基体或支持不足会失败 |
| 28 建模计划 | 候选转 ModelPatch + 修订绑定 | 长方体/棱柱/孔/曲面/壳体/投影程序 | 手动修改保护；修订不匹配拒绝 |
| 29 草图校验 | OCCT 构面 + 闭合/退化诊断 | 点/线/矩形/圆草图 | 开放/自交/退化轮廓不能消费 |
| 30 B-Rep 重建 | 特征历史重放 + OCCT 布尔 | 草图、拉伸、切除、五类基元、壳体 | 上游缺失即失败；无通用拓扑命名 |
| 31 原子提交 | expectedRevision + 全量回滚 | 命令序列和重建的原子性 | 不支持部分提交；重建失败整体回滚 |
| 34 生产编排 | 后台线程 + 阶段信号 + 修订核对 | 完整生产任务和结果交接 | 阶段边界取消；不重试；结果过期即丢弃 |

## 2. 输入与预处理

### 模块 01｜点云快照、单位与 PointId

**输入/输出**

- 输入：PLY 点云或内存 `PlyCloud`、活动点集合、源行映射、单位声明。
- 输出：`ObservationSnapshot`、源单位与尺度、稳定 `PointId`、点云指纹。

**核心算法**

- 客户端 `PointId = 原文档行号 + 1`。
- 跳过非有限坐标，但不给剩余点重新编号。
- 米制输入统一换算到毫米；`mm`/`m` 之外的未知单位标记为 `Arbitrary`。
- 有效输入法向直接保留，不做符号规范化。

**支持**

- 有明确 `mm` 或 `m` 单位的点云。
- 未知尺度点云，但必须由用户在客户端明确确认 `1 源单位 = 1 mm`。
- 带颜色、可选法向的 PLY。
- 删除点、跳过无效点后仍保持原始点 ID 可追溯。

**不支持 / 拒绝**

- 不根据几何尺寸自动推断真实尺度。
- 不把 `Arbitrary` 静默当作毫米。
- 不提供逐点置信度、协方差或测量不确定度。
- 不读取相机内外参、拍摄位姿或图像对应关系。
- 不接收三角网格、B-Rep 或带拓扑的点云作为几何输入。

**关键假设**

- 同一份快照内 `PointId` 非零且唯一。
- 坐标单位在进入几何算法前已经统一。

**源码**

- [PointCloudAdapter.cpp](../apps/ParamCAD%20Studio/src/io/PointCloudAdapter.cpp#L95-L137)
- [PointStore.h](../core/geometric_reconstruction/include/reconstruction/data/PointStore.h#L60)

### 模块 02｜体素降采样

**核心算法**

- 均匀体素网格，每个体素保留离体素中心最近的原始点。
- 距离相同则保留 `PointId` 更小的点。
- 保护点不参与体素竞争，始终保留。

**支持**

- 控制大规模点云的计算量。
- 保留原始点 ID 和源顺序。
- 模板点、人工选区点通过保护列表完整保留。

**不支持 / 拒绝**

- 不是特征保持简化；体素大于薄壁、孔壁或小凸台时会把特征吞掉。
- 不做点平均或插值，只选择已有代表点。
- 不保证降采样后密度均匀。
- 不处理网格简化、边折叠或曲率自适应采样。

**关键阈值**

- `voxel_size_mm = 0` 时使用包围盒对角线比例 `voxel_diagonal_fraction = 0.002`。

**源码**

- [Downsampling.cpp](../core/geometric_reconstruction/src/preprocessing/Downsampling.cpp#L30)
- [PointCloudPreprocessor.cpp](../core/geometric_reconstruction/src/preprocessing/PointCloudPreprocessor.cpp#L48)

### 模块 03｜统计离群过滤

**核心算法**

- 对每个点计算 K 近邻平均距离。
- 全局阈值：均值 + `outlier_standard_deviation_multiplier` × 标准差。
- 稳健上限：在全局阈值上再取 `min(全局阈值, max(3 × 中位数, 中位数 + 2.5 × 1.4826 × MAD))`。
- 超过阈值且不在保护列表中的点被删除。

**支持**

- 稀疏的飞点、远处噪点。
- 少量离群点不会抬高阈值到失效。
- 模板点和人工选区点可豁免。

**不支持 / 拒绝**

- 不能去除密集的离群块；这些点互相有邻居，平均距离不高。
- 可能删除合法的稀疏边缘或薄结构。
- 不做聚类分割、半径滤波或基于颜色的过滤。
- 少于 4 个点或邻域数为 0 时直接跳过。

**关键阈值**

- `outlier_neighbor_count = 12`
- `outlier_standard_deviation_multiplier = 2.5`
- `outlier_robust_median_multiplier = 3`

**源码**

- [OutlierFilter.cpp](../core/geometric_reconstruction/src/preprocessing/OutlierFilter.cpp#L4)

### 模块 04｜法向、定向与曲率

**核心算法**

- 输入法向先归一化；非法法向被清除。
- 缺失法向用 K 近邻 PCA 的最小特征向量估计。
- 曲率使用 `λ0 / (λ0 + λ1 + λ2)`。
- 方向传播使用多源最短路径：以已定向点为根，按法向对齐、切向偏差、间距比例和距离代价扩展；必要时翻转法向。

**支持**

- 局部光滑、邻域充足的曲面。
- 带定向法向的输入可以直接作为材料方向证据。
- 通过一个连通分量内有定向种子，把方向传播到其余点。

**不支持 / 拒绝**

- 没有定向种子的连通分量保持 `normalOrientationKnown = false`，常规实体识别会拒绝。
- 共线或重合邻域没有稳定法向，直接跳过。
- 尖锐边、薄壁两侧、遮挡边界处 PCA 法向会模糊，方向传播也可能选错。
- 不做全局方向优化、学习式法向或曲面类型先验。

**关键阈值**

- `normal_neighbor_count = 18`
- `normal_orientation_minimum_alignment = 0.75`
- `normal_orientation_maximum_tangent_deviation = 0.55`
- `normal_orientation_maximum_spacing_ratio = 4`
- `normal_orientation_distance_cost_weight = 0.001`

**源码**

- [NormalEstimation.cpp](../core/geometric_reconstruction/src/preprocessing/NormalEstimation.cpp#L4-L143)

## 3. 曲面分析与修复

### 模块 05｜单次基元检测

**核心算法**

- CGAL Efficient RANSAC，竞争性检测平面、圆柱、球、圆锥、圆环。
- 只有带有效法向的点进入 CGAL。
- 每类基元有独立的最小支持点、距离阈值、法向对齐、概率、聚类阈值和随机种子。

**支持**

- 五类解析曲面：平面、圆柱、球、圆锥、圆环。
- 尺度自适应阈值：距离阈值为 0 时由包围盒对角线推导。
- 同一轮内多类基元竞争。

**不支持 / 拒绝**

- 缺少有效法向、点数不足、所有点位置相同都会失败。
- RANSAC 是随机算法，结果受随机种子、概率和采样影响，不保证全局最优或完整。
- 基元只是 proposals，不保证对应真实 CAD 面；可能产生部分面、重复面或伪面。
- 不支持 NURBS、自由曲面、圆角/倒角过渡面。
- 每类基元有最大数量限制，超出部分可能被丢弃。

**关键阈值**

- 平面最多 6、圆柱最多 8、球/圆锥/圆环最多 32。
- 概率默认 0.01，法向对齐默认 0.9。

**源码**

- [CgalEfficientRansacDetector.cpp](../core/geometric_reconstruction/src/detection/cgal/CgalEfficientRansacDetector.cpp#L6-L151)
- [detection.conf](../apps/ParamCAD%20Studio/config/detection.conf)

### 模块 06｜多轮检测与证据汇集

**核心算法**

- 先做一轮全局检测，再用未分配点构造残差区域。
- 按法向一致性做并查集分组，形成局部区域。
- 在局部区域中逐族重新检测；球和圆环只允许在局部恢复中检测，避免在近似平面区域上生成巨大球/圆环。
- 对同一连通区域的多个族做竞争，按残差和法向误差选择首选族；同族备选保留。

**支持**

- 小表面被大表面“抢走”时的局部补偿。
- 平面、圆柱、球、圆锥、圆环之间的族竞争。
- 重新计算支持点、覆盖率、置信度和未分配点。

**不支持 / 拒绝**

- 不是全局最优分割；族竞争是启发式。
- 轮数有限，极小表面或严重遮挡表面仍可能漏检。
- 局部区域低于最小支持点会被丢弃。
- 同一区域只选一个首选族，复杂过渡面可能被错误归类。

**关键阈值**

- `multi_scale_rounds = 3`
- 初始对齐 0.94，恢复对齐 0.85
- 最小支持下限 3
- 单族覆盖判定阈值 0.98

**源码**

- [MultiScaleDetector.cpp](../core/geometric_reconstruction/src/detection/MultiScaleDetector.cpp#L10-L140)
- [SurfaceAnalysis.cpp](../core/geometric_reconstruction/src/analysis/SurfaceAnalysis.cpp#L7)

### 模块 07｜证据索引与有限面片

**核心算法**

- 校验点云指纹，拒绝跨快照的证据。
- 对每个基元，把支持点按空间网格连通性生长为有限面片。
- 计算面片质心、包围盒、材料法向和方向可信状态。
- 未归属点保留为 `unassignedPointIds`。

**支持**

- 把一个基元的支持点拆成一个或多个有限连通面片。
- 支持曲面和平面面片。
- 为关系推断提供点级支撑。

**不支持 / 拒绝**

- 支持点 ID 来自其它快照时直接失败。
- 连通半径是启发式：由关系距离或面积/点数推导，可能在稀疏处断裂或在密集处粘连。
- 曲面面片只是支持点集合，不是精确裁剪的解析曲面边界。
- 一个基元可能产生多个面片，也可能与其它基元共享点。
- 方向未知的面片会被标记，影响后续实体闭合。

**关键阈值**

- 连通距离为 0 时使用 `max(4 × 关系距离容差, 2.5 × sqrt(面积 / 点数))`。

**源码**

- [SurfacePatches.cpp](../core/geometric_reconstruction/src/analysis/SurfacePatches.cpp#L50-L134)

### 模块 08｜关系推断与人工关系校验

**核心算法**

- 用包围盒预筛邻接，再用点邻接和距离阈值确认。
- 推断关系：邻接、平行、垂直、共面、同轴、同半径、相交、相切、轴垂直平面、轴平行平面。
- 人工关系通过点重叠率绑定到当前面片，并要求该关系已经存在于自动推断结果中。

**支持**

- 平面-平面、平面-曲面、曲面-曲面的成对关系。
- 为关系优化提供约束。
- 人工关系可以确认或固定已有几何关系。

**不支持 / 拒绝**

- 只做成对面片关系，不做多体约束、运动副或装配配合。
- 人工关系不能凭空创建与观测矛盾的几何关系；冲突时返回错误。
- 关系是标注/约束，不直接驱动模型变形。
- 不推断特征级关系、基准关系或拓扑父子关系。
- 支持点来自其它快照时直接失败。

**关键阈值**

- 关系角度容差默认 5°。
- 邻接置信度 0.75，最大置信度 0.95。
- 人工关系最小点重叠比例 0.6。

**源码**

- [SurfaceRelations.cpp](../core/geometric_reconstruction/src/analysis/SurfaceRelations.cpp#L32-L151)
- [analysis.conf](../apps/ParamCAD%20Studio/config/analysis.conf)

### 模块 09｜关系优化

**核心算法**

- 在关系约束下对基元参数做加权最小二乘/阻尼优化。
- 目标包含残差和关系约束权重。
- 对优化结果做内点率、平均误差上升、轴对齐、半径/角度容差校验。

**支持**

- 局部修正基元参数，使平行、垂直、共面、同轴等关系更一致。
- 返回接受/拒绝关系及优化前后误差。

**不支持 / 拒绝**

- 只优化支持的关系类型和基元组合；其它组合拒绝。
- 不是全局优化，可能陷入局部极小。
- 内点率低于 0.9 或平均误差上升超过 15% 时拒绝。
- 不增加缺失面、不改变拓扑、不修复错误的基元类型。
- 支持点或面片引用失效时拒绝。

**关键阈值**

- 最大尝试 32 次，最多 12 个活动曲面，残差采样 256，迭代 15 次。
- 内点率 0.9，平均误差上升上限 0.15，验证容差比例 0.25。

**源码**

- [RelationOptimization.cpp](../core/geometric_reconstruction/src/analysis/RelationOptimization.cpp#L32-L153)

### 模块 10｜局部观测恢复

**核心算法**

- 从失败区域点 ID 出发，校验工作快照和源快照的单位、坐标和 ID 一致性。
- 在失败区域周围加光晕，取回源观测；已经由稳定平面解释的点不加入。
- 小区域再做连通分量过滤。
- 在局部点上做两轮有界检测，重新索引证据并优化关系。

**支持**

- 局部漏检、局部缺面、少量补点。
- 把全分辨率观测补回工作点云。
- 保留已有局部基元，追加新候选。

**不支持 / 拒绝**

- 源快照不包含失败区域观测、坐标不一致或 ID 不一致时拒绝。
- 空区域、非法光晕、单位不一致时拒绝。
- 无定向法向的源点不会加入恢复。
- 不放松距离阈值或边界完整性要求；不能凭空补出完全未观测的曲面。
- 只做有界局部检测，不做全局重新分析。

**关键阈值**

- 最小恢复区域点数 20。
- 光晕容差倍数 2，关系光晕倍数 16。
- 稳定平面重叠 0.1，法向余弦 0.95。
- 连通容差倍数 2/4。

**源码**

- [LocalRecovery.cpp](../core/geometric_reconstruction/src/analysis/LocalRecovery.cpp#L14-L153)

### 模块 11｜修复合并与复验

**核心算法**

- 把局部修复证据与原始分析合并，重新建立面片索引和关系。
- 用观测点 ID 重新绑定人工关系，不使用旧面片编号。
- 重新识别并比对修复前后的覆盖率。
- 只有修复候选不降低验证指标时才提交。

**支持**

- 单次可修复的局部失败。
- 合并补点、局部面片和局部关系。
- 保留原分析和原候选作为失败回退。

**不支持 / 拒绝**

- 合并后仍需要第二次修复时拒绝。
- 局部到全局面片映射缺失时拒绝。
- 覆盖率或关系验证退化时拒绝。
- 修复源指纹变化时拒绝。
- 不做无限迭代修复，不做全局重新优化。

**源码**

- [RepairTransaction.cpp](../core/geometric_reconstruction/src/recognition/RepairTransaction.cpp#L82-L130)

## 4. 常规识别与引导

### 模块 12｜箱体识别

**核心算法**

- 低层函数要求恰好 6 个平面候选。
- 穷举平面配对，寻找 3 组平行面，并要求 3 个方向互相正交。
- 由成对平面求原点和三维尺寸；用平面置信度、点覆盖率和关系一致性计算箱体置信度。
- 分析入口会按支持点数取最强的 6 个平面。

**支持**

- 完整六面长方体。
- 带任意朝向的正交箱体。
- 轴对齐快速入口：按 XYZ 最小/最大坐标生成箱体。

**不支持 / 拒绝**

- 少于 6 个平面、找不到 3 组正交平行面时失败。
- 退化尺寸、非法角度容差时失败。
- 不支持缺面箱体、圆角/倒角箱体、非正交箱体、空心箱体。
- 轴对齐入口要求每个边界平面都有支持点。
- 不处理箱体上的孔、凸台或切除，这些由后续模块处理。

**关键阈值**

- 角度容差默认 5°。
- 轴对齐入口要求毫米单位和非零三维跨度。

**源码**

- [BoxRecognizer.cpp](../core/geometric_reconstruction/src/recognition/BoxRecognizer.cpp#L77-L235)
- [AxisAlignedBoxRecognizer.cpp](../core/geometric_reconstruction/src/recognition/AxisAlignedBoxRecognizer.cpp#L24-L120)

### 模块 13｜贯穿孔识别

**核心算法**

- 只处理圆柱证据。
- 把圆柱轴与箱体三轴比较，要求夹角不超过 5°。
- 检查角覆盖、轴向覆盖、端点间隙、孔中心是否在箱体横向边界内。
- 计算角覆盖得分和轴向跨度得分，输出带置信度的通孔候选。

**支持**

- 单箱体上的轴向圆通孔。
- 孔壁圆柱与箱体轴对齐。
- 孔贯穿两个相对外表面。

**不支持 / 拒绝**

- 不支持盲孔、沉孔、螺纹孔、锥孔、异形孔。
- 不支持斜孔或与箱体轴不平行的孔。
- 角覆盖低于 0.7、轴向覆盖低于 0.85、端点间隙超过厚度 10% 时拒绝。
- 孔中心超出箱体边界或孔与边界干涉时拒绝。
- 不支持相交孔、交叉孔和多孔组合的布尔验证。

**关键阈值**

- 角度容差 5°。
- 最小角覆盖 0.7。
- 最小轴向覆盖 0.85。
- 最大端点间隙比例 0.1。
- 最小边界 clearance 0。

**源码**

- [ThroughHoleRecognizer.cpp](../core/geometric_reconstruction/src/recognition/ThroughHoleRecognizer.cpp#L51-L180)
- [recognition.conf](../apps/ParamCAD%20Studio/config/recognition.conf#L18-L23)

### 模块 14｜独立曲面实体识别

**核心算法**

- 支持球、圆柱、圆锥/圆台、圆环。
- 对每个候选做参数精修。
- 评估表面覆盖率、边界覆盖率、端面覆盖、角度分箱覆盖、法向一致性和圆锥顶点观测。
- 选择覆盖率减残差得分最高的候选。

**支持**

- 完整球。
- 带两端端面的圆柱。
- 圆锥/圆台，需要顶点附近观测和端面。
- 完整圆环体。

**不支持 / 拒绝**

- 要求所有点都有外向法向；否则直接拒绝。
- 表面覆盖率或边界覆盖率低于 0.9 时拒绝。
- 圆柱/圆锥缺端面时拒绝。
- 球/圆环角度覆盖不足时拒绝。
- 圆锥顶点无观测时覆盖率为 0，拒绝。
- 不支持部分球、部分圆环、自由曲面、圆角过渡、多个曲面实体组合。
- 一次只返回一个最佳曲面实体。

**关键阈值**

- 距离容差 0.1 mm。
- 最小表面/边界覆盖 0.9。
- 法向对齐 0.8。
- 端面法向余弦 0.98。
- 最小端面点 12。
- 球/圆环/圆柱/圆锥使用固定角度和轴向分箱。

**源码**

- [CurvedSolidRecognizer.cpp](../core/geometric_reconstruction/src/recognition/curved/CurvedSolidRecognizer.cpp#L44-L139)
- [BoundaryAssessment.cpp](../core/geometric_reconstruction/src/recognition/curved/BoundaryAssessment.cpp#L55-L160)
- [CapDetection.cpp](../core/geometric_reconstruction/src/recognition/curved/CapDetection.cpp)

### 模块 15｜组合体识别

**核心算法**

- 从有向法向恢复平面框架。
- 把空间划分为材料单元网格，每个单元用 5×5×5 采样判断材料占据。
- 对不能完全占据/完全空的单元，尝试用斜面棱柱解释。
- 用最大矩形覆盖和分支定界选择最少长方体覆盖。
- 验证组合体边界和覆盖率。

**支持**

- 正交框架下的多个长方体。
- 多边形棱柱和少量斜面棱柱。
- 平面加强筋、L 型等由多个长方体/棱柱组成的零件。
- 有边界证据和界面平面的组合体。

**不支持 / 拒绝**

- 要求毫米单位和有向法向。
- 网格单元超过 512、搜索节点超过 200000 时失败。
- 边界数超过 10、斜面数超过 6 时失败。
- 材料单元误差超过 0.12 时失败。
- 覆盖率低于 0.9 时失败。
- 不支持自由曲面、任意机械零件、复杂凹形或多材料组合。
- 斜面棱柱仍然是有限支持，不是任意多面体。

**关键阈值**

- 距离容差 0.1 mm。
- 法向对齐 0.98。
- 最小点 30，最小支持点 20。
- 单元采样 5×5×5。
- 覆盖率 0.9。

**源码**

- [CompoundRecognizer.cpp](../core/geometric_reconstruction/src/recognition/compound/CompoundRecognizer.cpp#L5-L22)
- [PlanarFrame.cpp](../core/geometric_reconstruction/src/recognition/compound/PlanarFrame.cpp#L4-L80)
- [MaterialCells.cpp](../core/geometric_reconstruction/src/recognition/compound/MaterialCells.cpp#L4-L90)
- [BoxCover.cpp](../core/geometric_reconstruction/src/recognition/compound/BoxCover.cpp#L4-L90)
- [BoundaryValidation.cpp](../core/geometric_reconstruction/src/recognition/compound/BoundaryValidation.cpp)

### 模块 16｜混合特征识别

**核心算法**

- 先识别“薄矩形宿主”，要求一对近似平行的主平面和侧壁。
- 从圆柱证据中筛选朝内的孔壁，识别贯穿孔。
- 把宿主上方的点分成“上方”和“未知”两组。
- 对每个连通组做局部识别：减法方向尝试矩形切除；加法方向尝试附着曲面体或平面组合体。
- 最后用材料占据检查计算表面覆盖率，低于 0.98 拒绝。

**支持**

- 薄板 + 贯穿孔。
- 薄板 + 附着凸台/曲面体。
- 薄板 + 矩形切除棱柱。
- 局部失败的组可以做一次局部观测恢复。

**不支持 / 拒绝**

- 没有薄矩形宿主时返回 "No observed thin rectangular host"。
- 附着体无法识别为曲面体或平面组合体时返回 "Unsupported attached component"。
- 未知组点数达到最小局部点数但仍失败时，整轮拒绝。
- 只支持矩形切除，不支持任意形状口袋。
- 要求外向法向；不支持自由曲面、复杂凸台组合。
- 覆盖率低于 0.98 时拒绝。

**关键阈值**

- 宿主平面余弦 0.995，最小宿主点 30。
- 最小局部点 20。
- 最小表面覆盖 0.98。
- 切除壁数 4–12。
- 凸台覆盖 0.9，边界覆盖 0.8。

**源码**

- [MixedFeatureRecognizer.cpp](../core/geometric_reconstruction/src/recognition/mixed/MixedFeatureRecognizer.cpp#L8-L134)
- [HostDetection.cpp](../core/geometric_reconstruction/src/recognition/mixed/HostDetection.cpp)
- [AttachedFeatures.cpp](../core/geometric_reconstruction/src/recognition/mixed/AttachedFeatures.cpp)
- [recognition.conf](../apps/ParamCAD%20Studio/config/recognition.conf#L72-L122)

### 模块 17｜模板与人工关系引导

**核心算法**

- 对每个模板选区单独检测，强制只输出一个指定类型基元。
- 曲面模板会扩展边界支持点；已完整观测的曲面部件先固定，并从剩余材料中扣除。
- 剩余材料走常规识别。
- 用最终 CAD 边界逐点复验模板选区，要求至少 90% 的点匹配。
- 人工关系在最后重新应用，保证端点仍指向原始快照。

**支持**

- 六类模板：长方体、球、圆柱、圆锥、圆环、平面。
- 用模板约束“先固定某个部件，再识别其余材料”。
- 人工关系确认已有几何关系。
- 模板失败时返回失败选区 ID。

**不支持 / 拒绝**

- 只支持六类解析模板，不支持草图模板、自由曲面模板或用户自定义特征。
- 模板点数少于 20 时拒绝。
- 模板拟合覆盖不足、模板与最终 CAD 边界冲突时拒绝。
- 人工关系与观测矛盾时拒绝。
- 模板是强约束：有模板时不会走自动装配分离。
- 只会扣除“已完整观测”的曲面部件，部分观测模板仍可能导致剩余材料识别失败。

**关键阈值**

- 最小模板点 20。
- 最小模板匹配比例 0.9。
- 边界容差倍数 3，边界法向余弦 0.8。
- 长方体模板最小支持点 20。
- 最小种子点 20。

**源码**

- [GuidedSolidRecognizer.cpp](../core/geometric_reconstruction/src/recognition/guided/GuidedSolidRecognizer.cpp#L168-L265)
- [HintFitting.cpp](../core/geometric_reconstruction/src/recognition/guided/HintFitting.cpp)
- [HintValidation.cpp](../core/geometric_reconstruction/src/recognition/guided/HintValidation.cpp#L36-L94)
- [recognition.conf](../apps/ParamCAD%20Studio/config/recognition.conf#L124-L138)

## 5. 装配分离

### 模块 18｜装配面片提取

**核心算法**

- 全分辨率观测按坐标哈希采样到工作点预算。
- KdTree 邻域 + PCA 法向/曲率。
- 基于法向夹角、连接距离、平面偏移和薄壁穿越检测做区域生长。
- 平面残差超过容差的面片再按局部法向光滑性做曲面生长。

**支持**

- 多体点云、相互连接的面片、平面与曲面面片。
- 最多 20 万工作点。
- 保护点不在装配模块中特殊处理，但全分辨率观测会在后续回填。

**不支持 / 拒绝**

- 要求毫米单位和合法参数；非毫米、重复/非法 ID、非法阈值直接拒绝。
- 工作点超过预算时按坐标哈希截断，可能丢失小面片。
- 最小面片点数 24，低于阈值的区域被丢弃。
- 薄壁、尖锐边仍可能泄漏；`crossesObservedBoundary` 只是启发式检查。
- 没有全局分割最优性，不做深度学习或谱聚类。

**关键阈值**

- `maximum_working_points = 200000`
- `neighbors = 32`
- `minimum_patch_points = 24`
- `normal_angle_deg = 10`
- `connectivity_factor = 2.5`
- `alpha_factor = 2.5`

**源码**

- [AssemblyReconstruction.cpp](../core/geometric_reconstruction/src/recognition/assembly/AssemblyReconstruction.cpp#L89-L195)
- [assembly.conf](../apps/ParamCAD%20Studio/config/assembly.conf)

### 模块 19｜连接、分组与全量回填

**核心算法**

- 遍历相邻点对，统计面片之间的连接数和凸连接数。
- 凸性由法向和连接位移的点积判断。
- 用并查集合并连接被接受的面片。
- 全分辨率观测按最近 4 个工作点分配回面片和部件；无法匹配的点保留为未分配。

**支持**

- 把面片合并成部件候选。
- 保留每个部件的工作点和全分辨率点 ID。
- 输出连接证据、接受状态和距离。

**不支持 / 拒绝**

- 法向方向都已知时，连接数少于 3 或凸连接比例低于 0.7 不合并；方向未知时只要求连接数足够。
- 凹接触、粘连、薄壁两侧可能合错或过度分裂。
- 全分辨率回填使用最近邻和法向对齐，边界附近可能未分配或错分配。
- 不保证部件等于真实装配零件实例；代码明确说明凹部和接触仍有歧义。
- 不做装配配合、运动副或装配树。

**关键阈值**

- `minimum_link_count = 3`
- `minimum_convex_link_fraction = 0.7`
- `connection_tolerance_multiplier = 2`

**源码**

- [AssemblyReconstruction.cpp](../core/geometric_reconstruction/src/recognition/assembly/AssemblyReconstruction.cpp#L197-L245)

### 模块 20｜有限边界恢复

**核心算法**

- 平面面片支撑点投影到二维。
- CGAL Delaunay 三角化，按外接圆半径过滤得到 Alpha Shape。
- 只被一个三角形使用的边组成边界，追踪成环。
- 自交环拆分、Douglas-Peucker 风格简化。
- 把边界边吸附到相邻面片的平面交线。
- 用 CGAL `Polygon_2` 检查简单性、面积和环间接触拓扑。

**支持**

- 平面面片的有限外轮廓。
- 孔和分离岛。
- 与相邻平面求交后的共享棱吸附。
- 拓扑不一致时回退到未简化轮廓。

**不支持 / 拒绝**

- 只处理平面面片；曲面面片跳过边界恢复。
- Alpha Shape 对稀疏、非均匀采样敏感；半径过大会粘连，过小会断裂。
- 拓扑校验失败时回退，边界精度下降。
- 不恢复自由曲面边界、参数曲线边界或精确 B-Rep 裁剪边。
- 非简单环可能被拆分或拒绝。

**关键阈值**

- Alpha 半径 `max(2 × 容差, 面片间距 × 2.5)`。
- 简化容差与间距相关。
- 拓扑回退次数、约束回退次数记录在诊断中。

**源码**

- [FiniteBoundaries.cpp](../core/geometric_reconstruction/src/recognition/assembly/FiniteBoundaries.cpp#L19-L78)
- [BoundaryGeometry.cpp](../core/geometric_reconstruction/src/recognition/assembly/BoundaryGeometry.cpp#L54-L196)

### 模块 21｜共边、壳闭合与试建

**核心算法**

- 平面凸体闭合法：
  - 合并共面面片为支撑平面；
  - 检查支撑平面集合是否存在 recession ray；
  - 用三个平面求交得到顶点；
  - 顶点必须满足所有半空间；
  - 每个面上的顶点做凸包；
  - 用入射面方程做 QR 最小二乘精修顶点；
  - 长边插入其它面的顶点处理 T-junction；
  - 统计边使用次数和方向，检查开放边/非流形边；
  - 构建 `ProfileBodyParams`，交给 OCCT 生成 B-Rep；
  - 用实体分类器检查内部是否有矛盾观测点。
- 曲面体闭合法：对部件点重新做 `reconstructCurvedSolid`，要求端面和角度覆盖，再构建并检查内部矛盾。

**支持**

- 由平面组成的凸壳。
- 通过验证的闭合曲面体。
- 输出闭合/开放状态、开放边数、非流形边数、置信度和拒绝原因。

**不支持 / 拒绝**

- 面片数少于 4 或多于 32 时凸支撑面闭合不成立；面片数超过 128 时部件直接标为 ambiguous。
- 非平面面片、分离边界岛、退化边界直接拒绝。
- 开放边或非流形边不为 0 时拒绝。
- 方向未知时拒绝。
- 构建出的实体内部包含大量观测点时，判定为材料冲突并拒绝。
- 凸支撑平面法只适合凸平面壳；一般凹形平面壳可能闭不上，需要局部轮廓恢复。
- 不支持自由曲面、非流形实体、任意凹多面体。

**关键阈值**

- `minimum_part_patches = 4`
- `maximum_part_patches = 32`
- `maximum_closure_patches = 128`
- `maximum_closure_vertices = 4096`
- `weld_tolerance_multiplier = 4`
- `minimum_part_span_multiplier = 5`
- 材料冲突阈值与点数相关。

**源码**

- [PartClosure.cpp](../core/geometric_reconstruction/src/recognition/assembly/PartClosure.cpp#L15-L200)

### 模块 21S｜专用恢复路径

#### 21S.1 解析轮廓体

- **算法**：用观测墙面半平面反复裁剪初始矩形，得到外轮廓；支持孔和圆角；用扇区/层网格验证孔壁观测覆盖。
- **支持**：有明确侧壁和端面的解析轮廓体、带孔轮廓、圆角轮廓。
- **不支持**：部分扫描、开口未观测、覆盖不足的轮廓；一般自由曲线轮廓。
- **阈值**：至少 48 个观测，覆盖度达到期望网格的 90%；孔口 12×3 扇区网格，至少 48 个观测，扇区不足一半拒绝。
- **源码**：[AnalyticProfileClosure.cpp](../core/geometric_reconstruction/src/recognition/assembly/AnalyticProfileClosure.cpp#L79-L111)

#### 21S.2 环形管

- **算法**：按 16 个扇区、3 层端面、6 层侧壁做 bitset 占用统计；拟合内外圆柱和端面；验证壁面覆盖。
- **支持**：完整环形管、内外壁和两端端面可观测的管件。
- **不支持**：部分扇区缺失、端面缺失、内部有矛盾观测的管件。
- **阈值**：至少 400 点；非平面面片；壁面覆盖 92%；每个扇区必须有观测；端面需独立观测。
- **源码**：[AnnularTubeClosure.cpp](../core/geometric_reconstruction/src/recognition/assembly/AnnularTubeClosure.cpp#L27-L156)

#### 21S.3 圆盘体

- **算法**：用半平面裁剪圆，得到圆弧和直线段组成的轮廓；验证圆心、半径和壁面覆盖。
- **支持**：圆形或部分被平面切掉的圆盘/凸台。
- **不支持**：非圆轮廓、覆盖不足、圆心/半径不稳定的情况。
- **源码**：[CircularSlabClosure.cpp](../core/geometric_reconstruction/src/recognition/assembly/CircularSlabClosure.cpp#L35-L78)

#### 21S.4 局部轮廓恢复

- **算法**：寻找开放部件，按点数和面片数筛选；用对立端面平面、签名裁剪、包络或通道窗口裁出局部样本；递归运行 `reconstructAssembly`；把支持新轮廓的点/面片从 donor 部件转移出来。
- **支持**：一个大组中可被端面/通道窗口分离出的子部件；父组拆分。
- **不支持**：没有对立端面、点数不足、源快照不完整、覆盖不足或与已接受实体冲突的情况。
- **阈值**：部件至少 2 个面片、点数不少于 400；最多 48 次局部恢复；局部覆盖 0.98/0.80 的门槛。
- **源码**：[LocalProfileRecovery.cpp](../core/geometric_reconstruction/src/recognition/assembly/LocalProfileRecovery.cpp#L441-L520)

## 6. 投影拟合

### 模块 22｜候选轴生成

**核心算法**

- 汇总圆柱轴、平面法向、PCA 主轴、侧平面法向叉积、侧壁法向矩和可选的小凸投影方向。
- 去重后限制最大轴数。

**支持**

- 单轴拉伸。
- 两轴拉伸求交。
- 分段拉伸、加材和切除。
- 轴线来自已有基元证据或点云主方向。

**不支持 / 拒绝**

- 最多 12 个候选轴，不保证真实拉伸轴一定在候选中。
- 没有足够平面/圆柱证据时，只剩 PCA 轴，稳定性下降。
- `include_area_directions` 默认关闭，面积方向不参与。
- 不支持扫掠、旋转、放样等非挤出操作。
- 轴的对齐余弦阈值 0.9986，近似方向会被去重。

**关键阈值**

- `maximum_axes = 12`
- `axis_alignment_cosine = 0.9986`
- `side_moment_normal_cosine = 0.2`
- `side_moment_eigen_gap = 0.03`

**源码**

- [ProjectionExtrusion.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionExtrusion.cpp#L216-L290)
- [projection.conf](../apps/ParamCAD%20Studio/config/projection.conf#L3-L20)

### 模块 23｜拉伸区间与截面观测

**核心算法**

- 沿候选轴建立局部坐标系。
- 用深度分位数裁剪拉伸区间。
- 计算凸投影面积。
- 在区间两端按深度容差和法向余弦收集端面点；端面点不足时 `Cap` 为空。

**支持**

- 有明确两端端面的拉伸体。
- 凸投影面积和包络体积估计。
- 端面点、端面深度和法向。

**不支持 / 拒绝**

- 缺端面时该轴在单轴路径中被跳过。
- 端面点少于 12 时端面不可用。
- 端面法向余弦低于 0.9 时不认为是端面。
- 只处理垂直于轴的平面端面，不处理任意斜端面。
- 凸投影面积不是精确截面面积。

**关键阈值**

- `minimum_cap_points = 12`
- `cap_band_multiplier = 3`
- `cap_normal_cosine = 0.9`
- `minimum_depth_multiplier = 3`

**源码**

- [ProjectionExtrusion.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionExtrusion.cpp#L176-L214)

### 模块 24｜二维截面恢复

**核心算法**

- 对端面点做轮廓拟合，得到一个外环和若干孔。
- 对外环做简化、Alpha Shape 和圆拟合。
- 圆拟合要求至少 8 个顶点、半径下限、点到圆距离容差和最大角间隙。
- 不满足圆条件时退化为 `Line2D` 折线。
- 内孔需要多个深度带的内壁支持。

**支持**

- 一个连通外环。
- 圆形外轮廓。
- 由直线段组成的多边形轮廓。
- 有内壁证据的圆孔。

**不支持 / 拒绝**

- 多个互不连通的外环会失败。
- 只生成 `Circle2D` 和 `Line2D`，不生成圆弧、椭圆、样条。
- 圆拟合角间隙超过 60° 时退化为折线，不生成圆。
- 折线边数超过 80 时返回空。
- 内孔需要至少 3/5 深度带有壁观测。
- 不支持自由曲线截面。

**关键阈值**

- `minimum_circle_vertices = 8`
- `maximum_circle_gap_deg = 60`
- `maximum_sketch_edges = 80`
- `inner_wall_depth_bands = 5`
- `inner_wall_minimum_bands = 3`

**源码**

- [ProjectionProfile.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionProfile.cpp#L23-L131)

### 模块 25｜基体、加材与切除提案

**核心算法**

- 基体：从端面轮廓生成 `NewBody` 拉伸。
- 台阶：从中间面证据生成加材或切除提案。
- 直槽：要求轮廓恰好 4 条 `Line2D`，检查侧壁法向、深度覆盖、点数和距离。
- 盲孔/通孔：从圆柱证据、端面点、开口点、法向、半径、残差和深度生成 `Cut`。
- 提案包含草图、深度、方向、操作类型和支持点。

**支持**

- 基体拉伸。
- 台阶加材/切除。
- 矩形直槽。
- 圆形盲孔和通孔。
- 沿候选轴的 Add/Cut 操作。

**不支持 / 拒绝**

- 只支持挤出类操作，不支持旋转、扫掠、放样。
- 孔只支持圆形；槽只支持 4 条直线组成的矩形。
- 不支持任意形状口袋、异形孔、螺纹、沉孔。
- 缺端面、缺开口、壁覆盖不足时跳过提案。
- 提案是假设，必须通过试建和评分。
- 搜索最多 3 步操作，复杂零件可能无法表达。

**关键阈值**

- `cut_cylinder_coverage = 0.65`
- `capped_cylinder_coverage = 0.8`
- `minimum_opening_points = 6`
- `minimum_slot_supported_edges = 2`
- `blind_hole_normal_cosine = 0.95`
- `through_hole_opening_normal_cosine = 0.9`

**源码**

- [ProjectionProposals.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionProposals.cpp#L11-L214)

### 模块 26｜候选实体评分

**核心算法**

- 用 `BRepExtrema_DistShapeShape` 计算观测点到候选实体面的距离。
- 统计中位距离、P95 距离、内点率、法向平均误差。
- 统计“无支持面”的面积比例。
- 总分 = 距离损失 + 无支持面比例 + 操作惩罚 + 曲线惩罚。

**支持**

- 对已试建 B-Rep 做统一质量评估。
- 固定评分视图，候选之间可比较。
- 输出残差点 ID 和质量指标。

**不支持 / 拒绝**

- 评分点来自固定视图，最多 1800 个，不是全量点。
- 法向比较只在点命中面时进行。
- 没有体积占据/内部矛盾检查；内部检查由后续建模或装配模块负责。
- 评分阈值是启发式：内点率低于 0.9、P95 超过 3 倍容差、无支持面超过 4% 时拒绝。
- 不是物理测量精度或公差分析。

**关键阈值**

- `minimum_inlier_fraction = 0.9`
- `maximum_p95_multiplier = 3`
- `maximum_unsupported_face_fraction = 0.04`
- `score_maximum_distance_loss = 8`
- `score_normal_weight = 0.5`
- `score_operation_penalty = 0.015`

**源码**

- [ProjectionScoring.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionScoring.cpp#L24-L93)
- [ProjectionSearch.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionSearch.cpp#L9-L13)

### 模块 27｜操作序列搜索与选择

**核心算法**

- 先生成单轴、两轴求交和分段提案。
- 从基体程序出发，按操作扩展，使用 Beam Search 保留前 4 个状态。
- 每个候选试建并评分；只有分数改善、体积变化或接受状态满足条件才继续扩展。
- 返回 Accepted、InsufficientSupport、NoValidBase 或 Cancelled。

**支持**

- 最多 3 步操作的序列。
- 单轴拉伸、两轴求交、分段加材/切除。
- 操作序列的试建和评分。

**不支持 / 拒绝**

- 搜索不是穷举，Beam 宽度 4，可能错过更优序列。
- 没有有效基体时返回 `NoValidBase`。
- 支持不足或评分不达标时返回 `InsufficientSupport`。
- 两轴求交要求两轴不平行，轴余弦上限 0.5。
- 缺端面或深度不足时跳过候选轴。
- 超过 3 步的复杂零件无法表达。

**关键阈值**

- `maximum_operations = 3`
- `search_beam_width = 4`
- `minimum_score_improvement = 0.01`
- `minimum_boolean_volume_multiplier = 1`

**源码**

- [ProjectionSearch.cpp](../core/geometric_reconstruction/src/recognition/projection/ProjectionSearch.cpp#L109-L278)

## 7. 建模与编排

### 模块 28｜识别结果转建模计划

**核心算法**

- 把 `CompoundCandidate` 转成绑定当前修订的 `ModelPatch`。
- 命令顺序：长方体 → 棱柱（草图 + 拉伸）→ 贯穿孔 → 曲面实体 → 有限壳体 → 投影程序。
- `replacePrevious` 时按依赖逆序删除本工作流的重建特征。
- 手动编辑特征在未明确覆盖时受保护。

**支持**

- 长方体、棱柱、贯穿孔、球/圆柱/圆锥/圆环、有限壳体、投影操作序列。
- 修订绑定和手动修改保护。
- 在私有副本上预演命令，保证最终补丁原子应用。

**不支持 / 拒绝**

- 模型修订不匹配时拒绝。
- 存在手动编辑的重建特征且未允许覆盖时拒绝。
- 删除旧特征失败（例如存在用户创建的依赖特征）时拒绝。
- 投影程序第一条操作不是 `NewBody` 时拒绝。
- 不支持任意特征图或用户自定义命令。

**源码**

- [ModelingAdapter.cpp](../core/geometric_reconstruction/src/modeling/ModelingAdapter.cpp#L5-L103)
- [ModelPatch.h](../core/parametric_modeling/include/modeling/ModelPatch.h#L16-L29)

### 模块 29｜草图校验与构面

**核心算法**

- 在 XY 基准面上尝试构建草图面。
- 支持点、线、矩形、圆等草图实体。
- 对消费草图的特征，要求轮廓闭合；开放草图可以保留为编辑状态，但不能被拉伸/切除消费。

**支持**

- 由直线段、矩形和圆组成的闭合轮廓。
- 草图平面引用解析。
- 一个外环加若干内孔。
- 退化、未闭合、自交等诊断。

**不支持 / 拒绝**

- 开放轮廓不能消费。
- 自交、退化、零面积轮廓不能构面。
- 不支持圆弧、椭圆、样条和 3D 曲线。
- 多个互不包含的外环不能作为一个拉伸轮廓。

**源码**

- [SketchValidation.cpp](../core/parametric_modeling/src/SketchValidation.cpp#L9-L16)
- [SketchBuilder.cpp](../core/parametric_modeling/src/occt/SketchBuilder.cpp#L296)

### 模块 30｜特征重建与 B-Rep

**核心算法**

- 按特征历史顺序重放。
- 支持草图、拉伸、切除、长方体、有限壳体、贯穿孔、球/圆柱/圆锥/圆环。
- 拉伸支持 `NewBody`、`Join`、`Cut`、`Intersect`。
- 带目标体的拉伸要求目标存在，且操作后仍是单一实体。
- 任一特征失败时，当前实体保留到失败点，后续特征标记为跳过。

**支持**

- 参数化特征历史重放。
- OCCT 布尔运算和 B-Rep 构建。
- 依赖草图、目标体和语义面解析。
- 失败时停止重建并保留诊断。

**不支持 / 拒绝**

- 上游草图缺失、无闭合轮廓、目标体缺失时失败。
- 带目标体的布尔结果不是单一实体时失败。
- 通用布尔结果拓扑命名未实现；面引用可能随编辑变化。
- 不支持一般 NURBS、自由曲面、扫掠、放样和装配体。
- 失败后不会自动跳过失败特征继续重建，而是停止并标记后续特征。

**源码**

- [RebuildEngine.cpp](../core/parametric_modeling/src/RebuildEngine.cpp#L212-L413)
- [Builders.h](../core/parametric_modeling/src/occt/Builders.h)

### 模块 31｜编辑与原子提交

**核心算法**

- `ModelPatch` 携带 `expectedRevision` 和有序命令。
- 先校验修订，再逐条执行命令。
- 任一命令失败或重建失败时，把文档恢复为执行前快照。
- 只有全部成功才递增修订号并返回受影响特征。

**支持**

- 命令序列的原子提交。
- 重建失败整体回滚。
- 受影响特征 ID 返回。

**不支持 / 拒绝**

- 不支持部分提交。
- 修订过期时直接拒绝。
- 重建失败时不保留半成品。
- 替换旧特征时，如果存在用户创建的依赖特征，整个替换失败。
- 不提供数据库级事务、并发写或跨进程锁。

**源码**

- [ModelingCore.cpp](../core/parametric_modeling/src/ModelingCore.cpp#L65-L100)
- [PartDocument.cpp](../core/parametric_modeling/src/PartDocument.cpp)

### 模块 34｜生产编排与结果交接

**核心算法**

- 客户端抓取工作点云、点 ID、模板、人工关系、缓存分析和修订号。
- 后台 `QThread` 执行完整重建，按阶段发出状态信号。
- 结果回到工作区线程后再次核对点云、模型和实体修订号。
- 成功则提交建模候选；装配部分成功时保留开放部件和诊断。

**支持**

- 完整生产任务、取消、异常转错误。
- 装配部分成功和开放部件显示。
- 结果缓存（带指纹的表面分析）。
- 模板拟合结果回填和关系重新推断。

**不支持 / 拒绝**

- 取消只在阶段边界生效，已进入的 CGAL/OCCT/装配计算不会强制中断。
- 计算期间点云、模型或实体修订号变化时，整轮结果丢弃。
- 建模提交失败时，后台的 `Success` 不会写入模型。
- 不自动重试、不做增量重建、不做跨会话学习。
- 后台线程不访问模型文档、视口或工作区状态。

**源码**

- [CadReconstructionTask.cpp](../apps/ParamCAD%20Studio/src/app/CadReconstructionTask.cpp#L31-L265)
- [PointCloudWorkspaceController.cpp](../apps/ParamCAD%20Studio/src/app/PointCloudWorkspaceController.cpp#L557-L639)
- [PointCloudWorkspaceController.cpp](../apps/ParamCAD%20Studio/src/app/PointCloudWorkspaceController.cpp#L836-L979)

## 8. 跨模块边界总结

### 8.1 当前整体支持

- 五类解析曲面检测：平面、圆柱、球、圆锥、圆环。
- 完整六面长方体、轴对齐长方体。
- 正交框架下的组合长方体/棱柱、平面加强筋。
- 单箱体上的轴向圆通孔。
- 独立完整球、带端面圆柱、圆锥/圆台、完整圆环。
- 薄矩形宿主上的通孔、附着曲面体/平面组合体、矩形切除。
- 有限平面面片的 Alpha Shape 边界和凸壳闭合。
- 特定几何族的专用恢复：解析轮廓体、环形管、圆盘体、局部轮廓。
- 单轴、两轴求交、分段投影拉伸，含基体、台阶、矩形槽、圆孔。
- 特征历史、原子补丁、撤销/重做、PCAD 保存、STEP 导出。

### 8.2 当前整体不支持或不可靠

- 任意机械零件的通用识别。
- 严重遮挡、缺面、只观测到部分曲面的情况。
- 圆角、倒角、过渡曲面、自由曲面、NURBS。
- 盲孔、沉孔、螺纹孔、锥孔、异形孔、相交孔（常规识别不支持；投影只支持有限圆孔/槽）。
- 任意凹形多面体、非流形实体、多材料零件。
- 装配配合、运动副、装配树和装配约束。
- 通用布尔结果拓扑命名；面引用在编辑后可能失效。
- 自动保证“拆出的部件等于真实零件实例”；凹接触和粘连仍有歧义。
- 自动恢复真实尺度、自动补齐完全未观测的几何。

### 8.3 输入质量要求

- 毫米尺度或用户确认的毫米假设。
- 足够密的点云；薄壁和孔壁需要可分辨的采样。
- 法向方向已知，或至少存在可传播的定向种子。
- 关键面有足够支撑点；端面、盖面、孔壁不能完全缺失。
- 离群点以稀疏飞点为主；密集离群块不会被统计过滤去除。
- 点云中不能有重复 ID、非有限坐标或跨快照混用。

### 8.4 参数敏感性

参数会直接改变模块的“支持/不支持”边界：

| 参数 | 影响 |
| --- | --- |
| `voxel_diagonal_fraction` | 越大越省算力，但越容易吞掉小特征 |
| `normal_angle_deg` | 越大面片越容易合并，越小越容易过分割 |
| `connectivity_factor` | 越大越容易跨间隙连接，越小越容易断裂 |
| `minimum_patch_points` | 越大越能抑制噪声面片，但会丢掉小面 |
| `alpha_factor` | 越大边界越容易粘连，越小越容易断裂 |
| `minimum_part_patches` | 越大闭合越严格，但简单壳体可能无法闭合 |
| `maximum_projection_parts` | 越大开放部件回退越多，但计算量和误拟合风险增加 |
| `minimum_surface_coverage` | 越大越严格，越容易拒绝部分扫描 |
| `minimum_inlier_fraction` | 越大投影候选越严格，越容易拒绝 |
| `maximum_operations` | 越大表达力越强，但搜索空间和误拟合风险增加 |

## 9. 源码索引

| 模块 | 源码 |
| --- | --- |
| 01 快照/单位 | [PointCloudAdapter.cpp](../apps/ParamCAD%20Studio/src/io/PointCloudAdapter.cpp) |
| 02–04 预处理 | [preprocessing](../core/geometric_reconstruction/src/preprocessing) |
| 05–06 检测 | [detection](../core/geometric_reconstruction/src/detection) |
| 07–11 分析与修复 | [analysis](../core/geometric_reconstruction/src/analysis) · [RepairTransaction.cpp](../core/geometric_reconstruction/src/recognition/RepairTransaction.cpp) |
| 12–17 常规识别 | [recognition](../core/geometric_reconstruction/src/recognition) |
| 18–21 装配 | [assembly](../core/geometric_reconstruction/src/recognition/assembly) |
| 22–27 投影 | [projection](../core/geometric_reconstruction/src/recognition/projection) |
| 28–31 建模 | [modeling](../core/geometric_reconstruction/src/modeling) · [parametric_modeling](../core/parametric_modeling/src) |
| 34 编排 | [CadReconstructionTask.cpp](../apps/ParamCAD%20Studio/src/app/CadReconstructionTask.cpp) · [PointCloudWorkspaceController.cpp](../apps/ParamCAD%20Studio/src/app/PointCloudWorkspaceController.cpp) |
| 配置 | [preprocessing.conf](../apps/ParamCAD%20Studio/config/preprocessing.conf) · [detection.conf](../apps/ParamCAD%20Studio/config/detection.conf) · [analysis.conf](../apps/ParamCAD%20Studio/config/analysis.conf) · [recognition.conf](../apps/ParamCAD%20Studio/config/recognition.conf) · [assembly.conf](../apps/ParamCAD%20Studio/config/assembly.conf) · [projection.conf](../apps/ParamCAD%20Studio/config/projection.conf) |
