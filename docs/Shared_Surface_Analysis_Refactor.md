# 共享曲面分析与混合特征重建

目录已按功能整理：共享分析实现位于 `src/analysis/`，实体识别实现位于
`src/recognition/` 及其 `compound/`、`curved/`、`guided/`、`mixed/` 子目录。
生产流程入口为 `src/recognition/ParametricSolidRecognizer.cpp`。完整导航见
[`几何重建模块目录说明`](../core/geometric_reconstruction/README.md#directory-guide)。

## 数据与调用链

```text
原始 PLY / 工作点云
  → PointStore + 预处理
  → analyzeSurfaces()
      分类型检测 → 曲面证据 → 连续面片 → 关系假设
  → SurfaceAnalysisResult
      ├─ 客户端显示与区域归属
      ├─ 长方体 / 孔识别
      ├─ 独立曲面实体识别
      ├─ 组合平面结构识别
      └─ 薄底板上的局部混合特征识别
  → CompoundCandidate
  → 原子 ModelPatch → OpenCASCADE B-Rep → STEP
```

`core/geometric_reconstruction/include/reconstruction/analysis/SurfaceAnalysis.h` 是共享边界。
它不依赖 Qt、VTK 或客户端 JSON。低层 `detectPrimitiveEvidence()` 保留；统一分析层
默认分别调用启用的类型，避免圆锥等竞争性候选先占用圆柱观测。共享并不等于仅执行
一次联合 RANSAC。每轮分析记录检测次数，实体候选构造不再自行重新执行全局检测。

`PrimitiveEvidence` 表达解析曲面方程；`SurfacePatch` 表达由该方程解释的连续观测区域。
同一个曲面可关联多个面片，同一个点也可暂时支持多个曲面候选。UI 的单一归属只是
显示选择，不能反过来删除核心层的竞争候选。面片保存支持点、AABB、质心、平均材料
法向和定向状态；当前没有输出精确裁剪环或完整 B-Rep 边拓扑。

关系包括邻接、平行、垂直、共面、同轴、等半径、候选相交、相切，以及轴垂直/平行
平面。邻接针对有限支持点查询；共面、平行可以发生在远处的面片之间，不代表接触。
候选相交仍是非平行面片邻接的证据，不是已计算出的精确交线。关系是带置信度的假设，
不会全部自动转成精确约束。

## 识别入口

- `recognizeBoxFromAnalysis()`：读取共享平面证据，兼容单箱体完成路径。
- `recognizeBoxWithThroughHolesFromAnalysis()`：共享平面和圆柱证据解释箱体与孔。
- `recognizeCurvedSolidFromAnalysis()`：读取共享曲面证据，校验独立实体的完整边界。
- `recognizeParametricSolidFromAnalysis()`：共享分析后的自动重建入口。
- `reconstructParametricSolid()`：兼容包装，先分析一次，再调用上述入口。
- `reconstructGuidedSolid()`：可接收有效共享分析；人工类型提示允许对选区或扩展区域
  做有界补检，修改工作观测之后重新计算受影响分析。原始点云不被改写。

组合平面识别继续使用点法向与材料网格进行占据验证，并复用共享的平面方向精修。
点仍用于几何精修、材料内外验证和覆盖率计算；共享曲面不是丢弃点云。

## 快照、单位与交互

`SurfaceAnalysisResult` 保存单位、检测/关系参数和源点指纹。点指纹覆盖点序、ID、
位置、法向及其定向状态、置信度。识别入口拒绝不同快照；后台缓存还比较分析设置。
删除/修改点、单位转换、法向修订或参数变化会使旧分析不适用。
`SurfaceId`、`PatchId` 只在所属分析指纹内有效，不能跨轮次裸引用。

客户端仍在原 PLY 点 ID 与工作点 ID 之间显式转换。表面识别与 CAD 任务共享同一套
核心分析实现；只有快照、单位和参数一致时复用缓存。区域子集分析不能误当作全场分析，
米制显示分析也不能未经转换直接用于毫米建模。

手动关系随区域支持点传入核心，与当前面片匹配后验证。与观测冲突或无法确定端点的
关系会报告失败，不会静默挪动几何。删除自动关系的 UI 抑制记录继续属于标注状态，
不表示禁止核心根据几何提出该关系。自动与人工来源保持区分。

## 当前混合识别范围

`MixedFeatureRecognizer` 面向一个有观测支持的薄矩形宿主底板：

- 从占优势的平行平面和周边平面恢复宿主及材料侧。
- 在宿主上方按空间连通性建立局部候选区域，各区域可有独立 XY 方向。
- 平面结构使用局部坐标系、共享平面精修与材料网格，支持旋转长方体、台阶和带斜筋 L 型件。
- 圆柱/圆台凸台结合侧壁、观测顶面、轴与平面的关系和宿主接触面完成。
- 圆孔由朝向空腔的圆柱支持点、宿主范围及贯穿跨度验证。
- 矩形孔由四个朝向空腔的平面壁和宿主厚度验证，并输出减料拉伸。
- 局部曲面完成要求侧壁/顶面覆盖；缺失顶面会拒绝。宿主接触面是显式结构推断，
  不作为虚构点混入原始观测。与宿主连接的局部框架共享接触平面，避免噪声导致微缝。
- 组合模型要求至少 98% 的整体定向边界一致性，并逐一处理足够支持的局部区域；
  不能靠大底板覆盖率高而忽略未能解释的凸台。

这是一条有边界的局部候选组合路径，不是任意零件的全局 B-Rep 拓扑求解器。
任意倾斜凸台、自由曲面、多个复杂宿主、精确边界环与 GlobFit 式约束子集优化仍需扩展。
混合路径不声称证明全局最少特征数；原有组合网格路径的证明标志只对其自身网格有效。

`Sphere/Cylinder/Cone/TorusPrimitiveParams` 增加默认关闭的 `join` 字段。
独立实体保持原行为，连接凸台采用布尔融合；项目编码保存该字段，旧项目缺省为 false。
`PrismCandidate::subtract` 区分加强筋加料与矩形孔减料。

预处理默认体素从整体对角线的 0.5% 调细到 0.2%，以保留大底板的小孔壁和薄筋。
它增加了大点云的处理量；特征远小于采样分辨率时仍需更细输入/局部重采样，不能放宽
完整性阈值来补造不存在的观测。

## 验证

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target surface_analysis_test geometric_reconstruction_regression_test cad_reconstruction_validation
ctest --test-dir "apps/ParamCAD Studio/build/msvc-debug" -C Release --output-on-failure -R "surface_analysis_test|geometric_reconstruction_regression_test|point_cloud_analysis_test|point_cloud_relations_test|cad_reconstruction_task_test|project_store_test"
python tools/run_mixed_feature_fit_tests.py
python tools/run_compound_fit_tests.py --noise 0.02 --output outputs/shared-surface-compound
python tools/run_curved_fit_tests.py --noise 0.02 --output outputs/shared-surface-curved
```

混合验证器从源尺寸独立构造参考 STEP，检查源/重建体积对称差、体积采样置信上界、
局部盒体位置/尺寸、圆柱/圆台参数、方孔尺寸和单实体 STEP 往返。盒体比较允许与
源叠层等价的最大重叠盒体分解，不能要求恢复不可唯一辨认的原始建模历史。
直接点云保留原坐标，参考 STEP 验收也保留原位置；显示导入时的自动居中行为不变。
源真值标签和尺寸文件仅用于独立验收，不传给识别器。

## 局部失败补检与关系优化（第一版）

混合特征识别在局部候选不能完成实体时，记录 `LocalAnalysisFailure`：
`insufficient_support`、`missing_end_face`、`candidate_conflict`、`invalid_relation`。
记录保存失败点 ID、原始候选、取回点 ID、来源指纹、局部指纹、补检后的候选/面片/关系及重试结果。
类别表示当前可见的直接阻碍，不表示已证明唯一根因。客户端后台的 `analysis.localFailures`
和批量验证的 `local_analysis_failures` 可用于定位；没有新增诊断 UI。

`recoverLocalSurfaces()` 在失败点包围盒及两倍检测距离的邻域取回真实观测，
要求单位、坐标和保留点身份匹配。工作点保持原位；新取回点须具有已定向法向。
稳定宿主平面已经解释的观测不作为局部特征的新边界。调用端完成单位转换后传入
同一坐标系的高分辨率来源。两轮独立随机流执行补检，局部最低支持量降为不超过
20 点，检测距离、材料方向和完整性阈值不变。原候选与重叠支持保留。

`optimizeSurfaceRelations()` 对平面、圆柱实现平行、垂直、同轴约束的联合拟合：
按置信度逐步试加入关系，用阻尼最小二乘同时调整参与曲面，以观测和关系残差共同评分。
每次试解复核整个已接受子集，冲突或观测退化则回退该关系。方向偏移受角度容差限制，
原支持点至少 90% 须仍在两倍检测距离内，平均误差增长不得超过检测距离的 15%。
通过后在原支持包围盒内重新评估支持点，允许多候选共享点，再重建面片和关系；
人工关系通过支持点重新匹配验证。未参与关系的曲面参数保持不变。
每次最多试 32 个唯一关系、联合调整 12 个曲面；求解使用确定性子样本，验收使用全部原支持点。
预算外未接受的关系也保留在报告中。

实体重试仍检查顶面、侧壁覆盖、贯穿跨度和整体材料边界。局部成功不会改写全局工作点
快照；局部面片及关系存入失败记录，其 ID 属于记录的局部指纹。缺失顶面不生成虚拟观测。
不足 20 点且补检仍不能建立实体的孤立残片保留诊断，沿用整体边界覆盖验收；完全未采到
的特征和超出失败区包围盒的缺失范围仍不能恢复。圆锥/球/环面参数尚未加入优化变量，
该版本不声称全局最优。

验证：`python tools/run_local_recovery_tests.py`。
矩阵在同一混合底板上增加 0.04/0.08 mm 噪声，把小盒特征采样密度降至 25%/10%，
把尺寸降至 50%/25%，并删除圆柱凸台顶面检验拒绝。完整观测案例使用独立源 STEP
检查参数、体积差及单实体结果；缺失案例要求拒绝、保留端面缺失诊断且不输出 STEP。
核心测试另覆盖高分辨率顶面取回、源单位不符、非法点 ID、冲突关系筛选、同轴拟合及稳定性。

混合区域分组按局部 6 邻域间距自适应连接，并使用双方较小的连接半径；
半径上限为原场景连接距离的四倍。密度不同的特征不再完全依赖大底板密度，
远隔特征仍不能通过无上限扩张合并。矩形孔在最多 12 个候选墙面中枚举四墙组合，
每个组合仍须解释全部局部观测，避免额外竞争平面导致完整矩形孔被直接拒绝。

当失败区不足 20 点时，来源查询允许扩展至关系距离容差的 16 倍，随后只保留与失败点
连通的来源分量；其连接距离也有上限。这样可从预处理后仅剩底部残片的区域，重新取回
仍存在于原始输入中的上部观测。已取回的工作点成员随修复记录，后续分组跳过已经解释
的成员，避免同一特征被重复重建。该分支不扩大检测误差或实体完整性容差。
