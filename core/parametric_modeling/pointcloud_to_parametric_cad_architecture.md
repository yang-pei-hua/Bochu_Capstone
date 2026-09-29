# 3D 点云转参数化 CAD 模型：实现路径与工程约束

## 1. 文档目标

本文档定义一套从 **3D Point Cloud（点云）** 到 **Parametric CAD Model（参数化 CAD 模型）** 再到 **B-Rep / STEP** 的实现路径。

当前阶段重点不是渲染，而是解决以下问题：

1. 从点云中识别基础几何表面；
2. 将几何观测组织为可追踪的证据；
3. 从表面关系推断可编辑的 CAD Feature；
4. 维护独立于第三方库的参数化模型状态；
5. 支持用户对局部区域进行纠正和重新拟合；
6. 将参数化模型稳定地转换为 OpenCASCADE B-Rep，并导出 STEP。

核心原则：

> **参数化 Feature Graph 是模型的唯一核心状态。点云算法库、OpenCASCADE、后续渲染库都只作为可替换的执行后端，不拥有业务层模型状态。**

---

## 2. 总体流程

```text
Point Cloud
    ↓
Surface Detection
    ↓
Evidence Graph
    ↓
Feature Recognition
    ↓
Parametric Feature Graph      ← 核心状态
    ↑
Local Refit / User Correction
    ↑
Natural Language Command

Parametric Feature Graph
    ↓
OpenCASCADE Adapter
    ↓
B-Rep
    ↓
STEP
```

当前阶段不处理 VTK 渲染逻辑。后续如需显示，可从 B-Rep 或 Feature Graph 派生显示数据，但不得反向让 VTK 成为模型状态来源。

---

# 3. 系统分层

建议工程分为六个核心层：

```text
01. Point Cloud Layer
02. Surface Detection Layer
03. Evidence Graph Layer
04. Feature Recognition Layer
05. Parametric Feature Graph Layer
06. CAD Kernel Adapter Layer
```

另外提供两个修改入口：

```text
Local Refit / User Correction
Natural Language Command
```

其中最核心的是：

```text
Parametric Feature Graph
```

其他所有层都围绕它提供“观测、推断、修改或几何生成”。

---

# 4. Point Cloud Layer

## 4.1 职责

Point Cloud Layer 保存输入的原始三维观测数据，并提供基础预处理能力。

输入可能来自：

- 多视角图像重建；
- 深度摄像头；
- 激光扫描；
- 测试数据集；
- 人工生成的点云。

基础数据至少包含：

```text
Point {
    id
    position: x, y, z
    optional normal
    optional confidence
}
```

建议每一个点都具有项目内部唯一的 `PointId`。

---

## 4.2 主要处理

该层可执行：

```text
去除离群点
体素降采样
法向估计
邻域查询
空间索引
ROI 提取
```

推荐使用：

- PCL：点云过滤、法向、KdTree、RANSAC 等；
- Eigen：向量、矩阵、PCA、SVD、坐标变换。

---

## 4.3 最重要的工程约束

### 原始点云不可被破坏

禁止采用：

```text
检测出 Plane
→ 删除 Plane 对应点
→ 对剩余点继续拟合
```

作为唯一状态。

正确做法：

```text
Raw Point Store
    ├── PointId 1
    ├── PointId 2
    ├── PointId 3
    └── ...

Detection Result
    └── 引用 PointId
```

即：

> 拟合算法只能创建 Point 的“引用集合”，不能破坏原始观测。

原因是后续可能需要：

- 局部重新拟合；
- 检查漏掉的孔；
- 重新解释某个区域；
- 修改容差重新运行算法；
- 比较两个候选模型。

---

# 5. Surface Detection Layer

## 5.1 职责

Surface Detection 不直接判断“这是一个 Box”。

它只负责回答：

> 点云中存在哪些可解释的基础几何表面？

第一阶段支持：

```text
Plane
Sphere
```

后续增加：

```text
Cylinder
Cone
Torus
Circle / Boundary
Free-form Surface
```

---

## 5.2 输出形式

每一次检测应该输出统一结果：

```text
SurfaceCandidate {
    surface_id
    type
    parameters
    support_point_ids
    residual_error
    coverage
    confidence
}
```

例如：

```text
Plane {
    normal
    offset
}

Sphere {
    center
    radius
}

Cylinder {
    axis_origin
    axis_direction
    radius
}
```

---

## 5.3 推荐算法

基础流程：

```text
PointCloud
    ↓
Normal Estimation
    ↓
RANSAC / Region Growing
    ↓
初始参数
    ↓
Least Squares / Robust Optimization
    ↓
SurfaceCandidate
```

推荐：

- PCL：RANSAC、Plane、Sphere、Cylinder、Circle、Boundary 等；
- Eigen：PCA 和解析拟合；
- Ceres Solver：第二阶段用于参数精修，不必在 MVP 强制加入。

---

## 5.4 Surface Detection 不负责什么

禁止在这一层直接产生：

```text
Box
TriangularPrism
HoleFeature
ExtrudeFeature
```

因为这些已经属于更高层的“Feature 语义”。

Surface Detection 只描述：

```text
这里存在一个平面
这里存在一个球面
这里存在一个圆柱面
```

---

# 6. Evidence Graph

## 6.1 为什么需要 Evidence Graph

仅保存拟合后的参数不够。

系统还必须知道：

> 为什么认为这里存在这个几何结构？

因此需要单独维护一个“几何证据层”。

Evidence Graph 描述的是：

```text
Point Cloud 中观察到了什么
```

而不是：

```text
最终 CAD 应该是什么
```

---

## 6.2 节点

第一阶段节点主要是：

```text
SurfaceEvidence
```

例如：

```text
SurfaceEvidence {
    id
    type = Plane
    parameters
    support_points
    nearby_residual_points
    rmse
    coverage
    confidence
}
```

---

## 6.3 边

Evidence Graph 中保存表面之间的关系：

```text
Parallel
Perpendicular
Intersect
Coaxial
Tangent
SameRadius
Opposite
Adjacent
```

例如：

```text
Plane A ──parallel── Plane B

Plane A ──perpendicular── Plane C

Cylinder A ──coaxial── Cylinder B
```

---

## 6.4 为什么这一层很重要

例如一个 Box 可以被描述为：

```text
P1 // P2
P3 // P4
P5 // P6

P1 ⟂ P3
P1 ⟂ P5
P3 ⟂ P5
```

这些属于“证据”。

Feature Recognition 再根据这些证据判断：

```text
这是一个 Box
```

这样以后即使 Feature 判断发生变化，也不需要重新跑最底层点云算法。

---

# 7. Feature Recognition

## 7.1 职责

Feature Recognition 将多个低层 Surface Evidence 组合成具有 CAD 语义的 Feature。

它回答：

> 这些平面、球面和圆柱面组合起来，最可能代表什么 CAD 结构？

第一阶段支持：

```text
Box
Triangular Prism
Sphere
```

后续增加：

```text
Extrude
Cylinder Solid
Hole
Pocket
Boss
Pattern
Chamfer
Fillet
```

---

# 8. Box Recognition

Box 不应直接通过一个专用黑盒 Fitter 得到。

推荐从 6 个 Plane 推断：

```text
P1 // P2
P3 // P4
P5 // P6
```

并满足三组方向近似互相垂直：

```text
N1 ⟂ N3
N1 ⟂ N5
N3 ⟂ N5
```

然后计算：

```text
center
local X/Y/Z
length
width
height
```

最终产生：

```text
BoxFeature {
    center
    orientation
    size_x
    size_y
    size_z
}
```

---

# 9. Triangular Prism Recognition

三角拉伸体建议识别为：

```text
Triangle Profile
+
Extrusion Direction
+
Extrusion Depth
```

而不是简单保存 5 个 Plane。

可能的证据结构：

```text
2 个平行端面
+
3 个侧面
```

或者：

```text
截面三角形
+
统一拉伸方向
```

最终产生：

```text
ExtrudeFeature {
    profile = Triangle(A, B, C)
    direction
    depth
}
```

这样以后：

```text
四边形柱体
五边形柱体
任意多边形拉伸体
```

都可以复用相同 Feature 类型。

---

# 10. Sphere Recognition

Sphere 是最简单的独立 Feature。

Evidence：

```text
Sphere Surface
```

如果球面覆盖率和残差满足条件，则产生：

```text
SphereFeature {
    center
    radius
}
```

---

# 11. Parametric Feature Graph

## 11.1 这是整个系统的核心状态

Feature Graph 描述：

> CAD 模型实际上是什么，以及它是如何构造出来的。

例如：

```text
BoxFeature #1
    ↓
HoleFeature #2
    ↓
ChamferFeature #3
```

或者：

```text
TriangleProfile
    ↓
ExtrudeFeature
    ↓
HoleCutFeature
```

---

## 11.2 Feature 的基本接口

建议统一定义：

```text
Feature {
    FeatureId
    FeatureType
    Parameters
    Parent / Dependency
    Evidence References
    Semantic Faces
}
```

例如：

```text
BoxFeature {
    FeatureId
    center
    rotation
    size
}
```

```text
HoleFeature {
    FeatureId
    support_face
    center_uv
    radius
    depth
}
```

---

## 11.3 Feature Graph 而不是 B-Rep 才是模型真源

必须满足：

```text
Feature Graph
    ↓ generate
B-Rep
```

而不是：

```text
B-Rep
    ↓ reverse guess
Feature Graph
```

OpenCASCADE 生成出的 `TopoDS_Shape` 属于派生数据。

只要 Feature Graph 没丢：

```text
B-Rep 可以重新生成
STEP 可以重新导出
显示数据可以重新生成
```

---

# 12. Semantic Face

为了支持：

```text
“这个面应该有一个孔”
```

不能让用户操作直接绑定 OCC 的 Face index。

项目内部需要定义稳定的：

```text
FaceId
```

例如：

```text
Box #12

Face 101 = +X
Face 102 = -X
Face 103 = +Y
Face 104 = -Y
Face 105 = +Z
Face 106 = -Z
```

这些叫：

```text
Semantic Face
```

---

## 12.1 Semantic Face 的意义

例如：

```text
FaceId = 105
role = BoxTop
```

打孔前：

```text
──────────────
```

打孔后：

```text
──────○───────
```

从 OCC 拓扑角度，这个 Face 可能已经被修改。

但业务语义上：

```text
FaceId = 105
```

仍然是“这个 Box 的顶面”。

---

# 13. Local Refit / User Correction

这是系统必须从第一版架构上预留的能力。

用户可能指出：

```text
这个面应该有孔
这里应该有槽
这个面拟合错了
这个区域应该重新识别
```

系统不应该因此重新拟合整个模型。

---

# 14. 局部重拟合总体流程

```text
用户指定 Feature / Face / Region
        ↓
确定 Semantic Face
        ↓
取得对应 Surface Evidence
        ↓
取得该区域原始 PointId
        ↓
取得 nearby residual points
        ↓
建立局部坐标系
        ↓
执行目标导向的局部检测
        ↓
产生新的 Feature Candidate
        ↓
修改 Feature Graph
        ↓
局部或整体重新生成 B-Rep
```

---

# 15. 示例：用户指出某个面上存在圆孔

假设当前系统只有：

```text
BoxFeature
```

用户指定：

```text
Face #105
```

并要求：

```text
该面应该存在孔
```

---

## 15.1 获取局部点云

根据 Face #105 的平面参数：

```text
origin
normal
U axis
V axis
```

建立局部坐标：

```text
3D point
    ↓
(u, v, h)
```

然后只取：

```text
该 Face 的 support points
+
该 Face 附近 residual points
```

---

## 15.2 目标导向拟合

因为用户已经明确：

```text
目标类型 = Hole
```

不需要重新运行所有 Primitive Detector。

只运行：

```text
Cylinder Detection
Circle Boundary Detection
```

可能路径一：

```text
Residual Points
    ↓
Cylinder RANSAC
    ↓
axis ≈ face normal
    ↓
得到 center / radius
```

可能路径二：

```text
Face Boundary
    ↓
Circle Detection
    ↓
得到 center / radius
```

---

## 15.3 生成 Feature Candidate

```text
HoleCandidate {
    support_face
    center_uv
    radius
    axis
    depth
    confidence
    evidence
}
```

确认后写入：

```text
Feature Graph
```

变成：

```text
BoxFeature
    ↓
HoleFeature
```

---

# 16. Natural Language Command

AI 在整个系统中只负责：

```text
Natural Language
    ↓
Structured Command
```

例如用户输入：

```text
这个面中间应该有一个贯穿孔
```

AI 转换为：

```text
RequestLocalRefit {
    target_face = selected_face
    target_feature = Hole
    depth_mode = ThroughAll
}
```

然后：

```text
几何算法
```

负责从点云中确定：

```text
center
radius
```

---

## 16.1 AI 不应该负责的内容

AI 不直接决定：

```text
球心是多少
孔半径是多少
Box 长宽高是多少
某几个点是否属于 Plane
```

这些由确定性几何算法负责。

---

# 17. OpenCASCADE Adapter

## 17.1 OCC 的定位

OpenCASCADE 只负责：

```text
参数化 Feature
    ↓
B-Rep Geometry / Topology
```

包括：

```text
Plane
Sphere
Cylinder
Wire
Face
Shell
Solid

Extrude
Revolve
Boolean Cut
Boolean Fuse
Chamfer
Fillet

STEP Export
```

---

## 17.2 OCC Adapter 输入

OCC 不读取业务状态。

它只接受：

```text
Feature Graph Snapshot
```

例如：

```text
BoxFeature
HoleFeature
```

并转换为：

```text
TopoDS_Shape
```

---

## 17.3 OCC Adapter 输出

```text
BRepBuildResult {
    shape
    topology_bindings
    validation_result
}
```

其中：

```text
topology_bindings
```

负责将：

```text
Semantic FaceId
```

映射到当前：

```text
TopoDS_Face
```

---

# 18. OCC 拓扑变化与状态维护

例如：

```text
Box
```

执行：

```text
Boolean Cut(Hole)
```

以后，一个原来的 Face 可能：

```text
Modified
Split
Deleted
```

因此：

> 不允许使用 `TopoDS_Face` 本身作为永久业务 ID。

应维护：

```text
FaceId
    ↓
BRepBinding
    ↓
current TopoDS_Face
```

每次 OCC 操作以后更新 Binding。

---

# 19. B-Rep

B-Rep 是 OpenCASCADE 生成的精确 CAD 几何表示。

典型结构：

```text
Solid
 └── Shell
      └── Face
           ├── Surface
           └── Wire
                └── Edge
                     └── Curve
```

它包含：

```text
Face
Edge
Vertex
Surface
Curve
Topology
```

B-Rep 是：

```text
Feature Graph 的几何执行结果
```

而不是项目核心状态。

---

# 20. STEP

最终使用 OpenCASCADE：

```text
TopoDS_Shape
    ↓
STEP Writer
    ↓
.step
```

STEP 用于：

```text
外部 CAD 软件打开
后续编辑
模型交换
测试输出
```

第一阶段目标应确保：

```text
参数化模型
→ B-Rep
→ STEP
```

全流程不经过：

```text
Triangle Mesh
→ Mesh to CAD
```

---

# 21. 第三方库职责边界

推荐第一阶段依赖：

| 库 | 职责 |
|---|---|
| PCL | 点云处理、法向、RANSAC、边界、基础 Primitive |
| Eigen | 数学、PCA、SVD、坐标变换 |
| Ceres | 可选：参数非线性精修 |
| OpenCASCADE | B-Rep、Boolean、Feature 几何生成、STEP |
| Qt | 客户端/UI，当前算法层不依赖 |
| VTK | 后续显示，当前流程不依赖 |

暂时不强制引入：

```text
CGAL
Open3D
OCAF
```

如以后需要，可通过 Adapter 接入。

---

# 22. 第三方库隔离原则

所有第三方库必须通过 Adapter 使用。

目录示例：

```text
src/
├── core/
│   ├── geometry/
│   ├── model/
│   ├── feature/
│   ├── evidence/
│   └── commands/
│
├── reconstruction/
│   ├── surface_detection/
│   ├── relation_analysis/
│   ├── feature_recognition/
│   └── local_refit/
│
├── adapters/
│   ├── pcl/
│   ├── occ/
│   └── ceres/
│
└── io/
    ├── pointcloud/
    └── step/
```

核心代码禁止：

```cpp
#include <pcl/...>
#include <TopoDS_Face.hxx>
```

除非处于 Adapter 层。

---

# 23. 核心接口建议

## Primitive Fitter

```text
IPrimitiveFitter

fit(
    PointView,
    FitRequest
) -> FitResult
```

输入：

```text
项目自己的 PointView
```

输出：

```text
项目自己的 FitResult
```

禁止返回：

```text
pcl::ModelCoefficients
```

作为业务层结果。

---

## CAD Builder

```text
ICadBuilder

build(
    FeatureGraph
) -> CadBuildResult
```

业务层只知道：

```text
CadBuildResult
```

OCC 类型只能存在于 OCC Adapter 内部。

---

# 24. 状态分为三类

建议明确区分：

## A. Source State

不可随算法运行丢失：

```text
Raw Point Cloud
User Constraints
Imported Metadata
```

## B. Core Model State

项目真正需要保存：

```text
Evidence Graph
Feature Graph
Semantic IDs
Command History
```

## C. Derived State

可以随时重新生成：

```text
PCL temporary cloud
KdTree
RANSAC temporary model
TopoDS_Shape
STEP
未来的 VTK PolyData
```

---

# 25. Undo / Redo

因为 Feature Graph 是唯一真源：

```text
Command
    ↓
Feature Graph v1
    ↓
Feature Graph v2
```

Undo 只需要：

```text
恢复 Feature Graph
```

然后：

```text
重新生成 B-Rep
```

而不需要反向恢复复杂 OCC Topology。

第一版建议：

```text
Command Pattern
+
Model Snapshot / Diff
```

自己管理 Undo / Redo。

---

# 26. MVP 实现顺序

## Phase 1：基础数据

实现：

```text
PointStore
PointId
FeatureId
FaceId
SurfaceId
```

以及：

```text
PCL Adapter
Eigen Math
```

---

## Phase 2：基础 Surface Detection

实现：

```text
Plane
Sphere
```

随后加入：

```text
Cylinder
```

输出统一：

```text
SurfaceEvidence
```

---

## Phase 3：Evidence Graph

实现：

```text
Parallel
Perpendicular
Intersection
Coaxial
```

先满足 Box / Prism 判断需要。

---

## Phase 4：Feature Recognition

优先：

```text
Sphere
Box
Triangular Extrude
```

不要一开始支持过多 CAD Feature。

---

## Phase 5：Feature Graph → OCC

实现：

```text
SphereFeature
BoxFeature
ExtrudeFeature
```

生成：

```text
TopoDS_Shape
```

并导出 STEP。

---

## Phase 6：Semantic Face

实现：

```text
Feature Face
↔
FaceId
↔
TopoDS_Face
```

为后续局部编辑打基础。

---

## Phase 7：Local Refit

第一个局部修改功能建议就是：

```text
在指定 Plane Face 上识别 Hole
```

实现：

```text
Face ROI
↓
Residual Point Extraction
↓
Cylinder / Circle Detection
↓
HoleFeature
↓
OCC Boolean Cut
```

这个功能一旦跑通，就说明系统架构真正具备“可修改性”。

---

## Phase 8：Natural Language Command

最后才接：

```text
LLM
```

首先支持少量结构化命令：

```text
AddHole
RefitFace
ResizeFeature
DeleteFeature
SplitFeature
```

LLM 只负责：

```text
文字 → Command
```

不直接调用 PCL / OCC。

---

# 27. 核心工程约束

## 约束 1：第三方对象不是业务状态

禁止：

```text
PCL Cloud = Model
TopoDS_Shape = Model
VTK PolyData = Model
```

正确：

```text
FeatureGraph = Model
```

---

## 约束 2：所有对象必须有项目内部稳定 ID

必须存在：

```text
PointId
SurfaceId
FeatureId
FaceId
```

不能依赖：

```text
数组位置
PCL index
TopoDS iterator 顺序
VTK CellId
```

作为永久身份。

---

## 约束 3：Raw Point Cloud 永远保留

任何算法：

```text
不能 destructive delete 原始 Point
```

只允许：

```text
创建 subset / view / assignment
```

---

## 约束 4：Evidence 与 Feature 分离

```text
Evidence = 我观察到了什么
Feature  = 我认为 CAD 是什么
```

不能混成同一个结构。

---

## 约束 5：局部修改必须是局部算法

用户已经指定：

```text
Face + Hole
```

就只运行 Hole 相关局部检测。

禁止默认：

```text
重新执行完整模型重建
```

除非局部修改导致全局约束失效。

---

## 约束 6：Feature Graph 必须能够独立重建 CAD

任何时候：

```text
删除当前 OCC Shape
```

之后，都必须可以仅依靠：

```text
Feature Graph
```

重新得到同一个 B-Rep。

---

## 约束 7：算法接口不得暴露第三方类型

核心接口使用项目自己的：

```text
Vec3
Mat3
PointView
FitResult
Feature
SurfaceEvidence
```

而不是：

```text
pcl::PointCloud
pcl::ModelCoefficients
TopoDS_Face
```

---

## 约束 8：所有自动推断必须保留 Provenance

每个 Surface / Feature 应记录：

```text
来源
支持点
拟合误差
覆盖率
置信度
生成算法
用户是否确认
```

方便：

```text
调试
重新拟合
解释
冲突处理
```

---

# 28. 最终架构

```text
                        Raw Point Cloud
                              │
                              ▼
                    Point Cloud Processor
                              │
                              ▼
                     Surface Detection
                              │
                              ▼
                       Evidence Graph
                 ┌────────────┴────────────┐
                 │                         │
                 ▼                         ▼
          Relation Analysis          Residual Evidence
                 │                         │
                 └────────────┬────────────┘
                              ▼
                     Feature Recognition
                              │
                              ▼
                Parametric Feature Graph
                         ↑          │
                         │          │
               Local Refit          │
                         ↑          │
               User Correction      │
                         ↑          │
              Natural Language      │
                                    ▼
                         OCC Adapter
                                    │
                                    ▼
                                  B-Rep
                                    │
                                    ▼
                                  STEP
```

---

# 29. 一句话定义整个系统

> **系统不是把点云直接“转换”为 STEP，而是先从点云建立可追踪的几何证据，再推断为可编辑的参数化 Feature Graph；用户和自然语言操作始终修改 Feature Graph 或触发局部证据更新，OpenCASCADE 最终只负责把当前 Feature Graph 构造成精确 B-Rep 并导出 STEP。**

这应当作为整个项目后续实现时最核心的架构原则。
