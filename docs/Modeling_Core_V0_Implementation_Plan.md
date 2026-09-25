# Modeling Core V0 第一版实现计划

## 1. 目标

实现一个**独立于 Qt / VTK / 点云算法**的最小参数化 CAD 建模核心。

第一版只需要证明以下主链完整可用：

```text
Feature Parameters
        ↓
PartDocument
        ↓
Feature History
        ↓
Rebuild
        ↓
OpenCASCADE
        ↓
TopoDS_Shape / B-Rep
        ↓
STEP
```

同时支持：

```text
修改 Feature 参数
        ↓
重新 rebuild
        ↓
得到新的正确几何体
```

第一版不是完整 CAD 内核，不追求复杂拓扑命名、草图约束、任意特征 DAG 或复杂建模能力。

---

## 2. 核心设计原则

### 2.1 Modeling Core 是唯一模型状态拥有者

未来以下入口：

```text
Qt 手工建模
点云自动重建
AI / Natural Language 修改
Local Refit
```

都只负责产生：

```text
ModelCommand
或
ModelPatch
```

最终统一修改：

```text
PartDocument
```

不得分别维护自己的模型状态。

### 2.2 参数状态是真实状态

真实状态必须是：

```text
FeatureParams
SketchEntity
FeatureReference
Feature 顺序
```

例如：

```text
Sketch001
Rectangle(50, 30)

Extrude001
depth = 20
```

而不是：

```text
TopoDS_Shape
TopoDS_Face
vtkActor
vtkPolyData
```

OCCT Shape 是由参数重新计算得到的**派生状态**。

因此必须保证：

```text
FeatureParams
      ↓ rebuild
TopoDS_Shape
```

而不能要求保存某个 Shape 后系统才能继续工作。

### 2.3 Core 禁止依赖 Qt / VTK

Modeling Core 第一版允许：

```text
C++ STL
OpenCASCADE
```

禁止：

```text
Qt
QObject
QString
VTK
vtkActor
vtkPolyData
UI 控件
```

---

## 3. V0 功能范围

第一版只支持：

```text
Sketch
Extrude
Cut
```

完成下面一个完整模型即可：

```text
矩形草图
    ↓
拉伸成长方体
    ↓
在顶部平面创建圆草图
    ↓
向下切除
    ↓
得到带圆孔的长方体
```

并且可以修改参数后重新生成。

---

## 4. 第一版核心数据结构

### 4.1 ID

所有长期引用必须使用稳定 ID。

例如：

```cpp
using FeatureId = uint64_t;
using SketchEntityId = uint64_t;
```

禁止使用：

```cpp
features[3]
```

作为模块之间的长期引用方式。

### 4.2 Sketch Entity

V0 只需要支持：

```text
Line
Rectangle
Circle
```

可以采用类似：

```cpp
enum class SketchEntityType {
    Line,
    Rectangle,
    Circle
};
```

数据使用 STL/POD 保存。

例如：

```cpp
struct Rectangle2D {
    double x;
    double y;
    double width;
    double height;
};

struct Circle2D {
    double x;
    double y;
    double radius;
};
```

V0 不实现：

```text
尺寸约束
几何约束
圆弧
样条
自动求解器
```

---

## 5. Sketch Plane

至少支持两种草图平面。

### A. 基准平面

```text
XY
YZ
XZ
```

### B. 已有 Feature 的平面

例如：

```text
Extrude001 的 EndFace
```

V0 可以定义：

```cpp
enum class FaceRole {
    StartFace,
    EndFace,
    Unknown
};
```

以及：

```cpp
struct FaceReference {
    FeatureId ownerFeature;
    FaceRole role;

    // fallback
    Vec3 normal;
    Vec3 centroid;
    double area;

    int transientFaceIndexHint = -1;
};
```

V0 不要求彻底解决 Topological Naming Problem。

---

## 6. Feature 参数

推荐第一版使用 POD + variant。

例如：

```cpp
struct SketchFeatureParams {
    SketchPlaneReference plane;
    std::vector<SketchEntity> entities;
};

struct ExtrudeFeatureParams {
    FeatureId sketchId;
    double depth;
    bool reverse = false;
};

struct CutFeatureParams {
    FeatureId sketchId;
    double depth;
    bool throughAll = false;
};
```

统一：

```cpp
using FeatureParams = std::variant<
    SketchFeatureParams,
    ExtrudeFeatureParams,
    CutFeatureParams
>;
```

Feature：

```cpp
struct Feature {
    FeatureId id;
    std::string name;

    FeatureType type;
    FeatureParams params;

    bool suppressed = false;
    bool valid = true;
    std::string errorText;
};
```

V0 优先简单数据结构，不建立复杂继承体系。

---

## 7. PartDocument

`PartDocument` 是 Modeling Core 的中心。

建议最小接口：

```cpp
class PartDocument {
public:
    FeatureId addFeature(const FeatureParams&);

    bool editFeature(
        FeatureId id,
        const FeatureParams& params
    );

    bool removeFeature(FeatureId id);

    bool rebuild();

    const std::vector<Feature>& features() const;

    const TopoDS_Shape& bodyShape() const;

private:
    std::vector<Feature> features_;
    TopoDS_Shape bodyShape_;
};
```

V0 使用：

```cpp
std::vector<Feature>
```

保存历史顺序。

暂时不要实现真正的通用 Graph。

Feature 参数中的引用本身已经形成依赖关系：

```text
Sketch001
     ↓
Extrude001
     ↓
Sketch002
     ↓
Cut001
```

---

## 8. Rebuild Engine

第一版 rebuild 采用简单顺序执行。

```text
Feature 0
 ↓
Feature 1
 ↓
Feature 2
 ↓
Feature 3
```

逻辑：

```text
清空当前 derived geometry
        ↓
从 Feature 0 开始
        ↓
解析 Feature 参数
        ↓
调用对应 Builder
        ↓
生成新的 TopoDS_Shape
        ↓
继续下一 Feature
```

如果 Feature 构建失败：

```text
feature.valid = false
feature.errorText = ...
```

同时：

```text
bodyShape 保留上一个成功状态
后续依赖 Feature 不再继续执行
```

第一版优先保证：

> rebuild 行为明确、确定、可测试。

---

## 9. OCCT Builder

OCCT 调用尽量收敛到单独目录。

建议：

```text
modeling/
├── include/
│   └── modeling/
│       ├── Id.h
│       ├── Sketch.h
│       ├── Feature.h
│       ├── FeatureParams.h
│       ├── FaceReference.h
│       ├── PartDocument.h
│       └── ModelCommand.h
│
└── src/
    ├── PartDocument.cpp
    ├── RebuildEngine.cpp
    │
    └── occt/
        ├── SketchBuilder.cpp
        ├── ExtrudeBuilder.cpp
        ├── CutBuilder.cpp
        └── ShapeOps.cpp
```

---

## 10. SketchBuilder

职责：

```text
SketchFeatureParams
        ↓
OCCT Wire / Face
```

V0 必须支持：

### Rectangle

```text
Rectangle
→ 4 Edge
→ Wire
→ Face
```

### Circle

```text
Circle
→ Edge
→ Wire
→ Face
```

暂时不需要复杂轮廓识别。

第一版允许规定：

> 一个 Sketch 只包含一个闭合 Profile。

这会大幅降低复杂度。

---

## 11. ExtrudeBuilder

输入：

```text
Sketch Profile
depth
direction
```

输出：

```text
TopoDS_Shape
```

使用 OCCT：

```text
BRepPrimAPI_MakePrism
```

V0 至少支持：

```text
New Body Extrude
```

如果已有 Body，则可以暂时不支持 Add/Fuse。

---

## 12. CutBuilder

输入：

```text
已有 body
+
Sketch Profile
+
depth
```

生成工具体：

```text
BRepPrimAPI_MakePrism
```

然后：

```text
BRepAlgoAPI_Cut
```

得到新的：

```text
TopoDS_Shape
```

V0 只要求：

```text
Blind Cut
```

`ThroughAll` 可以作为可选增强。

---

## 13. FaceReference V0

这是第一版最容易复杂化的地方。

不要尝试一次解决完整拓扑命名。

对于 Extrude：

```text
StartFace
EndFace
```

优先通过 Feature 语义寻找。

例如：

```text
FaceReference {
    ownerFeature = Extrude001
    role = EndFace
}
```

无法通过语义解析时，再采用：

```text
normal
centroid
area
```

进行简单几何匹配。

V0 可以接受部分复杂修改后：

```text
ReferenceLost
```

但必须明确返回错误，而不是静默引用错误的面。

---

## 14. Command 层

在基础 rebuild 工作正常以后增加。

定义最小：

```text
AddFeatureCommand
EditFeatureCommand
RemoveFeatureCommand
```

例如：

```cpp
struct EditFeatureCommand {
    FeatureId id;
    FeatureParams params;
};
```

统一入口可以是：

```cpp
ModelResult ModelingCore::execute(
    const ModelCommand&
);
```

未来：

```text
Qt
Point Cloud Reconstruction
AI
Local Refit
```

全部通过这个接口修改模型。

---

## 15. V0 不实现内容

以下全部不属于第一版：

```text
Qt UI
VTK 渲染
鼠标拾取
高亮

点云处理
Surface Detection
Feature Recognition
Local Refit

自然语言 Agent

草图约束求解
尺寸约束
圆弧
样条
Fillet
Chamfer
Loft
Sweep
Shell

装配体

完整拓扑命名系统

复杂 Feature DAG

增量 rebuild

多 Body

OCAF

复杂 Undo/Redo

工程文件正式格式
```

如果实现过程中发现必须大量增加上述能力，应该停下来重新检查设计，而不是继续扩大范围。

---

## 16. 实现阶段

### M0 — Modeling 数据模型

完成：

```text
FeatureId
SketchEntity
SketchPlaneReference
FaceReference
FeatureParams
Feature
PartDocument
```

要求：

```text
不调用 Qt
不调用 VTK
可以单独编译
```

#### 验收

能够：

```cpp
PartDocument doc;

auto sketch = doc.addFeature(...);
auto extrude = doc.addFeature(...);
```

并正确保存 Feature History。

此阶段可以暂时没有 Shape。

---

### M1 — Rectangle → Extrude

实现：

```text
Rectangle Sketch
       ↓
SketchBuilder
       ↓
Face
       ↓
ExtrudeBuilder
       ↓
TopoDS_Shape
```

测试模型：

```text
Rectangle:
50 × 30

Extrude:
20
```

预期：

```text
50 × 30 × 20
```

#### 验收

输出：

```text
test_box.step
```

使用任意 STEP Viewer / CAD 软件打开后应为正确长方体。

---

### M2 — 参数修改 + Rebuild

修改：

```text
Extrude depth
20 → 40
```

执行：

```cpp
doc.editFeature(...);
doc.rebuild();
```

重新输出：

```text
test_box_modified.step
```

预期：

```text
50 × 30 × 40
```

#### 验收

不得直接修改之前生成的 `TopoDS_Shape`。

必须：

```text
FeatureParams
→ rebuild
→ 新 Shape
```

完成。

---

### M3 — Face Sketch + Cut

建立：

```text
Rectangle 50 × 30
        ↓
Extrude 20
        ↓
选择 EndFace
        ↓
Circle
radius = 5
        ↓
Cut 10
```

输出：

```text
test_cut.step
```

#### 验收

最终模型必须是：

```text
50 × 30 × 20 长方体
+
顶面圆形盲孔
radius = 5
depth = 10
```

---

### M4 — 上游参数修改后整体重建

把：

```text
Extrude depth
20 → 30
```

执行完整：

```text
rebuild()
```

预期：

```text
长方体高度变成 30
顶部圆孔仍然存在
Cut 深度仍为 10
```

这是 V0 最关键的端到端测试。

---

### M5 — Command API

将：

```text
addFeature
editFeature
removeFeature
```

封装成：

```text
ModelCommand
```

至少验证：

```text
Add
Edit
Remove
```

能够通过统一入口工作。

---

## 17. 最小验收标准

V0 的最低交付要求不是“功能很多”，而是下面 **5 条全部成立**。

### Acceptance 1：独立 Core

能够单独构建：

```text
CadModelCore
```

依赖仅：

```text
STL
OpenCASCADE
```

不存在：

```text
Qt
VTK
```

### Acceptance 2：参数 → 几何

输入：

```text
Rectangle 50 × 30
Extrude 20
```

能够生成有效 B-Rep：

```text
50 × 30 × 20
```

并导出 STEP。

### Acceptance 3：参数修改 → Rebuild

将：

```text
depth = 20
```

修改成：

```text
depth = 40
```

重新 build 后得到：

```text
50 × 30 × 40
```

证明：

```text
FeatureParams
```

而不是 Shape 是核心状态。

### Acceptance 4：下游 Feature

支持：

```text
Extrude
    ↓
EndFace
    ↓
Circle Sketch
    ↓
Cut
```

成功得到带圆孔的实体。

### Acceptance 5：上游变化后下游仍能重建

将：

```text
Base Extrude
20 → 30
```

之后完整 rebuild。

必须仍得到：

```text
高度 30 的实体
+
正确位置的圆孔
```

如果 Reference 无法解析，则必须：

```text
返回明确 ReferenceLost / rebuild failure
```

禁止：

```text
崩溃
引用随机 Face
静默生成错误模型
```

---

## 18. 最终端到端测试

建立一个无 Qt 的测试程序：

```text
modeling_core_smoke_test
```

流程：

```cpp
PartDocument doc;

// 1. XY Plane 创建矩形草图
Sketch1:
    Rectangle(0, 0, 50, 30)

// 2. 拉伸
Extrude1:
    depth = 20

doc.rebuild();

exportStep(
    doc.bodyShape(),
    "01_base.step"
);

// 3. 修改参数
Extrude1.depth = 30;

doc.rebuild();

exportStep(
    doc.bodyShape(),
    "02_modified.step"
);

// 4. 顶面建圆
Sketch2:
    plane = EndFace(Extrude1)
    Circle(center=(25,15), radius=5)

// 5. 切除
Cut1:
    depth = 10

doc.rebuild();

exportStep(
    doc.bodyShape(),
    "03_cut.step"
);
```

必须得到：

```text
01_base.step
50 × 30 × 20

02_modified.step
50 × 30 × 30

03_cut.step
50 × 30 × 30
顶部中央 radius=5、depth=10 圆孔
```

---

## 19. 完成定义

只有以下闭环成立，V0 才算完成：

```text
Feature Parameters
        ↓
PartDocument
        ↓
Rebuild
        ↓
OCCT B-Rep
        ↓
STEP

并且：

修改上游 Feature
        ↓
重新 Rebuild
        ↓
下游 Feature 正确恢复
```

如果这个闭环已经稳定，则停止继续增加 Modeling Core 功能。

下一阶段再分别接入：

```text
Qt Client
      ↓ ModelCommand

Point Cloud Reconstruction
      ↓ ModelPatch

Natural Language
      ↓ ModelPatch
```

V0 的目标是建立稳定接口和状态模型，而不是完成一个完整 CAD 系统。

---

## 20. 一句话最小验收

> **不用 Qt，写一个 C++ 测试程序，通过 Feature 参数生成“长方体 → 修改高度 → 顶面打孔”，每一步都能 rebuild 并导出正确 STEP。**

只要这个成功，Qt 前端、点云参数化重建和后续 AI 修改就都有一个稳定的 Modeling Core 可以接入。
