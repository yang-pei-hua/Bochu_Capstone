# 参数化核心（CadModelCore）扩展需求清单

## 0. 文档目的与边界

客户端「从草图开始的手动建模」功能（绘制点 / 连线 / 预制圆 / 预制长方形，Properties 面板改参数，拉伸 / 切除）
需要在 `core/parametric_modeling`（以下称 **核心**）之上实现。

本文档用于记录客户端与核心之间的能力契约。所有能在 `apps/` 侧完成的交互仍由客户端承担；
必须成为 Feature Graph 真源或涉及几何内核一致性的能力则由核心实现。

- 提出方：`apps/cad_pointcloud_client`
- 目标核心：`core/parametric_modeling`（V0）
- 状态：已完成首轮评估与核心扩展；各项状态见下表和标题

### 0.1 评估结论（2026-09-27）

本轮已实现且有自动化测试覆盖：A1、A2、B、C1、C2、D2、E1、E2、E3、F1、F2、F3、F4。

仍需单独设计后再实现：

- D1 稳定语义面：需要稳定 `FaceId`、拓扑命名和布尔运算后的匹配策略，不能继续扩展
  `StartFace/EndFace` 枚举冒充完整方案。
- D3 面草图与上游特征的依赖边：SolidWorks 风格「点选模型面 → 在该面上开草图」目前把面记录为
  显式 `Plane3d`，因此 `dependentsOf()` 看不到这条依赖，级联删除不完整（见 §2 D3）。
- E4 Feature Graph 序列化：需要先冻结 schema、版本迁移、单位和错误恢复策略。
- E5 Undo/Redo：建议建立在原子命令批次或 E4 快照上，避免客户端与核心各维护一套历史。
- E6 结构化错误：对 AI agent 应从“低优先级”提升到“高优先级”，agent 不应解析英文错误文本。

客户端临时职责已于同日完成迁移：app 侧 Point 类型并入 `modeling::Point2D`；图元增 / 改 / 删改走核心
`ModelCommand`，不再由 app 分配 `SketchEntityId`；线段串接、轮廓过滤与单轮廓限制交还核心；草图创建即
进入 Feature Graph（未闭合时仍可继续编辑）；`startNewModel()` 改用 `PartDocument::clear()`；并移除了
「只能有一个 Extrude」的客户端限制。§4 的职责边界已同步为迁移后的现状。

草图交互随后改为 SolidWorks 风格（在 3D 视口左侧常驻工具条，点选面/基准面后开启草图，工具为
鼠标 / 点 / 线 / 图形下拉），不再有独立 2D 画布窗口。该轮只在 `apps/` 内实现，唯一新增的核心诉求
是 D3：面草图使用显式 `Plane3d` 后，上游依赖边丢失。

---

## 1. 评估前核心基线（历史对照）

| 能力 | 现状 |
| --- | --- |
| 草图图元 | `SketchGeometry = variant<Line2D, Rectangle2D, Circle2D>`，**无点图元** |
| 草图平面 | `DatumPlane{XY, YZ, XZ}`（原点恒为世界原点）或 `FaceReference` |
| 面引用 | `FaceReference{ownerFeature, FaceRole{StartFace, EndFace, Unknown}}`，owner **必须是 Extrude** |
| 拉伸 | `ExtrudeFeatureParams{sketchId, depth, reverse}` |
| 切除 | `CutFeatureParams{sketchId, depth, throughAll}`，方向恒为草图法向反向 |
| 命令 | `AddFeatureCommand` / `EditFeatureCommand` / `RemoveFeatureCommand` |
| 抑制 | `PartDocument::setFeatureSuppressed()` 存在，但**未进入 `ModelCommand`** |
| 重建失败策略 | 任一特征失败即 `return false`，其后所有特征标记为 Skipped |
| 文档 | 无 clear / 无序列化 / 无撤销重做 |
| 结果 | 单一 `bodyShape()`，仅支持一次 New Body 拉伸 |

关键源码位置：
- [Sketch.h](file:///d:/code/ECE4500J/core/parametric_modeling/include/modeling/Sketch.h)
- [FeatureParams.h](file:///d:/code/ECE4500J/core/parametric_modeling/include/modeling/FeatureParams.h)
- [FaceReference.h](file:///d:/code/ECE4500J/core/parametric_modeling/include/modeling/FaceReference.h)
- [ModelCommand.h](file:///d:/code/ECE4500J/core/parametric_modeling/include/modeling/ModelCommand.h)
- [PartDocument.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/PartDocument.cpp)
- [RebuildEngine.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp)
- [SketchBuilder.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/occt/SketchBuilder.cpp)
- [ExtrudeBuilder.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/occt/ExtrudeBuilder.cpp)
- [CutBuilder.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/occt/CutBuilder.cpp)

---

## 2. 扩展需求清单

### A. 草图图元类型

#### A1. 新增 `Point2D` 草图图元 —— 状态：已实现

- **涉及文件**：`include/modeling/Sketch.h`
- **需要新增**：

```cpp
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

enum class SketchEntityType { Point, Line, Rectangle, Circle };

using SketchGeometry = std::variant<Point2D, Line2D, Rectangle2D, Circle2D>;
```

- **需要什么功能**：让「点」成为 Feature Graph 里可持久化的**一等草图图元**，
  `sketchEntityType()` 增加 Point 分支。
- **为什么需要**：用户明确要求「草图支持绘制点」。当前核心没有点图元，客户端只能把点
  存在 app 侧并行结构里，违背 `pointcloud_to_parametric_cad_architecture.md` §11.3
  「Feature Graph 才是模型真源」的原则（模型状态会散落在前端）。

#### A2. 草图轮廓提取需忽略非轮廓图元 —— 状态：已实现

- **涉及文件**：`src/occt/SketchBuilder.cpp`（`buildSketchFace` / `buildLines`）
- **需要什么功能**：构建 profile wire 时**过滤掉 Point 图元**（保留其在 `states` 中的存在），
  而不是直接返回 `"A line profile may contain only Line entities"`。
- **为什么需要**：与 A1 配套。草图里同时存在点与线是正常状态，点属于参考几何，不参与面轮廓。

---

### B. 草图图元级编辑命令 —— 状态：已实现

- **涉及文件**：`include/modeling/ModelCommand.h`、`include/modeling/FeatureParams.h`、
  `src/ModelingCore.cpp`、`src/PartDocument.cpp`
- **需要新增**：

```cpp
struct AddSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntity entity;
};
struct EditSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntityId entityId = kInvalidSketchEntityId;
    SketchGeometry geometry;
};
struct RemoveSketchEntityCommand {
    FeatureId sketchId = kInvalidFeatureId;
    SketchEntityId entityId = kInvalidSketchEntityId;
};
```

- **需要什么功能**：对已存在的 Sketch 特征做**单图元粒度**的增 / 改 / 删，
  由核心负责 `SketchEntityId` 的分配与唯一性校验。
- **为什么需要**：手工建模每一步都是一次单实体操作（放一个点、改一个圆的半径、删一条线）。
- **当前可绕过方案**：客户端用 `EditFeatureCommand` 整体替换 `SketchFeatureParams`，
  由 app 自行分配并维护 entityId。若核心不提供 B，客户端将长期使用该绕过方案。
- **附带诉求**：若提供 B，请让 `PartDocument::addFeature` 之外也能校验
  「同一草图内 entityId 不重复」，当前核心对此无任何检查。

---

### C. 拉伸 / 切除语义

#### C1. 拉伸的布尔运算与多实体 —— 状态：已实现

- **涉及文件**：`include/modeling/FeatureParams.h`、`src/RebuildEngine.cpp`、
  `src/occt/ExtrudeBuilder.cpp`
- **现状**：[RebuildEngine.cpp L111-L114](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp#L111-L114)
  中 `if (!currentBody.IsNull()) error = "V0 supports only one New Body Extrude";`
  即一个文档**只能拉伸出一个实体**。
- **需要新增**：

```cpp
enum class ExtrudeOperation { NewBody, Join, Cut, Intersect };

struct ExtrudeFeatureParams {
    FeatureId sketchId = kInvalidFeatureId;
    double depth = 0.0;
    bool reverse = false;
    ExtrudeOperation operation = ExtrudeOperation::NewBody;  // 新增
};
```

- **需要什么功能**：在已有 body 上做 Join / Cut / Intersect，以及支持 NewBody 产生第二实体。
- **为什么需要**：手动建模最常见的操作是「在已有零件上再拉一个凸台 / 再拉一个第二实体」；
  当前限制会让用户在第二次拉伸时直接收到错误。

#### C2. 切除方向控制 —— 状态：已实现

- **涉及文件**：`include/modeling/FeatureParams.h`、`src/occt/CutBuilder.cpp`
- **现状**：[CutBuilder.cpp L35-L37](file:///d:/code/ECE4500J/core/parametric_modeling/src/occt/CutBuilder.cpp#L35-L37)
  恒用 `profileFrame.normal` 反向，`CutFeatureParams` 没有 `reverse`（`ExtrudeFeatureParams` 有）。
- **需要新增**：`CutFeatureParams::reverse`（或统一为共享的 `reverse` 字段）。
- **为什么需要**：切除与拉伸的方向控制应当对称，否则用户无法在草图平面另一侧切除。

#### C3. 双向 / 对称拉伸 —— 优先级：低

- **需要新增**：`ExtrudeFeatureParams::secondDepth` 或 `symmetric`，`BRepPrimAPI_MakePrism`
  改为按两侧深度分别成形的对称拉伸。
- **为什么需要**：以草图平面为中性面建模是常见习惯。

#### C4. 切除终止条件扩展（到面 / 到下一个） —— 优先级：低

- **现状**：只有 `depth` 与 `throughAll`。
- **需要新增**：`upToFace` 终止条件（依赖 D1 的语义面引用）。

---

### D. 草图平面与面引用

#### D1. `FaceReference` 需要语义面（Semantic Face）体系 —— 状态：待专项设计

- **涉及文件**：`include/modeling/FaceReference.h`、`src/RebuildEngine.cpp::resolvePlane`、
  `src/occt/*`
- **现状**：
  - [RebuildEngine.cpp L51-L61](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp#L51-L61)
    只解析 `StartFace` / `EndFace`，`Unknown` 直接 `ReferenceLost`；
  - owner 必须是 Extrude，Cut 产生的面、Cut 之后仍存在的面都无法引用。
- **需要什么功能**：实现架构文档 §12 的稳定 `FaceId` 语义面（如 Box 的 `+X/-X/+Y/-Y/+Z/-Z`），
  使草图可附着在**任意 body 面**上，且该引用在后续特征修改后仍然有效。
- **为什么需要**：手动建模「在零件侧面/端面画草图再切除」是核心流程；
  目前只有 Extrude 的 Start/End 两个面可用，功能严重受限。

#### D2. 偏移基准面 —— 状态：已实现

- **涉及文件**：`include/modeling/FaceReference.h`、`src/occt/SketchBuilder.cpp::datumPlaneFrame`
- **现状**：`DatumPlane` 仅 XY/YZ/XZ，原点恒为世界原点，无法在 z=10 这类位置建草图。
- **需要新增**：

```cpp
struct OffsetDatumPlane {
    DatumPlane base = DatumPlane::XY;
    double offset = 0.0;
};
// SketchPlaneReference 变为 variant<DatumPlaneReference, OffsetDatumPlane, FaceReference>
```

- **为什么需要**：手工建模需要在特定高度/位置起草图，而不是只在三个全局原点上。

#### D3. 面草图使用显式 `Plane3d` 后，与上游特征的依赖边丢失 —— 状态：待专项设计（依赖 D1）

- **涉及文件**：`include/modeling/FaceReference.h`、`src/PartDocument.cpp::dependsOn`、
  `src/RebuildEngine.cpp::resolvePlane`、`apps/cad_pointcloud_client/src/app/MainWindow.cpp`
- **现状**：客户端在实体面上开草图时（SolidWorks 风格：点选模型面 → New Sketch），把选中的面记为
  一个显式的世界空间 `Plane3d{origin, normal, xDirection}`，而不是
  `FaceReference{ownerFeature, FaceRole}`。好处是面可以是**任意平面**，不再受 Extrude 的
  Start/End 两个面限制；代价是 `PartDocument::dependsOn()` 无法再看出「这张草图依赖哪个特征」。
  实测（客户端 `sketch-modeling-check.ps1` §15）：

  | 文档 | 删除 `Sketch001` 后的结果 |
  | --- | --- |
  | Sketch001(XY) → Extrude001(NewBody) → Sketch002(面草图/Plane3d) → Extrude002(Join) + Cut001 | 只级联删除 `Sketch001`+`Extrude001`，`Sketch002`/`Extrude002`/`Cut001` 保留，重建报 `Extrude002: Extrude boolean operation requires an existing body` |

- **需要什么功能**：让「草图平面取自某特征的某个面」成为可查询的依赖边。即 D1 的语义面：
  先有稳定 `FaceId`，再把 `FaceId`（而非裸几何平面）作为草图平面引用，使 `dependentsOf()`
  与级联删除覆盖这条边。在 D1 落地前，核心至少需要能表达「本特征依赖 N 号特征所生成的那个面」。
- **为什么需要**：点选模型面直接开草图是手动建模的主路径；若这条依赖边缺失，删除上游特征后文档会
  进入无法重建的状态，且删除前的「将一并删除」提示不完整。
- **客户端当前行为（可接受，但属于绕过方案）**：客户端第 15 节的验证改为删除面草图本身——它的
  `Extrude002`/`Cut001` 消费者仍能被正确识别并级联删除，因此不依赖 D1 的那部分依赖语义已有测试覆盖。

---

### E. 文档与命令层

#### E1. 抑制命令未暴露到 `ModelCommand` —— 状态：已实现

- **涉及文件**：`include/modeling/ModelCommand.h`、`src/ModelingCore.cpp`
- **现状**：`PartDocument::setFeatureSuppressed()` 已实现，但 `ModelCommand` 只有
  Add / Edit / Remove，`ModelingCore::execute` 无法触达抑制功能。
- **需要新增**：`SetFeatureSuppressedCommand{FeatureId id, bool suppressed}` 并接入 `execute`。
- **为什么需要**：UI 需要「抑制 / 恢复特征」按钮，并且要让 `Feature::suppressed` 成为
  可通过命令层修改的、可撤销的状态。

#### E2. 依赖关系查询与级联删除 —— 状态：已实现

- **涉及文件**：`include/modeling/PartDocument.h`
- **现状**：[PartDocument::removeFeature](file:///d:/code/ECE4500J/core/parametric_modeling/src/PartDocument.cpp#L74-L86)
  直接 erase，下游 Extrude/Cut 会在下次 rebuild 时报
  `"Missing or invalid upstream sketch N"`，整个文档重建失败。
- **需要新增**：
  - `std::vector<FeatureId> dependentsOf(FeatureId id) const;`（或
  - `RemoveFeatureCommand` 增加 `bool cascade = false;`）
- **为什么需要**：删除草图时 UI 需要提示「有 N 个下游特征将一并删除」并执行级联删除，
  而不是让文档进入无法重建的状态。

#### E3. 文档新建 / 清空 —— 状态：已实现

- **需要新增**：`PartDocument::clear()`（含 `nextFeatureId_` 复位策略的明确约定）。
- **为什么需要**：「新建草图 / 新建模型」。
- **当前可绕过方案**：客户端 `new modeling::ModelingCore` 整体替换（见
  [ModelingController.cpp L51](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/ModelingController.cpp#L51)），
  代价是 app 侧被迫持有 document 的所有权语义。

#### E4. Feature Graph 持久化（序列化 / 反序列化） —— 状态：待 schema 设计

- **现状**：核心**没有任何** serialize / deserialize 接口，Feature Graph 无法存盘。
  架构文档 §11.3 要求「只要 Feature Graph 没丢，B-Rep / STEP / 显示数据都可以重新生成」，
  但当前无法把 Feature Graph 落到磁盘。
- **需要新增**：`PartDocument` 的 `toJson() / fromJson()`，或独立的
  `FeatureGraphSerializer`，需覆盖：
  - `SketchFeatureParams`（含 `SketchPlaneReference` 的全部 variant 分支）
  - `ExtrudeFeatureParams`、`CutFeatureParams`
  - `Feature{id, name, type, suppressed}`
- **为什么需要**：File > Save / Open 工程、撤销重做快照、跨会话恢复模型。

#### E5. 撤销 / 重做 —— 状态：待命令事务设计

- **需要新增**：命令栈（`UndoStack`）或 `ModelingCore::undo() / redo()`。
- **为什么需要**：手工建模需要 Undo/Redo 按钮。
- **当前可绕过方案**：客户端基于 E4 的快照自行实现，但需要 E4 先落地。

#### E6. 结构化错误码 —— 状态：待实现，AI agent 高优先级

- **现状**：`ModelResult::error` 与 `Feature::errorText` 是自由文本，
  仅靠 `"ReferenceLost: ..."` 这样的约定式前缀区分类型。
- **需要新增**：

```cpp
enum class ModelErrorCode {
    None, ReferenceLost, InvalidProfile, BooleanFailed, InvalidParameter, NotFound
};
```

- **为什么需要**：UI 需要按错误类型做差异化提示与定位，而不是解析英文文案。

---

### F. 轮廓（Profile）构建能力

#### F1. 多环轮廓（带内环的草图） —— 状态：已实现

- **涉及文件**：`src/occt/SketchBuilder.cpp`
- **现状**：只支持「一个 Rectangle」「一个 Circle」「一条连通闭合的 Line 链」三种二选一；
  「矩形 + 内圆」这类**含孔草图**必然失败。
- **需要什么功能**：面轮廓支持**外环 + 若干内环**（holes），
  即 `BRepBuilderAPI_MakeFace` 接收多个 wire 并生成带孔的 face。
- **为什么需要**：用户在一个草图里同时画矩形和圆孔是最自然的画法，
  当前只能拆成「基准草图 → 拉伸 → 端面草图 → 切除」两步，且端面引用受 D1 限制。

#### F2. Line 链自动串接与闭合校验 —— 状态：已实现

- **现状**：[buildLines](file:///d:/code/ECE4500J/core/parametric_modeling/src/occt/SketchBuilder.cpp#L92-L128)
  按传入顺序逐条 `Add`，顺序不对即报
  `"Line entities do not form a connected wire"`。
- **需要什么功能**：
  1. 按端点自动串接无序的边集合；
  2. 提供「当前草图是否构成闭合轮廓」的**查询 API**，供 UI 在提交前提示用户。
- **为什么需要**：手工点击画线天然是乱序的；把排序责任推给前端会让前端重复实现几何逻辑。

#### F3. 未完成草图的容错 —— 状态：已实现

- **现状**：[RebuildEngine.cpp L163-L177](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp#L163-L177)
  任一特征失败即 `return false`，并把其后的所有特征标记为
  `"Skipped because rebuild stopped after ..."`，同时 `bodyShape_` 回退到上一次成功值。
- **需要什么功能**：允许 Sketch 特征处于**开放 / 未完成**状态（仅点、未闭合的线链）而不阻断
  整个文档重建——例如「只有被 Extrude/Cut 消费时才判定失败」，或引入
  `Feature::state = {Valid, UnderDefined/Open, Error}`。
- **为什么需要**：用户从空白草图开始画到一半时，其他已完成的特征不应该变成 Skipped，
  且不应把 `lastError` 变成阻塞性错误。这是手工建模流程能否「通顺」的关键。

#### F4. 轮廓合法性预检 API —— 状态：已实现

- **需要新增**：

```cpp
bool validateSketch(const SketchFeatureParams& params, std::string& error);
```

- **为什么需要**：让「拉伸 / 切除」按钮能在提交前判断当前草图能否成形，
  并给出「轮廓未闭合」这类可读提示，而不是等 rebuild 失败后回退整个 body。

---

### G. 其他

| 编号 | 需求 | 优先级 | 说明 |
| --- | --- | --- | --- |
| G1 | `shapeOf(FeatureId)` 或 `TopoDS_Compound` 多实体结果 | 低 | 配合 C1，当前 `bodyShape()` 只能返回单一实体 |
| G2 | `Rectangle2D` 增加 `angle`（旋转矩形） | 低 | 当前为轴对齐 `x, y, width, height` |
| G3 | `Circle2D` 支持圆弧 / 槽 | 低 | 当前仅整圆，用户需求未涉及 |
| G4 | 草图约束（重合 / 水平 / 竖直 / 尺寸驱动） | 低 | V0 完全没有约束体系，属独立课题 |

---

## 3. AI Agent 场景补充需求

原清单主要从手动 UI 建模出发。客户端 AI agent 还需要以下能力，不能只依赖现有自由文本命令：

| 编号 | 需求 | 优先级 | 目的 |
| --- | --- | --- | --- |
| H1 | 原子命令批次与 dry-run | 高 | 一组 agent 操作要么全部成功、要么不改变文档；执行前可预检 |
| H2 | 结构化 `ModelErrorCode` + 关联 feature/entity ID | 高 | agent 可恢复、重试和向 UI 精确定位错误 |
| H3 | 文档 revision / expectedRevision | 中 | 防止 agent 基于过期 Feature Graph 覆盖用户刚完成的修改 |
| H4 | 带 schemaVersion 的序列化与命令审计日志 | 高 | 工程保存、会话恢复、Undo/Redo 和 agent 可追溯性 |
| H5 | 语义查询 API | 高 | 按类型、名称、依赖和稳定面 ID 选择目标，不依赖数组下标或瞬时拓扑 |
| H6 | 显式单位元数据 | 中 | agent 生成尺寸命令时避免毫米/米混淆 |
| H7 | `RenameFeatureCommand` | 中 | 允许用户和 agent 使用稳定、可读的语义名称协作 |

建议下一阶段先实现 H1、H2、H3，并与 E4/E5/D1 统一设计；否则先做序列化或撤销很容易形成
第二套互不兼容的命令历史。

---

## 4. 明确「不需要改核心」的部分（由客户端承担）

以下内容不构成对核心的扩展诉求，将在 `apps/cad_pointcloud_client` 内实现：

- 草图会话态（当前工具、选中图元、两点工具的草稿锚点）
- SolidWorks 风格的在视口内草图交互：3D 视图左侧常驻工具条（模型页 / 草图页）、鼠标拾取、
  点 / 线 / 图形下拉工具、退出草图隐藏工具
- 模型面拾取与草图平面推导：`BodyTessellation` 为每个三角面记录面索引与精确 `gp_Pln`
  （`origin / normal / xDirection`），拾取到平面面时在客户端组装成 `Plane3d`
- 草图图元拾取：在**草图坐标系内**比较点击点与图元的几何距离（点 / 线段 / 矩形四边 / 圆），
  不使用 VTK 几何拾取器——细线在拾取器中无法被命中，且单元格 ↔ 图元映射依赖 VTK 的
  单元编号约定，脆弱
- 选中图元后在 Properties 面板中编辑参数
- 草图在 3D 视口中的可视化（独立的 VTK actor，只消费草图数据）
- 通过核心 `ModelCommand`（`Add/Edit/RemoveFeature` 与 `Add/Edit/RemoveSketchEntity`）驱动 Feature Graph；
  图元级编辑使用核心实体命令，不再整体替换 `SketchFeatureParams`
- 「新建草图」与「新建模型」分别通过 `AddFeatureCommand` 与 `PartDocument::clear()` 实现文档重置
- 拉伸 / 切除按钮到 `ExtrudeFeatureParams` / `CutFeatureParams` 的映射，并在拉伸 / 切除前调用核心
  `validateSketch()`，把核心返回的错误文本显示在 UI 中
- 删除特征前用 `PartDocument::dependentsOf()` 提示引用者，确认后发送 cascade 删除命令
- 已提交草图（已被 Extrude / Cut 消费）的平面**不可再改**：用户重新选择平面时，客户端改为
  在该平面上开启一张新草图，而不是改动已入图的草图

---

## 5. 汇总表

| 编号 | 需求 | 类别 | 优先级 |
| --- | --- | --- | --- |
| A1 | 新增 `Point2D` 草图图元 | 类型 | 高 |
| A2 | 轮廓提取忽略点图元 | 行为 | 高 |
| B | 草图图元级增 / 改 / 删命令 | 接口 | 中 |
| C1 | 拉伸布尔运算与多实体 | 接口 + 行为 | 高 |
| C2 | 切除方向控制 | 接口 | 中 |
| C3 | 双向 / 对称拉伸 | 接口 | 低 |
| C4 | 切除到面 / 到下一个 | 接口 | 低 |
| D1 | 语义面（FaceId）体系 | 行为 | 高 |
| D2 | 偏移基准面 | 接口 | 中 |
| D3 | 面草图（`Plane3d`）与上游特征的依赖边 | 行为 | 高 |
| E1 | 抑制命令进入 `ModelCommand` | 接口 | 中 |
| E2 | 依赖查询 / 级联删除 | 接口 | 中 |
| E3 | 文档 clear() | 接口 | 低 |
| E4 | Feature Graph 序列化 | 接口 | 高 |
| E5 | 撤销 / 重做 | 接口 | 中 |
| E6 | 结构化错误码 | 接口 | 低 |
| F1 | 多环轮廓（带孔草图） | 行为 | 高 |
| F2 | Line 链自动串接 + 闭合查询 | 行为 | 中 |
| F3 | 未完成草图容错 | 行为 | 高 |
| F4 | 轮廓合法性预检 API | 接口 | 中 |
| G1-G4 | 多实体结果 / 旋转矩形 / 圆弧 / 约束 | 接口 | 低 |

首轮客户端闭环所需能力已完成；剩余主阻塞项是 D1（稳定语义面）、D3（面草图的依赖边）、E4（版本化持久化）以及
AI agent 补充项 H1/H2/H3。C3、C4、G1-G4 可继续按产品需求延后。
