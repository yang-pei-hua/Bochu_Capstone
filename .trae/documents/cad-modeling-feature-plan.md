# CAD Modeling Core 前端接入与模型渲染计划

## 一、目标

在 `apps/cad_pointcloud_client` 中接入已经实现的 `CadModelCore`，通过现有 Command API 生成、修改和重建参数化模型，并使用 VTK 显示 `PartDocument::bodyShape()`。

本计划只实现下面这条链路：

```text
Qt 参数控件 / Generate 按钮
        ↓
modeling::ModelCommand
        ↓
modeling::ModelingCore::execute()
        ↓
modeling::PartDocument::rebuild()
        ↓
PartDocument::bodyShape()       只读
        ↓
前端 OCCT Shape → vtkPolyData 适配器
        ↓
VTK Actor
        ↓
VTKViewer 显示
```

第一版要在前端完整演示：

1. 生成 `50 × 30 × 30` 的长方体。
2. 在 Extrude 的 `EndFace` 上创建半径为 `5` 的圆草图。
3. 向实体内部切除 `10`，形成盲孔。
4. 修改拉伸高度后调用 Core 重建。
5. VTK 视图立即显示重建后的模型，圆孔仍位于新的顶面。

---

## 二、强制边界

### 2.1 不修改 Modeling Core

本阶段不得修改：

```text
core/parametric_modeling/include/
core/parametric_modeling/src/
core/parametric_modeling/tests/
```

如果前端需求无法通过现有公开 API 完成，应在计划或 issue 中记录缺口，不得把前端逻辑塞入 Core，也不得绕过 API 修改 Core 私有状态。

### 2.2 前端不重复实现建模逻辑

客户端禁止重新定义或实现以下内容：

- `FeatureId`、`SketchEntityId`
- `SketchFeatureParams`、`ExtrudeFeatureParams`、`CutFeatureParams`
- `Feature`、`PartDocument`、`ModelingCore`
- Sketch Profile 构建
- Extrude、Cut 或其他布尔运算
- Feature History 与 rebuild
- `StartFace` / `EndFace` 语义引用
- ReferenceLost 判断
- STEP 导出逻辑

这些能力全部来自 `CadModelCore`。

### 2.3 状态所有权

唯一模型状态仍由：

```cpp
modeling::ModelingCore
```

持有。

前端只保存：

- 当前模型的 `FeatureId`，用于后续发送 Edit/Remove Command。
- Qt 控件值。
- 从当前 `bodyShape()` 生成的临时 `vtkPolyData` 和 actor。

前端不得再维护一套 Feature 列表或可继续建模的 `TopoDS_Shape` 副本。VTK 数据是可随时重新生成的显示缓存，不是真实模型状态。

### 2.4 允许的 OCCT 使用范围

前端渲染适配器可以只读访问：

```cpp
const TopoDS_Shape& PartDocument::bodyShape() const;
```

并使用 OCCT 的三角化/遍历 API 将 Shape 转换为 `vtkPolyData`。

适配器中禁止使用 `BRepPrimAPI_*`、`BRepAlgoAPI_*` 或修改 Shape 的 API；所有建模运算必须留在 Core。

---

## 三、现有接口

前端直接使用以下公开头文件：

```cpp
#include <modeling/ModelCommand.h>
#include <modeling/ModelingCore.h>
#include <modeling/PartDocument.h>
```

模型修改统一走：

```cpp
modeling::ModelResult result = core.execute(command);
```

当前支持的命令：

```cpp
modeling::AddFeatureCommand
modeling::EditFeatureCommand
modeling::RemoveFeatureCommand
```

命令只修改参数化历史。提交命令后，前端显式调用：

```cpp
if (!core.document().rebuild()) {
    showError(QString::fromStdString(core.document().lastError()));
}
```

显示层只读取：

```cpp
core.document().features();
core.document().bodyShape();
core.document().lastError();
```

禁止直接调用 `PartDocument::addFeature/editFeature/removeFeature`；前端所有模型写操作必须通过 `ModelingCore::execute()`。

---

## 四、前端架构

```text
MainWindow
 ├─ ModelingPanel                 参数输入和 Generate/Apply 按钮
 ├─ ModelingController            Command 编排与错误处理
 │   └─ modeling::ModelingCore    唯一模型状态
 └─ VTKViewer
     └─ BodyActor
         └─ OcctShapeTessellator  只读 Shape → vtkPolyData
```

职责边界：

| 组件 | 职责 | 禁止承担 |
|---|---|---|
| `ModelingPanel` | 采集宽、高、深度、孔半径等 UI 参数 | 保存 Feature/Shape、调用 OCCT 建模 |
| `ModelingController` | 构造 Command、保存返回的 FeatureId、调用 rebuild | 自己实现 Feature History 或几何算法 |
| `OcctShapeTessellator` | 将当前 Shape 转为显示网格 | 修改 Shape、执行拉伸/布尔 |
| `BodyActor` | 管理 mapper、surface actor、edge actor 和显示属性 | 成为模型状态源 |
| `VTKViewer` | 添加/更新 actor、相机适屏、Render | 理解 Feature 参数或执行 Command |

---

## 五、目录和文件调整

```text
apps/cad_pointcloud_client/
  CMakeLists.txt                              [改] 链接 CadModelCore
  README.md                                   [改] Core/OCCT 配置和运行说明
  src/
    MainWindow.{h,cpp}                        [改] 装配面板、Controller、Viewer
    app/
      ModelingController.{h,cpp}              [新] 唯一的前端建模编排层
      DemoModelIds.h                          [新] 保存四个 FeatureId
    rendering/
      OcctShapeTessellator.{h,cpp}            [新] 只读 Shape → vtkPolyData
      BodyActor.{h,cpp}                       [新] VTK actor/mapper 管理
    widgets/
      ModelingPanel.{h,cpp}                   [新] 最小参数面板
      VTKViewer.{h,cpp}                       [改] 去掉 demo cube，显示 BodyActor
```

本阶段不新增客户端 `modeling/` 目录，避免与 `core/parametric_modeling` 混淆，也不新增客户端版本的 Feature/PartDocument/ShapeOps。

现有 `Scene`、`ScenePanel`、`PropertyPanel` 可以先保留。`VTKViewer` 的下列槽函数签名保持不变，并改为作用于 `BodyActor`；模型为空时安全 no-op：

```text
setObjectTransform
setObjectVisible
setObjectRepresentation
setObjectOpacity
setObjectColor
setLightingEnabled
```

---

## 六、CMake 接入

客户端不再下载或维护第二套 OCCT SDK。它通过子目录链接 Core，并沿用 Core 的 OpenCASCADE 配置。

在客户端 `CMakeLists.txt` 中增加：

```cmake
set(CAD_MODEL_CORE_BUILD_TESTS OFF CACHE BOOL "" FORCE)

add_subdirectory(
    "${CMAKE_CURRENT_SOURCE_DIR}/../../core/parametric_modeling"
    "${CMAKE_CURRENT_BINARY_DIR}/cad_model_core"
)

target_link_libraries(CadPointCloudClient PRIVATE
    CadModelCore::CadModelCore
)
```

同时把新建的 Controller、Panel 和 rendering 文件加入 `qt_add_executable()`。

约束：

- 不复制 Core 源文件到客户端目标。
- 不在客户端重新列举 `TKernel/TKBRep/TKBO/...`；传递依赖由 `CadModelCore` target 提供。
- 配置时通过 `OpenCASCADE_DIR` 或 `CMAKE_PREFIX_PATH` 指向一套可用的 OpenCASCADE。
- 仓库统一依赖根目录为 `deps/`，MSVC OCCT 位于 `deps/occt-8.0.1/`。
- VTK 位于 `deps/vtk-9.7.0/`，COLMAP runtime 位于 `deps/colmap/`；三个二进制目录均由根 `.gitignore` 忽略。
- Qt、VTK、CadModelCore 和 OCCT 统一使用 MSVC 2022 x64 Release；不得链接 MinGW `.dll.a`。
- 保留现有本地 VTK SDK 探测逻辑，不读取或修改 `deps/vtk-9.7.0/` 内部文件。
- Qt/VTK 只能出现在客户端 target，不能反向传入 `CadModelCore`。

---

## 七、ModelingController

`ModelingController` 是 QObject 编排层，但它不实现任何几何算法。

建议接口：

```cpp
struct DemoModelParameters {
    double width = 50.0;
    double height = 30.0;
    double extrusionDepth = 30.0;
    double holeCenterX = 25.0;
    double holeCenterY = 15.0;
    double holeRadius = 5.0;
    double cutDepth = 10.0;
};

class ModelingController final : public QObject {
    Q_OBJECT
public:
    explicit ModelingController(QObject* parent = nullptr);

    bool createDemoModel(const DemoModelParameters& parameters);
    bool updateDemoModel(const DemoModelParameters& parameters);

    const TopoDS_Shape& bodyShape() const;
    const std::vector<modeling::Feature>& features() const;

signals:
    void modelRebuilt();
    void modelError(const QString& message);

private:
    bool execute(const modeling::ModelCommand& command,
                 modeling::FeatureId* createdId = nullptr);
    bool rebuildAndPublish();

    std::unique_ptr<modeling::ModelingCore> m_core;
    DemoModelIds m_ids;
};
```

### 7.1 新建演示模型

每次选择“新建/重新生成”时，Controller 新建一个空的 `ModelingCore` 实例，然后依次发送四个 Add Command：

```cpp
// 1. XY 矩形草图
SketchFeatureParams baseSketch;
baseSketch.plane = datumPlane(DatumPlane::XY);
baseSketch.entities = {
    {1, Rectangle2D{0.0, 0.0, p.width, p.height}}
};
baseSketchId = add(baseSketch);

// 2. 拉伸
extrudeId = add(ExtrudeFeatureParams{
    baseSketchId, p.extrusionDepth, false
});

// 3. Extrude EndFace 上的圆草图
SketchFeatureParams holeSketch;
holeSketch.plane = featureFace(extrudeId, FaceRole::EndFace);
holeSketch.entities = {
    {2, Circle2D{p.holeCenterX, p.holeCenterY, p.holeRadius}}
};
holeSketchId = add(holeSketch);

// 4. 盲孔切除
cutId = add(CutFeatureParams{
    holeSketchId, p.cutDepth, false
});

rebuildAndPublish();
```

伪代码中的 `add()` 必须封装为 `m_core->execute(AddFeatureCommand{...})`，并检查 `ModelResult::success`。

### 7.2 参数修改

修改已有模型时不重新构造 Shape，也不直接改 Feature 数组，而是对已有稳定 ID 发送 Edit Command：

```cpp
execute(EditFeatureCommand{
    m_ids.extrude,
    ExtrudeFeatureParams{m_ids.baseSketch, p.extrusionDepth, false}
});

execute(EditFeatureCommand{
    m_ids.cut,
    CutFeatureParams{m_ids.holeSketch, p.cutDepth, false}
});

rebuildAndPublish();
```

宽高、孔中心或半径变化时，同样重新构造对应的 `SketchFeatureParams`，通过 Edit Command 替换参数。

所有 Edit Command 成功后再调用一次 `rebuild()`。若任一命令失败，停止本次提交并显示 `ModelResult::error`。

---

## 八、Shape 到 VTK 的只读适配

`OcctShapeTessellator` 接受 `const TopoDS_Shape&`，返回新的 `vtkPolyData`：

```cpp
vtkSmartPointer<vtkPolyData> tessellate(
    const TopoDS_Shape& shape,
    double linearDeflection = 0.1,
    double angularDeflection = 0.5);
```

处理流程：

1. Shape 为空时返回空 `vtkPolyData`。
2. 调用 `BRepMesh_IncrementalMesh` 生成只读三角化缓存。
3. 使用 `TopExp_Explorer(shape, TopAbs_FACE)` 遍历面。
4. 使用 `BRep_Tool::Triangulation(face, location)` 获取三角形。
5. 对节点应用 `TopLoc_Location` 变换后写入 `vtkPoints`。
6. 面方向为 `TopAbs_REVERSED` 时交换三角形第二、第三顶点。
7. 写入 `vtkCellArray` 后构造 `vtkPolyData`。
8. 通过 `vtkPolyDataNormals` 生成显示法向，开启 `SplittingOn()`。

这一层只负责显示转换，不保存 OCCT Feature 信息，不向 Core 回写任何内容。

第一版不做面拾取，因此不需要 `FaceIds` 数组、`TopoDS_Face` 映射表或拓扑匹配。

### 8.1 BodyActor

`BodyActor` 持有：

```text
vtkPolyDataMapper
vtkActor             实体表面
vtkFeatureEdges      可选的轮廓/锐边
vtkPolyDataMapper
vtkActor             黑色特征边
```

每次 Core rebuild 成功后：

```cpp
bodyActor->setShape(controller.bodyShape());
viewer->resetCamera();       // 仅首次生成或用户明确要求适屏时
viewer->renderNow();
```

参数连续修改时只更新 mapper input，不自动重置相机，避免视角跳动。

---

## 九、最小 UI

`ModelingPanel` 第一版只提供演示模型需要的参数：

| 参数 | 默认值 |
|---|---:|
| Rectangle Width | 50 |
| Rectangle Height | 30 |
| Extrude Depth | 30 |
| Hole Center X | 25 |
| Hole Center Y | 15 |
| Hole Radius | 5 |
| Cut Depth | 10 |

按钮：

- `Generate Model`：新建 Core 文档并发送四个 Add Command。
- `Apply Parameters`：对稳定 FeatureId 发送 Edit Command，然后 rebuild。
- `Fit View`：只操作 VTK 相机。
- `Export STEP`：可选，直接调用 Core 已有 `exportStep()`；不在前端重新实现导出器。

状态显示：

- 成功：`Rebuild succeeded | 4 features`。
- 失败：显示 `document.lastError()`，保留并渲染 Core 返回的最后成功 Shape。
- Feature 列表可以只读展示 `id/name/type/valid/errorText`，不得复制为另一份可编辑模型数据。

本阶段不实现交互式画草图、面拾取、PropertyManager、快捷栏、确认角或 SolidWorks 完整键鼠映射。

---

## 十、事件流程

### 10.1 Generate Model

```text
用户点击 Generate Model
  → ModelingPanel 读取参数
  → ModelingController 创建新的 ModelingCore
  → execute(AddFeatureCommand) × 4
  → document.rebuild()
  → emit modelRebuilt()
  → MainWindow 读取 bodyShape()
  → OcctShapeTessellator 生成 vtkPolyData
  → BodyActor 更新 mapper
  → VTKViewer Render + Fit View
```

### 10.2 Apply Parameters

```text
用户修改 Extrude Depth 30 → 40
  → execute(EditFeatureCommand{extrudeId, ...})
  → document.rebuild()
  → EndFace 语义引用由 Core 重新解析
  → Hole Sketch 移到新的顶面
  → Cut 重新生成
  → 更新 vtkPolyData
  → 保持原相机并 Render
```

### 10.3 错误

```text
Command 失败
  → 不调用 rebuild
  → 显示 ModelResult.error

rebuild 失败
  → 显示 document.lastError()
  → 若 bodyShape 非空，显示 Core 保留的最后成功实体
  → 不尝试在前端修复 Shape 或引用
```

---

## 十一、实施里程碑

| 里程碑 | 内容 | 验证方式 |
|---|---|---|
| M0 | 客户端链接 `CadModelCore::CadModelCore` | Core 不修改；客户端编译、启动成功 |
| M1 | 删除硬编码 `vtkCubeSource`，加入 Shape→VTK 适配器 | 使用 Core 创建矩形拉伸，视口显示 50×30×30 长方体 |
| M2 | Controller 通过四个 Add Command 创建完整模型 | 视口显示顶面半径 5、深度 10 的盲孔 |
| M3 | 参数面板通过 Edit Command 修改模型 | 深度 30→40 后孔仍位于新顶面，相机不跳动 |
| M4 | 错误和只读 Feature 状态显示 | 非法深度明确报错，应用不崩溃，最后成功实体仍显示 |
| M5 | 构建、运行和回归验收 | Core smoke test 与客户端构建均通过 |

不得把交互式 CAD 编辑器能力插入 M0–M5。需要草图拾取、任意面选择或撤销时另开后续计划，并仍然以 Core API 为边界。

---

## 十二、验收标准

以下条件必须全部满足：

1. `git diff -- core/parametric_modeling` 没有前端任务造成的修改。
2. 客户端不存在第二份 `Feature`、`PartDocument`、`FeatureParams` 或 ShapeOps。
3. 所有模型写操作均通过 `ModelingCore::execute(ModelCommand)`。
4. 每轮命令完成后显式调用 `PartDocument::rebuild()`。
5. VTK 只显示 `bodyShape()` 的三角化结果。
6. Generate 后显示 `50 × 30 × 30` 带圆形盲孔的模型。
7. Extrude Depth 改为 `40` 后，模型高度和圆孔位置都正确更新。
8. 非法参数或 ReferenceLost 能显示 Core 原始错误，不静默失败、不在前端猜测修复。
9. 现有 Core smoke test 保持通过。
10. 客户端 Qt/VTK 功能不反向进入 Core target。

---

## 十三、不在本阶段实现

- 修改或扩展 Core 建模能力
- 客户端自建 Feature History
- 交互式草图绘制
- 任意面拾取与高亮
- 草图约束求解
- Add/Fuse、多 Body
- Fillet、Chamfer、Loft、Sweep
- Undo/Redo
- 完整工程文件格式
- STEP 导入
- SolidWorks 完整快捷键和 PropertyManager 工作流
- VTK 数据反向生成 Feature 参数

本阶段的完成定义只有一句话：

> Qt 前端通过现有 Command API 生成和编辑 Core 文档，并把 Core 的 `bodyShape()` 转成 VTK 网格正确显示；前端不拥有、不复制、也不实现参数化建模核心。
