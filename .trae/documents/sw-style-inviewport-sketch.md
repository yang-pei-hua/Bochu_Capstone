# SolidWorks 风格视口内草图建模

## Context（背景与目标）

当前 `apps/cad_pointcloud_client` 的草图工作流是「2D 画布 + 下拉选平面」：
在底部 `Sketch` dock 里用一个 `QGraphicsView`（`SketchCanvas`）画 2D 草图，平面从
`sketchPlane` 下拉框（XY / YZ / XZ / Extrude End Face）里选，点/线/圆/矩形靠
`Add Point`、`Add Line`… 按钮往画布上放。这与 SolidWorks 的交互差异很大。

目标：改成 SolidWorks 风格——**在 3D 视口里选面 → 直接在该面上建草图 → 在视口里
用工具在平面上绘制**，工具条紧贴 3D 视图左边缘，退出草图后草图工具隐藏。

用户已确认的 4 项决策：
1. **允许扩展 core**：`SketchPlaneReference` 增加显式平面变体（当前 core 无法表达任意面）。
2. **基准面 + 实体面**都可作为草图平面（空场景可用 XY/YZ/XZ 起步）。
3. 工具条**紧贴 3D 视图左边缘**（与 VTK 视口并排的竖条，不用悬浮覆盖层）。
4. 工具条**常驻、按模式切换内容**。

---

## 一、core 改动（已获授权）

`core/parametric_modeling` 目前 `SketchPlaneReference = variant<DatumPlaneReference,
OffsetDatumPlane, FaceReference>`，且 [RebuildEngine.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp) 的
`resolvePlane()` 末尾用硬 `std::get<FaceReference>` 兜底 → 新增变体前必须先加分支。

1. **[FaceReference.h](file:///d:/code/ECE4500J/core/parametric_modeling/include/modeling/FaceReference.h)**
   新增（`Vec3` 为裸 POD，无几何算子，需内联实现）：
   ```cpp
   struct Plane3d {
       Vec3 origin{};
       Vec3 normal{0.0, 0.0, 1.0};
       Vec3 xDirection{1.0, 0.0, 0.0};
   };
   using SketchPlaneReference = std::variant<
       DatumPlaneReference, OffsetDatumPlane, FaceReference, Plane3d>;
   ```
2. **[RebuildEngine.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/RebuildEngine.cpp) `resolvePlane()`**
   在 `FaceReference` 分支之前插入 `Plane3d` 分支：校验 9 个数有限、`normal` 与
   `xDirection` 非退化且不平行 → 归一化 `normal`，`xDirection` 对 `normal` 正交化，
   `yDirection = normal × xDirection`，填 `occt::PlaneFrame`。失败错误文本：
   `Explicit sketch plane must be finite and non-degenerate`。
3. **[PartDocument.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/src/PartDocument.cpp)**：无需改动 ——
   依赖扫描用 `std::get_if<FaceReference>(&sketch->plane)`（L62），`Plane3d` 自然落入
   「无上游 owner」，`dependentsOf` 语义正确。
4. **[modeling_core_smoke_test.cpp](file:///d:/code/ECE4500J/core/parametric_modeling/tests/modeling_core_smoke_test.cpp)**
   新增 `testExplicitPlaneSketch()`：在倾斜/侧向平面上画矩形并拉伸，断言体积 = 面积 × 深度
   （沿用 `testUnorderedLinesAndReferencePoints` 的断言写法），在 `main()` 注册。
5. **[README.md](file:///d:/code/ECE4500J/core/parametric_modeling/README.md)** authoring capabilities
   补一条「explicit Plane3d sketch planes」。
6. 同步 `apps/cad_pointcloud_client/docs/parametric-core-extension-requests.md`：新增 D3 条目
   （显式平面引用），并记录「显式平面是世界坐标快照」的限制。

---

## 二、客户端改动（`apps/cad_pointcloud_client`）

### 2.1 拾取基础：面索引 + 面平面（精确，不靠三角形法向猜）

- **[OcctShapeTessellator.h/.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/rendering/OcctShapeTessellator.cpp)**
  按 `TopExp_Explorer(shape, TopAbs_FACE)` 的遍历顺序给每个面分配 `faceIndex`，在**输出
  polydata 的 cell data 上写 `FaceIds`（每三角形一个 faceIndex）**（写在 `vtkPolyDataNormals`
  **之后**：SplittingOn 会复制点但不重排/增删 cell，故 cell 顺序与我们的数组一一对应）。
  同时返回 `std::vector<FacePlaneInfo>`：用 `BRepAdaptor_Surface`(OCCT) 判定
  `GetType()==GeomAbs_Plane`，是则取 `gp_Pln` 的 `Location()` / `Axis().Direction()` /
  `XAxis().Direction()` 作为精确的 `origin` / `normal` / `xDirection`——**比三角形法向启发式可靠**。
- **[BodyActor.h/.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/rendering/BodyActor.h)**
  保存 tessellation 结果；新增
  `vtkPolyData* surfacePolyData()`、`int faceCount()`、`const FacePlaneInfo* faceInfo(int)`、
  `int faceIdOfCell(vtkIdType cellId)`；
  新增面高亮 actor（`highlightFace(int)` / `clearHighlight()`）：按该面的 cell 列表构造子
  polydata，沿法向微偏移渲染高亮色。高亮 actor 由 VTKViewer 加入 renderer。

### 2.2 视口交互：自定义 interactor style + 射线求交

- **VTKViewer.cpp** 中 `vtkInteractorStyleTrackballCamera` 改为新的
  `src/ui/viewport/SketchInteractorStyle.h/.cpp`（`vtkInteractorStyleTrackballCamera` 子类），
  实现 SW 键位：**左键 = 拾取/落点（不旋转相机）**，中键 = 旋转，Ctrl+中键 = 平移，
  Shift+中键 = 缩放，滚轮 = 缩放。左键按下/抬起通过 `std::function` 回调抛给 VTKViewer。
- **VTKViewer** 新增：
  - 属性：当前 `ViewportTool`（`Select/Point/Line/Rectangle/Circle`）、是否处于草图模式。
  - `void setSketchPlane(const sketchapp::PlaneFrame&)`（草图模式下射线求交的目标平面）。
  - 信号：`facePicked(int faceId, bool planar, sketchapp::PlaneFrame plane)`、
    `emptyPicked()`、`sketchEntityPicked(modeling::SketchEntityId)`、
    `sketchPointRequested(modeling::Point2D)`。
  - 射线求交：`renderer->SetDisplayPoint(x,y,0/1)` + `DisplayToWorld` 得到世界射线，
    与 `vtkPlane`（草图平面）`IntersectWithLine` → 世界点 → 用 `PlaneFrame` 的
    origin/x/y 点积换算成草图 mm 坐标。
  - 面拾取：`vtkCellPicker` + `PickFromListOn` + 只把 body surface actor 加入 pick list
    → `cellId` → `BodyActor::faceIdOfCell()`。
  - 图元拾取：`vtkCellPicker` 只把 sketch actor 加入 pick list；`SketchActor` 写 cell data
    `EntityIndex`，并在内部维护 `std::vector<modeling::SketchEntityId>` 与该索引对齐。

### 2.3 左侧工具条（新建）

- **新建 `src/ui/panels/SketchToolPalette.h/.cpp`**：`QWidget` 竖条，`objectName=sketchToolPalette`，
  固定宽度、紧贴视口左边缘。两组按钮按模式切换：
  - **模型模式**：`toolSelect`（鼠标/箭头）、基准面按钮组 `planeXY`/`planeYZ`/`planeXZ`、
    `newSketch`（**仅当已选中实体面或基准面时可用**）。
  - **草图模式**：`toolSelect`、`toolPoint`、`toolLine`、`shapeButton`（`QToolButton` + 菜单：
    `shapeRectangle` / `shapeCircle`）、`exitSketch`。
  - 顶部一行提示/状态（当前选择：「Face 3 (planar)」/「XY Plane」/「Nothing selected」）。
- 信号：`toolChanged(ViewportTool)`、`datumPlaneSelected(modeling::DatumPlane)`、
  `newSketchRequested()`、`exitSketchRequested()`。

### 2.4 中央区域与模式接线（MainWindow）

- **[MainWindow.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/MainWindow.cpp) `createCentralArea()`**：
  central widget 由裸 `VTKViewer` 改为 `QWidget{ QHBoxLayout{margin=0,spacing=0} →
  [m_toolPalette(固定宽) | m_viewer(拉伸)] }`。
- 新增 **`m_sketchMode`**（`bool`）与 **`m_pendingSketchPlane`**（`std::optional<modeling::SketchPlaneReference>`）。
  - `facePicked` 非平面 → 状态栏/日志提示，不进入 `m_pendingSketchPlane`；平面 → 记 `Plane3d` 并高亮。
  - `datumPlaneSelected` → 记 `DatumPlaneReference`（并清掉面高亮）。
  - `newSketchRequested` → 用 `m_pendingSketchPlane` 建 `SketchFeatureParams` → `addSketch` →
    进入草图模式（`m_sketchMode=true`，工具条切到草图组，`m_viewer->setSketchPlane(frame)`，
    相机可选切到「正视该面」）。
  - `exitSketchRequested` → `m_sketchMode=false`，工具条回到模型模式，隐藏草图工具体；
    草图仍留在 Feature Graph（未闭合也可保留）。
- **删除** `planeReference()` / `onSketchPlaneRequested()` / `PlaneChoice::ExtrudeEndFace` 这套
  「已提交草图改平面」逻辑：新流程下草图平面在创建时确定，换面 = 在另一个面上新建草图。
- `addSketch` 之后**不再自动进入草图模式以外的新窗口**；`pushSketchState()` 保留（负责把 core
  的 `SketchFeatureParams` + `resolveSketchFrame` 结果推给 `SketchActor` 与图元列表）。

### 2.5 SketchPanel 改造（去掉 2D 画布）

- 删除 `SketchCanvas` 类、`m_planeCombo`、`m_toolCombo`；删除 `newSketchRequested` /
  `planeRequested` / `entityAddRequested` 之外的绘图入口。
- 保留并作为「草图/特征参数」面板：图元列表（点/线/矩形/圆）、状态文本、
  Extrude（深度 / reverse / `ExtrudeOperation` 下拉）、Cut（深度 / throughAll / reverse）、
  删除图元、`Exit Sketch` 也可放这里。`setExtrudeAvailable()` 语义不再需要。
- `SketchEntityPanel`（Properties 的 Sketch 页）：补齐点/线的属性显示与回写
  （点：X/Y；线：X1/Y1/X2/Y2；矩形/圆已有）。

### 2.6 类型适配

- **[SketchFrame.h/.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/core/SketchFrame.h)**：
  保留 `PlaneFrame`、`datumPlaneFrame`、`translatedFrame`、`resolveSketchFrame`；
  新增 `PlaneFrame` ↔ `modeling::Plane3d` 互转；`planeChoiceOf` / `planeReferenceOf` /
  `planeChoiceLabel` / `PlaneChoice` 随平面下拉框一起删除（`resolveSketchFrame` 保留
  `FaceReference` 分支以兼容旧文档）。
- **[ModelingController.h/.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/ModelingController.cpp)**：
  `addSketch(...)` 改为接受 `modeling::SketchPlaneReference`；其余（`extrude` 带 `operation`、
  `cut` 带 `reverse`、`removeFeature(id, cascade)`、`dependentsOf`）不变。
- **CMakeLists.txt**：新增 `SketchToolPalette.*`、`SketchInteractorStyle.*`。

### 2.7 交互状态机（草图模式）

| 工具 | 第 1 次左键 | 第 2 次左键 |
| --- | --- | --- |
| Select | 拾取最近图元 → Properties 显示属性 | — |
| Point | 在平面上落一个 `Point2D` | — |
| Line | 记录起点 + 橡皮筋预览 | 提交 `Line2D`，起点更新为终点（连续画线） |
| Rectangle▾ | 记录角点 | 提交 `Rectangle2D{min, min, w, h}` |
| Circle▾ | 记录圆心 | 提交 `Circle2D{center, r}` |

`Esc` / 右键 → 取消待定操作。退出草图模式后点/线/图形按钮隐藏。

---

## 三、验证

1. **core**：`cmake --build apps/cad_pointcloud_client/build/core-tests --config Release`，
   再把 `deps/occt-8.0.1/win64/vc14/bin` 加入 PATH 后
   `ctest --test-dir apps/cad_pointcloud_client/build/core-tests -C Release --output-on-failure`
   → 含新 `testExplicitPlaneSketch` 全部通过。
   （注意：`core/parametric_modeling/build` 是既有的 Ninja+MinGW 目录，其第三方 SDK 缺
   ffmpeg/freeimage 等 DLL，无法启动——与本改动无关，不改动它。）
2. **客户端 Release 构建**：`cmake --build apps/cad_pointcloud_client/build/msvc-debug --config Release`
   无 error。
3. **重写 `build/sketch-modeling-check.ps1`**（UIA，视口点击前先切标准视图 + Fit View 保证射线命中）：
   无选择时 `newSketch` 禁用 → 选 XY 基准面后启用 → 新建草图（切到草图模式，`1 features`）→
   Select 工具点击不产生图元 → Point 工具点击产生 Point 且 Properties 显示坐标 → Line 工具两次点击
   产生 Line 且 Properties 显示端点 → 退出草图后点/线/图形按钮隐藏 → 模型模式下**点击实体面**选中
   并高亮 → 在该面上新建草图 → Shape▾ 画 Rectangle / Circle → Extrude NewBody/Join →
   Cut reverse → 删除被引用草图弹级联确认框（`used by`）。
4. **更新 `build/ui-restructure-check.ps1`** 中引用已删除控件（2D 画布、`sketchPlane`、`sketchTool`、
   `Add Point/Line/Circle/Rectangle` 按钮）的断言；其余章节（截图/重建/PLY 加载）保持通过。

---

## 四、已知限制（写进文档，不隐藏）

- `Plane3d` 是**世界坐标快照**：草图建在某面上之后，若上游特征（如拉伸深度）改变，草图平面
  不会跟着面移动（core 的稳定面命名 D1 明确不在本版本范围内）。
- 只有**平面**面可作为草图平面；选中非平面面（圆柱面等）会被拒绝并在 UI 提示。
- 相机操作键位改为 SW 风格（左键拾取、中键旋转），原有的左键拖拽旋转不再可用。

---

## 五、改动文件清单

**core（已授权）**：`include/modeling/FaceReference.h`、`src/RebuildEngine.cpp`、
`tests/modeling_core_smoke_test.cpp`、`README.md`

**apps**：
- 新增：`src/ui/panels/SketchToolPalette.h/.cpp`、`src/ui/viewport/SketchInteractorStyle.h/.cpp`
- 修改：`src/rendering/OcctShapeTessellator.h/.cpp`、`src/rendering/BodyActor.h/.cpp`、
  `src/rendering/SketchActor.h/.cpp`、`src/ui/viewport/VTKViewer.h/.cpp`、
  `src/ui/panels/SketchPanel.h/.cpp`、`src/ui/panels/SketchEntityPanel.cpp`、
  `src/app/MainWindow.h/.cpp`、`src/app/ModelingController.h/.cpp`、`src/core/SketchFrame.h/.cpp`、
  `CMakeLists.txt`、`build/sketch-modeling-check.ps1`、`build/ui-restructure-check.ps1`、
  `docs/parametric-core-extension-requests.md`
