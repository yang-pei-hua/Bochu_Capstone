# 工具栏精简 · Capture 改为 Tools 工具 · 源码按功能重组

## 一、背景与目标

当前客户端主工具栏堆了 9 个按钮（Open Model / Reset Camera / Front View / Top View / Side View / Orthographic / Capture Image …），但绝大多数与「模型构建 → 拍摄」这条主线无关，且 `Reset Camera`、`Side View` 等与 View 菜单、Camera 页功能重复。同时 `src/` 下 `MainWindow.{h,cpp}` 游离在 `app/` 之外，`widgets/` 把 3D 视口（VTKViewer）和 6 个停靠面板混在一起，职责边界不清。

本次要做三件事：

1. **精简工具栏**：只保留 `Open Model`。
2. **拍摄改为 Tools 菜单下的工具**：默认不显示；点击后在 Properties 里挂出 Capture 页。
3. **按功能重组源码目录**：`MainWindow` 归入 `app/`，`widgets/` 拆成 `ui/viewport` 与 `ui/panels`。

### 已确认决策（用户选择）

| 项 | 决定 |
|---|---|
| View 菜单 | **保留**（Reset Camera + 6 个标准视角 + File/Help 菜单均不动），只从工具栏移除 |
| Capture 工具入口 | **只放 Tools 菜单**，工具栏不加 |
| 重组力度 | `MainWindow` → `app/`；`widgets/` → `ui/viewport` + `ui/panels`；`core/io/rendering` 不动 |

### 写入范围

- 代码改动**全部在 `apps/cad_pointcloud_client/`** 内，符合项目记忆的 `apps/` 可写约束。
- `.trae/documents/` 下的本计划文件是规划流程的指定产物（与既有 `camera-capture-feature-plan.md`、`cad-modeling-feature-plan.md` 同处），不属于项目源码改动。

---

## 二、当前结构的问题

```
src/
  main.cpp
  MainWindow.h / .cpp        ← 游离在 app/ 之外
  app/       DemoModelIds.h, ModelingController, CaptureController
  core/      Scene, CameraController, OrbitCamera
  io/        ModelLoader.h
  rendering/ BodyActor, OcctShapeTessellator
  widgets/   VTKViewer        ← 3D 视口，不是「面板」
             ModelingPanel, ScenePanel, PropertyPanel,
             CameraPanel, CapturePanel, RenderPanel   ← 真正的停靠面板
```

两处错位：`MainWindow` 属于 app 层却放在 `src/` 根；`widgets/` 把视口控件和停靠面板混为一谈。

---

## 三、目标结构

```
src/
  main.cpp
  app/       MainWindow.h / .cpp        ← 移入
             DemoModelIds.h, ModelingController, CaptureController
  core/      Scene, CameraController, OrbitCamera            ← 不动
  io/        ModelLoader.h                                   ← 不动
  rendering/ BodyActor, OcctShapeTessellator                 ← 不动
  ui/
    viewport/  VTKViewer.h / .cpp
    panels/    PropertyPanel, CameraPanel, CapturePanel,
               RenderPanel, ModelingPanel, ScenePanel
```

因 `CMakeLists.txt` 已有 `target_include_directories(... PRIVATE src)`，所有 include 都相对 `src/` 解析，**无需改任何 include 目录配置**，只需改写文件内的 include 字符串。

**已知的分层代价**：`app/CaptureController.cpp` 需要包含 `ui/viewport/VTKViewer.h`（它驱动视口拍摄），于是 `app` 目录反向依赖 `ui`。`CaptureController.h` 保持只前置声明 `VTKViewer`，耦合被限制在这一个 `.cpp` 里。这是既有 widgets↔app 耦合的平移，不新增问题。

---

## 四、任务 A：工具栏精简

只改 [MainWindow.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/MainWindow.cpp) 的 `createToolBar()`：只留 `toolBar->addAction(m_openAction);`，删掉后续的 Reset Camera / Front View / Top View / Side View（临时 QAction）/ Orthographic / Capture Image 以及所有 `addSeparator()`。

同时**删除两个已无 UI 归属的 QAction**：

- `m_projectionAction`（Orthographic）：原只挂在工具栏，没有任何菜单项。删除后平行投影仍可从 Camera 页 → `Projection` 下拉框切换（走 `CameraPanel::applyRequested → VTKViewer::applyCamera`）。需连带删除：
  - `createActions()` 中的创建；
  - `connectUi()` 中 `connect(m_projectionAction, &QAction::toggled, ...)`；
  - `cameraChanged` lambda 里 `m_projectionAction->setChecked(...)` 与 `setText(...)` 两行（**保留** `updateCameraStatus(parameters.parallelProjection)`）。
- `m_captureAction`（Capture Image）：删除创建与 `connect(m_captureAction, &QAction::triggered, this, &MainWindow::captureImage)`。`MainWindow::captureImage()` 方法**保留**，因为 `RenderPanel::captureRequested` 仍在用它（Render 页那个「Capture Image」按钮是另一套「选路径存单图」流程，本次不动）。

`MainWindow.h` 中移除这两个成员声明。

**保留不动**：`m_resetCameraAction`（View 菜单 + 被 `CameraPanel::resetRequested` 触发）、`m_frontViewAction` / `m_backViewAction` / `m_leftViewAction` / `m_rightViewAction` / `m_topViewAction` / `m_bottomViewAction`（View 菜单）、`m_openAction` / `m_closeAction` / `m_saveAction` / `m_exitAction`（File 菜单）。

> 取舍说明：Top/Bottom 视角是唯一能到达俯仰角 ±90°（正上方/正下方）的入口，Camera 页的 Elevation 被 clamp 在 ±89.9°。它们留在 View 菜单里，能力不丢失。

---

## 五、任务 B：Capture 改为 Tools 菜单工具

### PropertyPanel：Capture 页改为按需挂载

[PropertyPanel.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/PropertyPanel.cpp) 构造函数中 **仍然创建 `m_capturePanel`**（这样 MainWindow 启动时就能给它播种输出目录与环绕距离），但**不调用 `addTab`**：

```cpp
m_tabs->addTab(m_objectTab, tr("Object"));
m_tabs->addTab(m_cameraPanel, tr("Camera"));
m_tabs->addTab(m_renderPanel, tr("Render"));
// Capture is a tool, not a permanent tab: it is inserted on demand.
```

新增 API（[PropertyPanel.h](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/PropertyPanel.h)）：

```cpp
void showCapturePanel(bool visible);
bool isCapturePanelVisible() const;
```

实现要点：用 `m_tabs->indexOf(m_capturePanel)` 判断当前是否已挂载；`true` 时若 `index < 0` 则 `addTab` 并 `setCurrentWidget`；`false` 时 `removeTab(index)`。

**关键点**：`QTabWidget::removeTab()` 只摘掉页签，**不销毁页面控件**，所以隐藏再打开时拍摄列表、输出目录、环绕参数都会原样保留，且不需要重新接线。

### MainWindow：Tools 菜单项

- `createActions()`：新增可勾选 action
  ```cpp
  m_captureToolAction = new QAction(tr("Capture"), this);
  m_captureToolAction->setCheckable(true);
  m_captureToolAction->setToolTip(tr("Show the capture panel in Properties"));
  ```
- `createMenus()`：Tools 菜单里原来那个 disabled 的 `Reserved for future tools` 占位项**替换**为该 action。
- `connectUi()`：
  ```cpp
  connect(m_captureToolAction, &QAction::toggled, this, [this](bool visible) {
      m_propertyPanel->showCapturePanel(visible);
  });
  ```
- `createCaptureController()` **保持现状**（启动时 `setBaseDirectory` + `setDefaultDistance(currentDistance())`）。面板存在但未挂载，播种照常生效；不新增「首次打开再播种」的状态位，避免范围蔓延。

默认状态：action 未勾选 → 启动时 Properties 只有 Object / Camera / Render 三页。

---

## 六、任务 C：源码重组

按「一个子系统一次移动 + 每次移动后立即构建」执行，避免一次性大改产生无法定位的报错风暴。

### C1 移动 `MainWindow.{h,cpp}` → `src/app/`

| 文件 | 改写 |
|---|---|
| [main.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/main.cpp#L1) | `"MainWindow.h"` → `"app/MainWindow.h"` |
| MainWindow.cpp:1 | `"MainWindow.h"` → `"app/MainWindow.h"` |

`MainWindow.h` 自身的 `"app/ModelingController.h"`、`"core/Scene.h"` 相对 `src/` 不变，无需改动。

### C2 移动 `VTKViewer.{h,cpp}` → `src/ui/viewport/`

| 文件 | 改写 |
|---|---|
| VTKViewer.cpp:1 | `"widgets/VTKViewer.h"` → `"ui/viewport/VTKViewer.h"` |
| [CaptureController.cpp:3](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/CaptureController.cpp#L3) | `"widgets/VTKViewer.h"` → `"ui/viewport/VTKViewer.h"` |
| MainWindow.cpp:12 | `"widgets/VTKViewer.h"` → `"ui/viewport/VTKViewer.h"` |

`VTKViewer.h` 的 `"core/CameraController.h"`、`"rendering/BodyActor.h"` 不变。

### C3 移动 6 个面板 → `src/ui/panels/`

每个 `.cpp` 的首行自包含改为 `"ui/panels/<Name>.h"`：CameraPanel、CapturePanel、ModelingPanel、PropertyPanel、RenderPanel、ScenePanel。

跨文件引用改写：

| 文件 | 原 include | 新 include |
|---|---|---|
| PropertyPanel.cpp:3,4,5 | `widgets/{Camera,Capture,Render}Panel.h` | `ui/panels/...` |
| MainWindow.cpp:6–11 | `widgets/{Camera,Capture,Modeling,Property,Render,Scene}Panel.h` | `ui/panels/...` |

面板对其它层的 include **全部不变**：`CameraPanel.h`→`core/CameraController.h`；`CameraPanel.cpp`/`CapturePanel.cpp`→`core/OrbitCamera.h`；`CapturePanel.h`→`app/CaptureController.h`；`ModelingPanel.h`→`app/ModelingController.h`；`ScenePanel.h`→`core/Scene.h`。

### C4 更新 CMakeLists 源列表

`qt_add_executable(...)` 列表最终为：

```
src/main.cpp
src/app/MainWindow.h              src/app/MainWindow.cpp
src/app/DemoModelIds.h
src/app/ModelingController.h      src/app/ModelingController.cpp
src/app/CaptureController.h       src/app/CaptureController.cpp
src/core/Scene.h                  src/core/Scene.cpp
src/core/CameraController.h       src/core/CameraController.cpp
src/core/OrbitCamera.h            src/core/OrbitCamera.cpp
src/io/ModelLoader.h
src/rendering/BodyActor.h         src/rendering/BodyActor.cpp
src/rendering/OcctShapeTessellator.h  src/rendering/OcctShapeTessellator.cpp
src/ui/viewport/VTKViewer.h       src/ui/viewport/VTKViewer.cpp
src/ui/panels/PropertyPanel.h     src/ui/panels/PropertyPanel.cpp
src/ui/panels/CameraPanel.h       src/ui/panels/CameraPanel.cpp
src/ui/panels/CapturePanel.h      src/ui/panels/CapturePanel.cpp
src/ui/panels/RenderPanel.h       src/ui/panels/RenderPanel.cpp
src/ui/panels/ModelingPanel.h     src/ui/panels/ModelingPanel.cpp
src/ui/panels/ScenePanel.h        src/ui/panels/ScenePanel.cpp
```

含 `Q_OBJECT` 的头文件必须继续登记（AUTOMOC 依赖）。

---

## 七、执行顺序

1. 任务 A（工具栏精简 + 删两个 action）→ 构建
2. 任务 B（PropertyPanel 按需挂载 + Tools 菜单项）→ 构建
3. C1 移动 MainWindow → 构建
4. C2 移动 VTKViewer → 构建
5. C3 移动 6 个面板 → 构建
6. C4 已在每次移动时同步 → 最终全量构建 + 验证

---

## 八、验证

### 1. 构建

```powershell
cmake --build "d:\code\ECE4500J\apps\cad_pointcloud_client\build\msvc-debug" --config Release
```

要求 `/W4 /permissive-` 下**无 error、无 `warning C`**。为确认无警告，需 touch 改动的 `.cpp` 后重编（增量构建会跳过未变的翻译单元）。

### 2. 范围回归

- `git diff -- core/parametric_modeling` 为空
- `git status --porcelain` 显示改动全部落在 `apps/cad_pointcloud_client/`
- 旧 `src/widgets/`、`src/MainWindow.*` 目录/文件已清空，无残留孤儿文件

### 3. UI 自动化端到端（`System.Windows.Automation`，沿用前一轮脚本范式）

- **工具栏只剩 Open Model**：枚举 `mainToolBar` 下的 Button 子元素，断言名字集合 == {`Open Model`}
- **View 菜单仍完整**：展开 `View`，断言存在 `Reset Camera` 与 6 个视角项
- **Capture 工具**：
  - 启动时 Properties 页签集合 == {Object, Camera, Render}（**无 Capture**）
  - 展开 `Tools`，激活 `Capture` → 页签集合变为 {Object, Camera, Render, Capture} 且当前页为 Capture
  - 再次取消勾选 → 页签集合回到三页
  - 重新勾选 → Capture 页回来，且**已拍列表/输出目录/环绕参数保持原值**（验证 `removeTab` 不销毁控件）
- **拍摄功能未被重构破坏**：在 Capture 页设 Count=8 / Elevation=30 / Distance=10，点 `Capture Orbit`，断言
  - 状态序列出现 `8/8`，最终为 `Captured 8 photos`
  - 时间戳目录内 `shot_000..007.png` + 同名 `.json` + `manifest.json` 齐全
  - `manifest.json`：`count=8`、`stepAzimuthDeg=45`、方位角 0/45/…/315
  - sidecar：`distance=10`、`elevationDeg=30`、`target=(0,0,0)`、`output=1920x1080`
  - 批量前后相机 position/target/FOV 逐位一致（up 按既定正交化残差断言）
- **既有功能回归**：Apply Parameters 把 Extrude Depth 30→40，状态仍为 `Rebuild succeeded | 4 features`

### 4. 截图留证

工具栏（只有 Open Model）、展开的 Tools 菜单（含勾选中的 Capture）、Capture 页（8 条拍摄记录）。

---

## 九、需要注意的问题

1. **`m_resetCameraAction` 不能删**：`CameraPanel` 的 Reset 按钮通过 `connect(cameraPanel, &CameraPanel::resetRequested, m_resetCameraAction, &QAction::trigger)` 复用它。只从工具栏移除，action 本身保留。
2. **`m_projectionAction` 删除后要清干净同步代码**：`cameraChanged` lambda 里有两行操作它，漏删会编译失败；`updateCameraStatus` 调用保留。
3. **`removeTab` 而非删除控件**：务必不要在隐藏时 `delete` 面板，否则重新打开会丢掉已拍列表并需要重新接线。
4. **文件移动必须同时改 CMakeLists**：`qt_add_executable` 是显式源列表，漏登记会导致 MOC 缺失或链接错误；移动后务必确认 `src/widgets/`、`src/MainWindow.*` 没有残留文件。
5. **本任务不改动 `core/parametric_modeling`**：M0–M5 的建模核心与其门禁不受影响。