# 相机环绕控制与多视角拍摄计划

## 一、背景与目标

当前客户端已能编辑相机的原始 `position/target/up/FOV/投影`（[CameraPanel.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/CameraPanel.cpp)），也能把视口按指定分辨率存成单张 PNG（[VTKViewer.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/VTKViewer.cpp#L89-L115)）。但无法用「方位角 / 俯仰角 / 距离」这种符合拍摄直觉的方式摆放相机，也无法批量产出多视角照片，更没有记录每张照片对应的相机位姿。

本次要补齐这条链路，落成 README 中已列为 deferred 的 “Multi-view dataset and camera-pose generation”：

1. Camera 页新增**环绕式相机控制**（Distance / Azimuth / Elevation），默认**对准世界原点**，并提供「Look at origin」开关决定是否始终对焦零点。
2. 新增 **Capture 页**：手动单张拍摄、一键环绕批量拍摄。
3. 每张照片落盘时**记录相机坐标与方位信息**。

### 已确认的决策（来自用户）

| 项 | 决定 |
|---|---|
| 一键拍摄角度集合 | 仅**环绕水平一圈**（方位角 N 等分，俯仰角可设） |
| 输出组织 | 每次拍摄**新建时间戳子目录** |
| 元数据格式 | **单图 sidecar JSON + 目录 manifest.json** |
| 界面位置 | 俯仰角/环绕控制进 **Camera 页**；拍摄操作新建 **Capture 页** |

### 写入范围

本次改动**全部位于 `apps/cad_pointcloud_client/`**，不触碰 `core/parametric_modeling`、`deps/`、`configs/`。
唯一例外是本计划文件本身位于 `.trae/documents/`，与该目录既有的计划文档工作流一致。
应用运行期把照片写到用户选择的目录（默认在用户图片目录下），这是产品行为，不属于 AI 的写入范围约束。

---

## 二、复用清单（不重复造轮子）

| 已有能力 | 位置 | 本次如何用 |
|---|---|---|
| `CameraParameters` + `CameraController::apply/parameters` | [CameraController.h](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/core/CameraController.h) | 直接复用，不改其语义 |
| `VTKViewer::applyCamera()` → 渲染并发 `cameraChanged` | [VTKViewer.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/VTKViewer.cpp#L134-L140) | 批量拍摄逐步设相机 |
| `VTKViewer::captureImage(path,w,h)` | [VTKViewer.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/VTKViewer.cpp#L89-L115) | 直接产出 PNG，**不新写截图代码** |
| `VTKViewer::cameraParameters()` | 同上 | 批量前取原相机用于精确还原 |
| `RenderPanel::outputWidth/Height()` | [RenderPanel.h](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/RenderPanel.h) | 作为拍摄分辨率来源 |
| `PropertyPanel` 的 tab 机制 | [PropertyPanel.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/widgets/PropertyPanel.cpp#L45-L59) | 挂第 4 个 tab |
| `ModelingController` 的控制器范式 | [ModelingController.h](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/ModelingController.h) | `CaptureController` 照此风格 |
| `MainWindow::logInfo/logError` + `QMessageBox::warning` | [MainWindow.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/MainWindow.cpp#L422-L441) | 沿用既有报错风格 |

**`VTKViewer` / `RenderPanel` / `CameraController` 的现有接口无需改动。**

---

## 三、文件清单

### 新建（6 个）

**`src/core/OrbitCamera.h` / `.cpp`** — 纯数学，只依赖 `CameraParameters`，不依赖 VTK/Qt。

```cpp
struct OrbitParameters {
    double distance = 5.0;
    double azimuthDeg = 0.0;    // 绕 +Z，自 +X 起算
    double elevationDeg = 0.0;  // 俯仰角，+ 为向上
};
class OrbitCamera {
public:
    static constexpr double kMaxElevation = 89.9;
    static OrbitParameters fromCamera(const CameraParameters&);   // 反解
    static CameraParameters toCamera(const OrbitParameters&,
                                     const CameraParameters& base,  // 保留 fov/projection
                                     bool lookAtOrigin);
};
```

- `direction = (cosE·cosA, cosE·sinA, sinE)`，`position = target + distance·direction`。
- `up = (-sinE·cosA, -sinE·sinA, cosE)`，随环绕同步旋转、保持水平线。
- 反解：`d = position - target`；`distance = |d|`；`azimuth = atan2(dy, dx)`；`elevation = asin(dz / distance)`。
- **万向节死锁处理（双重防御）**：Elevation 输入框范围限死 `[-89.9, 89.9]`，且 `toCamera()` 内部再 `clamp`。真正的极视图仍可由 View 菜单的 Top/Bottom 标准视图得到，功能不丢。

**`src/app/CaptureController.h` / `.cpp`** — `QObject` 控制器，持有 `VTKViewer*`、会话内已拍摄列表、输出基目录；负责目录创建、PNG 落盘、JSON 写出、批量循环与暂停恢复。依 `ModelingController` 风格：`bool` 返回 + `QString m_lastError` + `captureError(QString)` 信号。

**`src/widgets/CapturePanel.h` / `.cpp`** — 第 4 个 tab 的 UI，只收集参数并转发，不持有模型或相机状态。

### 修改（4 个）

**`CMakeLists.txt`** — 把上述 6 个 `.h/.cpp` 全部加进 [qt_add_executable 的显式文件列表](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/CMakeLists.txt#L48-L76)（头文件也要列）。`QJsonDocument/QFile/QDir/QDateTime/QStandardPaths` 都在已链接的 `Qt6::Core` 内，**无需新增 `find_package` 或 Qt 模块**。

**`src/widgets/CameraPanel.h` / `.cpp`** — 加环绕控件组与 Look at origin 开关。

**`src/widgets/PropertyPanel.h` / `.cpp`** — 加 `m_capturePanel`、`m_tabs->addTab(m_capturePanel, tr("Capture"))`、访问器 `CapturePanel* capturePanel() const`。

**`src/MainWindow.h` / `.cpp`** — 构造 `CaptureController` 并在 `connectUi()` 连线；错误经 `logError` + `QMessageBox::warning` 呈现。

---

## 四、Camera 页：环绕控制与对焦开关

新增控件：
- `Distance`（距离，>0）
- `Azimuth`（方位角，`[0,360)`，后缀 `°`）
- `Elevation`（俯仰角，`[-89.9,89.9]`，后缀 `°`）
- `Look at origin` 复选框，**默认勾选**（即默认对准零点）

行为：
- **勾选时**：`target` 强制 `(0,0,0)`、`up` 强制 `(0,0,1)`，三个 Target 输入框 `setEnabled(false)`；编辑 Distance/Azimuth/Elevation 即算出 position 并应用。
- **取消勾选时**：Target/Up 恢复可编辑，环绕三框转为只读派生值。
- 用户在视口里用鼠标轨道旋转/平移时（非本面板发起），从相机反解并回写环绕字段；勾选状态下把 target 拉回原点，正是「始终对焦零点」的语义。

**同步防抖（关键）**：`applyRequested → applyCamera → cameraChanged → setParameters` 是同步直连，用发射期标记隔断回环：

```cpp
bool m_emitInProgress = false;  // 本轮 apply 由本面板发起
// 编辑时: m_emitInProgress = true; emit applyRequested(...); m_emitInProgress = false;
// setParameters() 开头: if (m_emitInProgress) return;   // 既不抖，也不吞用户正在输入的值
```

**注意**：`CameraPanel` 构造函数末尾已调用 `setParameters(CameraParameters{})`，`MainWindow` 构造中还会再调一次 [cameraPanel()->setParameters(...)](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/MainWindow.cpp#L51)。因此必须在构造里**先**建好复选框与环绕控件（复选框先 `setChecked(true)`），最后才调 `setParameters`。默认相机 `(3,3,3)→(0,0,0)` 反解得 distance≈5.197、azimuth=45°、elevation≈35.26°，非退化，安全。

---

## 五、Capture 页

- **输出目录**：只读路径显示 + `Browse...`（`QFileDialog::getExistingDirectory`）。默认 `QStandardPaths::PicturesLocation`（取不到则回退 `QDir::homePath()`）。
- **Capture Photo**（手动单张）：新建时间戳子目录，存 1 张 PNG + 同名 sidecar JSON。
- **Capture Orbit**（一键批量）：`Count`（`[1,360]`，默认 8）+ `Elevation`（默认 30）+ `Distance`（默认取当前相机距离）；方位角按 `360/Count` 等分铺满一圈。
- **已拍摄列表**（只读）：序号 / 文件名 / 方位角 / 俯仰角。
- **状态标签**：`Capturing 3/8` 之类进度，结束后显示结果或 Core 风格错误原文。

### 输出组织

```text
<base>/20260926_143012/
├── shot_000.png
├── shot_000.json
├── shot_001.png
├── shot_001.json
└── manifest.json
```

同秒重名时追加 `-2` 后缀；`QDir::mkpath` 失败即 `logError` + `QMessageBox::warning` 并中止。

### JSON Schema

Sidecar `shot_000.json`：

```json
{ "schema": "cadpc.capture.sidecar", "version": 1,
  "image": "shot_000.png", "group": "20260926_143012",
  "capturedAt": "2026-09-26T14:30:12+08:00",
  "camera": { "position": [x,y,z], "target": [x,y,z], "up": [x,y,z],
              "azimuthDeg": 135.0, "elevationDeg": 30.0, "distance": 10.0,
              "fieldOfViewDeg": 30.0, "projection": "perspective" },
  "output": { "width": 1920, "height": 1080 } }
```

Manifest `manifest.json`：

```json
{ "schema": "cadpc.capture.manifest", "version": 1,
  "group": "20260926_143012", "createdAt": "...", "mode": "orbit",
  "orbit": { "count": 8, "startAzimuthDeg": 0.0, "stepAzimuthDeg": 45.0,
             "elevationDeg": 30.0, "distance": 10.0 },
  "shots": [ { "index": 0, "image": "shot_000.png",
               "azimuthDeg": 0.0, "elevationDeg": 30.0 } ] }
```

手动单张时 `mode` 为 `"single"`、`orbit` 为 `null`。写盘用 `QFile::open(WriteOnly|Truncate)` + `QJsonDocument::toJson(Indented)`，检查 `write() == -1`。

### 批量循环（UI 线程内同步）

```
if (m_batchRunning) return;                  // 防重入
m_batchRunning = true;  面板两按钮 disable
const CameraParameters original = viewer->cameraParameters();   // 已正交化，回放幂等
mkdir 时间戳组目录
for i in [0,N):
    az = i * (360.0/N)
    viewer->applyCamera(OrbitCamera::toCamera({dist,az,elev}, original, true))
    ok = viewer->captureImage(dir/"shot_%03d.png".arg(i), w, h)
    写 sidecar；刷新列表与状态
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents)   // 让进度重绘
viewer->applyCamera(original)                // 精确还原原相机并重渲染
m_batchRunning = false
```

- `renderNow()` 不泵事件，进度必须靠 `processEvents` 刷新；`ExcludeUserInputEvents` 必须带，避免用户点击插进循环。
- **重入风险（最易出错处）**：`applyCamera` 会同步触发 `cameraChanged → CameraPanel::setParameters`。批量期间必须让 `CameraPanel` 进入抑制态（复用第四节标记或加 `setFeedbackSuppressed(bool)`），否则每步都回写环绕框并可能触发「归零重发」。
- 还原必须取自 `cameraParameters()` 的 `original`，再次 `apply` 幂等，可逐位还原。
- 任一 `captureImage` 失败即 `logError` 并中止剩余循环。

---

## 六、需要留意的问题

1. **`vtkImageResize` 会非等比拉伸**：当窗口宽高比与输出宽高比不一致时（如 480×320 窗口输出 1920×1080）图像会变形。这是既有行为，本次不修，仅在 UI 上以提示文案说明。
2. **命名撞车**：工具栏/File 菜单的 `Capture Image` action 与 `RenderPanel` 的 `Capture Image` 按钮都走旧的「选路径存单图」流程。建议把该 action 改名为 `Save Viewport PNG...` 以区别于新面板的 `Capture Photo`。此改名不是用户要求，属可选澄清项，若不需要可去掉。
3. **UI 自动化依赖 objectName**：新控件必须显式 `setObjectName`，否则 `System.Windows.Automation` 定位不到 —— `captureTab`、`captureBaseDir`、`captureBrowseButton`、`capturePhotoButton`、`captureOrbitButton`、`captureCount`、`captureElevation`、`captureDistance`、`captureStatus`、`captureShotList`，以及 `CameraPanel` 的 `orbitDistance`、`orbitAzimuth`、`orbitElevation`、`lookAtOrigin`。

---

## 七、验证

1. **构建**：`cmake --build --preset msvc-release --parallel 8`（在 `apps/cad_pointcloud_client`），要求 `/W4` 下无警告、无错误。
2. **回归**：确认 `git diff --stat -- core/parametric_modeling` 为空；M0–M5 既有功能（建模面板、Apply Parameters）仍工作。
3. **UI 自动化端到端**（沿用项目既有 `System.Windows.Automation` PowerShell 脚本）：
   - 启动 `build/msvc-debug/Release/CadPointCloudClient.exe`，切到 Capture tab，断言输出目录非空。
   - 设 `Count=8`、Elevation=30、Distance=10，点 `Capture Orbit`。
   - 断言状态出现过 `8/8`；断言时间戳目录内含 `shot_000.png`…`shot_007.png`、同名 `.json`、`manifest.json`。
   - 解析 `manifest.json`：`count==8`、`stepAzimuthDeg==45`，`shots[].azimuthDeg` 为 0/45/90…315。
   - 抽验 sidecar：`camera.distance≈10`、`elevationDeg≈30`、`output` 为 1920×1080、`projection=="perspective"`。
   - **相机还原**：批量前后各读一次相机参数，断言一致。
   - 回 Camera tab 勾选 `lookAtOrigin`，改 Azimuth，断言 Target 三框被 disable 且恒为 0；在视口内用鼠标轨道旋转后确认字段不抖动、target 被拉回原点。
4. **截图留证**：拍摄完成后对主窗口截图，确认 Capture 页列表与状态文本。