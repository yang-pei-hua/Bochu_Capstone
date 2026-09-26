# 点云重建流程接入软件 — 实施计划

## Context（背景与目标）

用户请求原文：

> 帮我把接受多视角图片生成并渲染点云的流程接入软件

目标：在 `CadPointCloudClient` 中把「接收多视角图片 → COLMAP 重建 → 生成 PLY 点云 → 在视口中渲染」这条链路做成可用功能，
入口是工具栏上现已存在的**「点云重建」按钮**（当前为禁用占位）。

经 AskUserQuestion 澄清，用户选定（均为推荐项）：

| 议题 | 用户选择 | 含义 |
| --- | --- | --- |
| 图像来源 | 两种都支持 | 面板可选任意图片文件夹，也可一键使用软件最近一次拍摄的照片组 |
| 相机位姿 | 两者都要 | 有拍摄 sidecar 时自动生成 CameraInfo（固定位姿、尺度即模型单位）；没有时退回 COLMAP 自动估计 |
| 重建模式 | 稀疏默认 + 稠密开关 | 默认稀疏 PLY；面板提供 `--dense --gpu` 开关 |
| 点云显示 | 与模型共存 | Scene 树新增 Point Cloud 节点，可与 CAD 模型同时显示、各自切换可见性，载入后自动适配视图 |

**强制边界（最高优先级）**：唯一允许写入的目录是 `apps/`（即 `d:\code\ECE4500J\apps`）。
`core/`、`configs/`、`data/`、`docs/`、`outputs/`、`pipeline/`、`.vscode/` 及仓库根文件一律**只读**。
`deps/colmap/`、`deps/vtk-9.7.0/`、`deps/occt-8.0.1/`、`core/reconstruction/install/colmap/` 为不透明依赖：不递归列出/搜索/读取。
（本计划文件位于 `.trae/documents/`，是 plan mode 规定的规划产物，不属于交付物。）

---

## 现状分析

### 1. 仓库已有完整的 COLMAP 无头编排层（只读复用，不改）

`core/reconstruction/` 是一个纯标准库 Python 包，我们只通过 CLI 调用，**不重写编排逻辑**：

- 入口：[reconstruct.py](file:///d:/code/ECE4500J/core/reconstruction/reconstruct.py) → `from colmap_reconstruction.cli import main`
  （脚本目录会进入 `sys.path`，因此可从任意工作目录启动）
- CLI：[cli.py](file:///d:/code/ECE4500J/core/reconstruction/colmap_reconstruction/cli.py#L16-L59)
  `--images --output(.ply) --camera-info --workspace --colmap --dense --matcher {exhaustive,sequential} --gpu --input-unit {unknown,mm,m}`
  - 成功时 stdout 打印 4 行：`point cloud: <path>` / `points: <n>` / `camera mode: <mode>` / `manifest: <path>`
  - 失败时 stderr 打印 `error: <msg>`，**退出码 2**
- 流水线：[pipeline.py](file:///d:/code/ECE4500J/core/reconstruction/colmap_reconstruction/pipeline.py#L211-L373)
  - 约束：`output` 必须 `.ply` 且**不存在**（拒绝覆盖）；`workspace` 必须**不存在或为空目录**
  - 每次 COLMAP 调用前向 `<workspace>/colmap.log` **追加**一行 `$ colmap <command> <args>`（可用于进度轮询）
  - 稀疏顺序：`feature_extractor` → `{exhaustive|sequential}_matcher` → `mapper`（无位姿）/ `point_triangulator`（全位姿）/ `point_triangulator`+`mapper --Mapper.fix_existing_frames 1`（部分位姿）→ `model_converter --output_type PLY`
  - 稠密顺序：…同上… → `image_undistorter` → `patch_match_stereo` → `stereo_fusion`（**稠密分支不再跑 `model_converter`**）
  - 结束时写 `<workspace>/reconstruction.json`；若 `point_count <= 0` 抛 `reconstruction produced an empty point cloud`
  - [find_colmap_executable](file:///d:/code/ECE4500J/core/reconstruction/colmap_reconstruction/pipeline.py#L51-L76) 会自动落到 `project_root/deps/colmap/bin/colmap.exe`，**无需也不能传 `--colmap`**
- CameraInfo 校验与坐标转换：[camera_info.py](file:///d:/code/ECE4500J/core/reconstruction/colmap_reconstruction/camera_info.py#L109-L146)
  - `camera_to_world {qvec, position}`：内部 `q_wc = (w,-x,-y,-z)`、`tvec = -R(q_wc)·position` —— 语义即「相机机体系 → 世界系」旋转 + 相机中心在世界系
  - 相机机体系为 COLMAP 约定：**X 右、Y 下、Z 前**
  - `PINHOLE` 参数为 `[fx, fy, cx, cy]`，`model_id = 1`
  - `images[].name` 相对 `--images`、用正斜杠、无 `..`、**集合必须与 COLMAP 抽取到的图片集合完全相等**
  - 位姿数量必须为 **0 或 ≥2**；`cameras[*].width/height` 必须与该图片**实际像素尺寸**一致（否则 pipeline 直接报错）
- 已实测确认本机 COLMAP 可用：`deps/colmap/bin/colmap.exe` 支持 CUDA，且具备
  `image_undistorter` / `point_triangulator` / `patch_match_stereo` / `stereo_fusion`（稠密可用）
- 已实测确认 PLY 输出格式（读取了既有产物头部）：
  - [fountain_p11_sparse.ply](file:///d:/code/ECE4500J/outputs/fountain_p11_sparse.ply)：`format binary_little_endian 1.0`、`element vertex 13668`、属性 `x y z red green blue`，无 face、无 alpha（15 B/顶点）
  - [fountain_p11_dense.ply](file:///d:/code/ECE4500J/outputs/fountain_p11_dense.ply)：`element vertex 1632443`、属性 `x y z nx ny nz red green blue`（27 B/顶点）

### 2. 客户端已有的可复用结构

- 工具栏已把流程阶段搬进「Open Model 统一栏」：[MainWindow.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/MainWindow.cpp#L190-L211)
  现布局 `[Open Model] | [Capture Image] [点云重建(禁用)] [几何拟合(禁用)] [AI 修改(禁用)]`
- 「按需挂载面板」已有成熟范式（Capture）：[PropertyPanel::showCapturePanel](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/ui/panels/PropertyPanel.cpp#L79-L92)
  —— `addTab`/`removeTab` 只解挂不销毁，参数在隐藏/再显示间保留
- 拍摄输出与元数据：[CaptureController](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/CaptureController.cpp)
  - 每次拍摄建 `<baseDirectory>/yyyyMMdd_HHmmss/`（同秒加 `-2` 后缀），写 `shot_XXX.png` + 同名 `.json` sidecar + `manifest.json`
  - sidecar 的 `camera` 含 `position / target / up / fieldOfViewDeg / projection`，`output` 含 `width / height`
  - 默认 `baseDirectory = QStandardPaths::PicturesLocation`（本机为 `D:\Pictures`）
  - `createGroupDirectory()` 的命名策略是**按名字字典序即时间序**，可直接用于「最近一次照片组」
- 视口：[VTKViewer](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/ui/viewport/VTKViewer.h) 目前只管理 `BodyActor`（CAD 体）+ 方向标
- 场景树：[Scene](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/core/Scene.cpp) 为静态节点表；[ScenePanel::setScene](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/ui/panels/ScenePanel.cpp#L33-L57) 重建树并选中 `DemoModel`
- 构建：CMake 显式源文件列表（含 `Q_OBJECT` 的头必须登记），`target_include_directories(... PRIVATE src)` → include 相对 `src/` 解析
- **依赖限制（关键）**：vendored VTK **没有 `IOPLY` 模块**（`deps/vtk-9.7.0/lib/cmake/vtk-9.7` 下无 `vtk*IOPLY*.cmake`），
  链接清单也**没有 `FiltersGeneral`** → 不能用 `vtkPLYReader`，也**不能用 `vtkVertexGlyphFilter`**；
  必须自写 PLY 解析 + 手工构造 vertex cell。本方案**不新增任何 VTK 模块与 Qt 组件**。

### 3. 已知的语义陷阱（必须在实现中处理）

1. **`vtkCamera::GetViewAngle()` 是垂直视场角**（angular height），不是水平。
2. `VTKViewer::captureImage()` 用 `vtkImageResize` 把视口图**非等比拉伸**到 Render 面板配置的输出尺寸。
   因此最终图片的水平焦距与垂直焦距**一般不等**：
   `fx = fy * (W * Hw) / (H * Ww)`，其中 `(Ww, Hw)` 是渲染窗口（拉伸前）的像素尺寸、`(W, H)` 是输出尺寸。
   sidecar 目前**只记录 `(W, H)`**，不记录 `(Ww, Hw)` —— 需要补一个字段，否则内参错约 18%（1500×920 窗口 vs 1920×1080 输出）。
3. sidecar 里 `projection` 可能是 `"orthographic"`，正交投影无法用针孔模型描述。
4. COLMAP `point_triangulator` 不需要图片之间有共同可见点以外的额外条件，但**仍然依赖 2D 特征匹配**才能三角化 —— 即便位姿已知。
5. 稀疏重建在 CPU 上对多张大图可能耗时数分钟 → 必须可取消，且不能阻塞 UI。

---

## 关键约定（实现必须严格遵守）

### CameraInfo 数学

设 sidecar 给出：相机中心 `P = position`（世界/模型系）、注视点 `T = target`、上方向 `U = up`、垂直视场角 `fov = fieldOfViewDeg`、
输出尺寸 `(W, H)`、拉伸前渲染窗口尺寸 `(Ww, Hw)`（新增字段，缺失时退回 `Ww/Hw = W/H`）。

1. `f = normalize(T - P)` → 世界系下的相机 **+Z（前）** 轴
2. `r = normalize(cross(f, U))` → 世界系下的相机 **+X（右）** 轴
3. `d = normalize(cross(f, r))` → 世界系下的相机 **+Y（下）** 轴（等于 `-U` 的正交化版本）
4. `R_cw = [r | d | f]`（**按列**拼接，即机体系→世界系）
   ⇒ `R_wc = R_cwᵀ = [r; d; f]`（按行），`v_cam = R_wc · v_world`
5. `tvec = -R_wc · P`（仅用于自检；写 JSON 时**不输出 tvec**）
6. quaternion：由 4 的 `R_cw` 求 Hamilton 四元数 `(w,x,y,z)`，用 Shepperd 分支法保证数值稳定
7. 内参（`PINHOLE`，`params = [fx, fy, cx, cy]`）：
   - `fy = (H / 2) / tan(fov / 2)`
   - `fx = fy * (W * Hw) / (H * Ww)`
   - `cx = W / 2`，`cy = H / 2`
8. 输出 JSON（每位姿写 `camera_to_world`，最小化符号出错面）：

```json
{
  "cameras": {
    "cam_0": { "model": "PINHOLE", "width": 1920, "height": 1080,
               "params": [2393.667, 2015.303, 960.0, 540.0] }
  },
  "images": [
    { "name": "shot_000.png", "camera": "cam_0",
      "camera_to_world": { "qvec": [w, x, y, z], "position": [8.660254037844387, 0, 5.0] } }
  ]
}
```

- `name` 用**正斜杠**、相对 `--images`、无 `..`
- `cameras` 按 `(width, height, fov, 拉伸比)` 去重生成 `cam_0..cam_n`（同目录多组不同输出尺寸也能正确处理）

### CameraInfo 启用规则（单一、诚实的判据）

**仅当目录内每张图片都有可用的透视 sidecar，且图片数 ≥ 2 时**才生成并传 `--camera-info`（结果必为 `camera mode: provided`）。
否则**完全不传**，交由 COLMAP 自动估计（`estimated`）。跳过原因写进日志，不让流程失败。

跳过原因枚举：`图片数量不足` / `目录内没有拍摄元数据(sidecar)` / `部分图片缺少拍摄元数据` / `含正交投影(orthographic)照片`。

理由：pipeline 会**严格校验** `cameras[*].width/height` 与实际图片像素尺寸一致，而混合目录里"没有 sidecar 的图片"其真实尺寸无从得知（读 PNG/JPEG 尺寸需额外解码逻辑）。
全有/全无的判据从根上消除这一类失败，并且与用户选择的语义（有 sidecar→固定位姿、没有→自动估计）完全一致。
pipeline 的 partial/seed 分支本方案**故意不使用**（未充分验证的路径，不引入不必要的复杂度）。

### 重建进度阶段表（用于解析 `colmap.log`）

| 命令 | 阶段序号 | 中文标签 |
| --- | --- | --- |
| `feature_extractor` | 1 | 特征提取 |
| `exhaustive_matcher` / `sequential_matcher` | 2 | 特征匹配 |
| `mapper` / `point_triangulator` | 3 | 稀疏重建 |
| `image_undistorter` | 4 | 图像去畸变 |
| `patch_match_stereo` | 5 | 稠密深度图 |
| `stereo_fusion` | 6 | 稠密点云融合 |
| `model_converter` | 稀疏 4 / 稠密 6 | 导出 PLY |

总阶段数：稀疏 **4**、稠密 **6**。`point_triangulator` 之后若再跑一次 `mapper` 仍计入阶段 3（不推进进度条）。

### 运行目录布局

```
<outputRoot>/<yyyyMMdd_HHmmss>/          # outputRoot 默认 <Pictures>/cadpc_reconstructions
    cloud.ply                            # --output（必须不存在）
    camera_info.json                     # --camera-info（仅当启用时）
    workspace/                           # --workspace（由 Python 侧创建，我们不要预先创建）
        colmap.log
        reconstruction.json
```
`workspace` 必须**不存在或为空**：新建的 run 目录天然满足，**不要**提前 `mkpath` workspace。

---

## 实施方案

### 新增文件（全部位于 `apps/cad_pointcloud_client/src/`）

| 文件 | 职责 |
| --- | --- |
| `io/PlyReader.h/.cpp` | 零依赖 PLY 解析（ASCII + `binary_little_endian` + `binary_big_endian`） |
| `io/CameraInfoBuilder.h/.cpp` | 由 sidecar 生成 CameraInfo JSON（上文数学约定） |
| `rendering/PointCloudActor.h/.cpp` | 点云 VTK 管线（`vtkPoints` + 手工 vertex cell + 可选直接标量着色） |
| `app/ReconstructionController.h/.cpp` | `QProcess` 异步驱动 Python CLI、进度轮询、取消、结果解析 |
| `ui/panels/ReconstructPanel.h/.cpp` | 重建参数面板（图片目录 / 输出目录 / 选项 / 运行 / 进度） |

### 修改文件

| 文件 | 改动 |
| --- | --- |
| `ui/viewport/VTKViewer.h/.cpp` | 点云 API + 记录「拉伸前渲染窗口尺寸」 |
| `app/CaptureController.cpp` | sidecar 增加 `render {width,height}` 字段（内参必需） |
| `core/Scene.h/.cpp` | 新增 `SceneNodeType::PointCloud` + `Point Clouds` 分组节点 |
| `ui/panels/ScenePanel.h/.cpp` | 节点 id→item 映射 + `updateNodeLabel()` |
| `ui/panels/PropertyPanel.h/.cpp` | `showReconstructPanel()` / `isReconstructPanelVisible()` / `reconstructPanel()` |
| `app/MainWindow.h/.cpp` | 工具栏「点云重建」改为可用可勾选动作、Tools 菜单同步、全链路接线 |
| `CMakeLists.txt` | 源列表新增 10 项 |
| `build/ui-restructure-check.ps1` | 断言更新：5 个按钮中**启用 3 个**、禁用 2 个 |

---

### Step 0 — 无头可行性验证（去风险，先做，做完再写 UI 代码）

目的：在只读检查阶段就把「位姿/内参约定」和「DemoModel 是否有足够特征」这两个最大风险证伪或证实。

新建 `apps/cad_pointcloud_client/build/headless-reconstruction-check.py`（Python 3.13 已确认可用），内容：

1. 读 `D:\Pictures\20260926_180805\shot_*.json`（8 张实拍 sidecar + manifest）
2. 按上文数学约定生成 `camera_info.json` 写到临时目录（用 `$env:TEMP` 或 `build/reconstruct-check/`）
3. 运行
   `python core/reconstruction/reconstruct.py --images D:\Pictures\20260926_180805 --output <tmp>\cloud.ply --workspace <tmp>\ws --camera-info <tmp>\ci.json`
4. 断言：
   - 退出码 0 且 stdout 含 `camera mode: provided`
   - `points: N`，且 `N > 0`
   - PLY 包围盒直径在 `[0.5, 30]`（DemoModel 的尺度量级），质心接近原点
   - 若 `colmap.log` 中含 `Mean reprojection error`，要求 < 2 px（有助于发现外参转置/符号错）
5. 输出失败时的完整 stderr 与 `colmap.log` 末尾若干行

**若 8 张立方体渲染图产出 0 点**（低纹理平面 + 平光，SIFT 特征可能不足）：
如实记录结论，并把结论写回本计划文件的「实施记录」，不要为了"过测试"而调参或造假数据。
此时该功能的正确性用另一条路径验证（见 Step 9 的候选素材）。

### Step 1 — `PlyReader`

```cpp
// io/PlyReader.h
struct PlyCloud
{
    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<unsigned char, 3>> colors;   // 无颜色属性时为空
};

class PlyReader
{
public:
    static bool read(const QString& path, PlyCloud& cloud, QString& error);
};
```

实现要点：
- 头部逐行解析至 `end_header`：`format {ascii|binary_little_endian|binary_big_endian} 1.0`、`element <name> <count>`、`property <type> <name>`、`property list <count-type> <item-type> <name>`
- 顶点属性**按名字查表**取值（`x y z`、`red green blue`），不假设顺序 —— 因此自动兼容 `x y z nx ny nz red green blue`
- 标量类型宽度表：`char/int8=1, uchar/uint8=1, short/int16=2, ushort/uint16=2, int/int32=4, uint/uint32=4, float/float32=4, double/float64=8`
- 非顶点 element（如 `face`）整体跳过；顶点 element 内出现 `property list` 视为不支持 → 报错返回 false
- 颜色：任一顶点缺色则**整体丢弃** colors（保证与 positions 等长）
- ASCII 分支按空白切分逐属性读取
- 守卫：二进制时校验剩余字节数 ≥ `count * stride`；`x/y/z` 缺失或 `count == 0` → 报错

### Step 2 — `PointCloudActor` + `VTKViewer` 扩展

```cpp
// rendering/PointCloudActor.h
class PointCloudActor
{
public:
    PointCloudActor();
    bool setCloud(const PlyCloud& cloud);   // 空点集返回 false
    void clear();
    bool hasCloud() const noexcept;
    void setVisible(bool visible);
    void setPointSize(double size);
    void bounds(double out[6]) const;
    vtkActor* actor() const noexcept;
private:
    vtkSmartPointer<vtkPoints> m_points;
    vtkSmartPointer<vtkCellArray> m_vertices;      // 手工 vertex cell（无 FiltersGeneral）
    vtkSmartPointer<vtkUnsignedCharArray> m_colors;
    vtkSmartPointer<vtkPolyData> m_polyData;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    vtkSmartPointer<vtkActor> m_actor;
    bool m_hasCloud = false;
};
```
- 顶点 cell 构造：`m_vertices->InsertNextCell(1); m_vertices->InsertCellPoint(i);`
- 有颜色：`SetScalarModeToUsePointData()` + `SetColorModeToDirectScalars()`；无颜色：actor 用统一浅蓝
- `GetProperty()->SetPointSize(2.0)`（DPI 125% 下取 2.0~3.0）、`LightingOff()`

`VTKViewer` 新增（不删除/不修改既有 API）：

```cpp
QSize captureSourceSize() const;                     // m_renderWindow->GetSize()，拉伸前像素尺寸
bool loadPointCloud(const QString& plyPath, QString& error);
void clearPointCloud();
bool hasPointCloud() const noexcept;
void setPointCloudVisible(bool visible);
void setPointCloudPointSize(double size);
void resetCameraToPointCloud();                      // ResetCamera(m_pointCloudActor.bounds())
```
- 构造函数中 `m_renderer->AddActor(m_pointCloudActor.actor())`
- `resetCameraToPointCloud()` 内部 `ResetCameraClippingRange()` → `ResetCamera(bounds6)` → `renderNow()` → `emit cameraChanged(...)`
  （点云与模型并存时按点云取景：`estimated` 模式下点云位于 COLMAP 任意坐标系，与模型不重合，按点云取景才有意义）

### Step 3 — `CameraInfoBuilder` + sidecar 补字段

```cpp
// io/CameraInfoBuilder.h
struct CameraInfoBuildResult
{
    bool built = false;
    int totalImages = 0;
    int posedImages = 0;
    QString cameraMode;    // "provided" | "estimated"
    QString skipReason;    // built == false 时说明原因
};

class CameraInfoBuilder
{
public:
    static QStringList findImages(const QString& directory);   // 支持的图片扩展名，大小写不敏感，返回文件名并排序
    static CameraInfoBuildResult build(const QString& imageDirectory,
                                       const QString& outputJsonPath,
                                       QString& error);
};
```
按上文「CameraInfo 启用规则」与「CameraInfo 数学」实现；图片扩展名集合与 COLMAP 一致：`.jpg .jpeg .png .bmp .tif .tiff .exr .ppm .pgm`。

**[CaptureController.cpp](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/CaptureController.cpp#L266-L291) 的 sidecar 增补**：
在 `camera` 对象之外追加

```json
"render": { "width": <Ww>, "height": <Hw> }
```
- 取值来源：`m_viewer->captureSourceSize()`，在 `writeSidecar()` 内调用（`CaptureShot` 结构不变）
- `schema` 与 `version` **保持 v1 不变**（纯增量字段）；缺该字段的旧 sidecar 按平方像素退化（`fx = fy`）
- 同时更新 `README.md` 中 sidecar 字段说明

### Step 4 — `ReconstructionController`

```cpp
// app/ReconstructionController.h
struct ReconstructRequest
{
    QString imageDirectory;
    QString outputRoot;
    bool useCameraInfo = true;
    bool dense = false;
    QString matcher = QStringLiteral("exhaustive");   // "exhaustive" | "sequential"
};

class ReconstructionController final : public QObject
{
    Q_OBJECT
public:
    explicit ReconstructionController(QObject* parent = nullptr);

    bool isRunning() const noexcept;
    const QString& lastError() const noexcept;
    QString runDirectory() const;

    bool start(const ReconstructRequest& request);   // 返回 false 时 lastError 已填
    void cancel();

signals:
    void started(const QString& runDirectory, const QString& cameraModeHint);
    void progressChanged(int stage, int totalStages, const QString& label);
    void logMessage(const QString& line);
    void finished(const QString& pointCloudPath, int pointCount, const QString& cameraMode, bool dense);
    void failed(const QString& message);

private:
    QString resolveRepoRoot() const;          // 从 applicationDirPath() 上溯 ≤8 层找 core/reconstruction/reconstruct.py
    QString resolvePython(QStringList& prefixArgs) const;
    QString createRunDirectory(const QString& outputRoot) const;
    bool buildArguments(const ReconstructRequest&, QStringList& args, QString& cameraModeHint);
    void pollColmapLog();
    void parseStdout();
    void handleFinished(int exitCode, QProcess::ExitStatus status);
    void cleanup();
    ...
};
```

细节：
- **仓库根解析**：`QSettings("reconstruction/repoRoot")` 优先 → 否则 `QCoreApplication::applicationDirPath()` 上溯最多 8 层探测 `core/reconstruction/reconstruct.py`
- **Python 解析**：`QSettings("reconstruction/pythonInterpreter")` → `QStandardPaths::findExecutable("python")` → `"python3"` → `"py"`；
  命中 `py` 时前置 `-3`；Python 路径出现在 `lastError` 里便于排查
- **启动方式**：`setProgram(python)` + `setArguments(prefix + [reconstruct.py] + args)`（**不拼命令行字符串**）；
  `setProcessChannelMode(QProcess::MergedChannels)`；`setWorkingDirectory(repoRoot)`；
  `QProcessEnvironment` 注入 `PYTHONUNBUFFERED=1`、`PYTHONIOENCODING=utf-8`
- **参数**（不传 `--colmap`、不传 `--input-unit`）：
  `--images <imageDir> --output <runDir>/cloud.ply --workspace <runDir>/workspace [--camera-info <runDir>/camera_info.json] [--matcher sequential] [--dense --gpu]`
  稠密开关同时决定 `--gpu`（与用户选择的"稠密重建(--dense --gpu)"一致；稀疏默认走 CPU）
- **前置校验**：图片目录存在且 `findImages()` ≥ 2；`outputRoot` 可用则 `mkpath`
- **进度**：`QTimer` 400 ms 轮询 `<runDir>/workspace/colmap.log` 的新增字节（记住 `m_logOffset`）；
  对每个新行匹配 `^\s*\$ colmap (\S+)`，按「阶段表」映射为 `(stage, total, label)`；
  顺带把该行以 `logMessage()` 透传到控制台
- **stdout 解析**：逐行累积缓冲，匹配 `point cloud: ` / `points: ` / `camera mode: ` / `manifest: `；
  其余行原样 `logMessage()`（COLMAP 自身输出在 colmap.log，不会刷屏）
- **失败**：退出码非 0 → `lastError` 取最后一条非空输出行（CLI 形态为 `error: <msg>`）→ `failed()`
- **取消**：Windows 上 `python` 的子进程 `colmap.exe` 不随 `kill()` 退出，故
  `cancel()` → 通过 `QProcess::startDetached("taskkill", {"/F","/T","/PID", pid})` 终止进程树，随后 `m_process->kill()` 兜底；
  发 `failed(tr("Reconstruction cancelled"))`；保留 run 目录（内含 `colmap.log`，便于排查）
- 结束时停表、关闭日志文件句柄、`emit finished(...)` / `failed(...)`

### Step 5 — `ReconstructPanel` + `PropertyPanel`

`ReconstructPanel`（沿用 `CapturePanel` 的「只收集值、转发请求」风格，控件带 `objectName` 供 UI 自动化定位）：

| 区域 | 控件 | objectName |
| --- | --- | --- |
| Images | 可编辑 `QLineEdit` + `Browse...` + `Use Latest Capture` | `reconstructImageDir` / `reconstructBrowseButton` / `reconstructUseLatestButton` |
| Output | 可编辑 `QLineEdit` + `Browse...`（默认 `<Pictures>/cadpc_reconstructions`） | `reconstructOutputDir` / `reconstructOutputBrowseButton` |
| Options | `QCheckBox` 使用拍摄位姿与内参（默认勾选） / `QCheckBox` 稠密重建 `--dense --gpu`（默认不勾） / `QComboBox` 匹配器 `Exhaustive`(默认)·`Sequential` | `reconstructUseCameraInfo` / `reconstructDense` / `reconstructMatcher` |
| Run | `Reconstruct`(primary) / `Cancel`(非运行态禁用) | `reconstructRunButton` / `reconstructCancelButton` |
| Progress | `QProgressBar` + 状态 `QLabel`(word wrap) | `reconstructProgress` / `reconstructStatus` |

- 图片目录默认值：`CaptureController::baseDirectory()` 下**字典序最大的子目录**（命名即时间序），存在即预填
- 「Use Latest Capture」按钮重新执行上述解析（面板打开期间又拍了一组时使用）
- 可编辑而非只读：便于粘贴路径，也让 UI 自动化能用 `ValuePattern` 设值
- 公开接口：`imageDirectory()/setImageDirectory()`、`outputRoot()/setOutputRoot()`、`setStatusText()`、`setBusy(bool)`、`setProgress(int value, int maximum)`
- 信号：`reconstructRequested(const ReconstructRequest&)`、`cancelRequested()`
- 面板头 `#include "app/ReconstructionController.h"`（取 `ReconstructRequest`），与 `CapturePanel` 完全同构

`PropertyPanel`：仿 `showCapturePanel` 增加 `showReconstructPanel(bool)` / `isReconstructPanelVisible()` / `reconstructPanel()`
（`m_reconstructPanel` 在构造函数中创建，**默认不加入 tabs**，标题 `Reconstruct`）。

### Step 6 — `Scene` / `ScenePanel`

- `Scene.h`：`enum class SceneNodeType` 增加 `PointCloud`
- `Scene.cpp`：在 `lights` 之外新增分组与节点
  `{"point-clouds", "Point Clouds", Group, "scene"}`、`{"point-cloud-01", "PointCloud", PointCloud, "point-clouds"}`
  （静态节点，与既有 `Camera01`/`Light01` 占位风格一致；不在运行时重建整棵树，避免选中态副作用）
- `ScenePanel.h/.cpp`：新增成员 `QHash<QString, QTreeWidgetItem*> m_items`（`setScene` 时填充）；
  新增 `bool updateNodeLabel(SceneNodeType type, const QString& label)` —— 按 `NodeTypeRole` 找到首个匹配节点改名；
  `updateNodeLabel` 不触发 `nodeSelected`

### Step 7 — `MainWindow` 接线

- `createActions()`：新增
  `m_reconstructToolAction = new QAction(tr("点云重建"), this); m_reconstructToolAction->setCheckable(true);`
  `setToolTip(tr("Reconstruct a point cloud from multi-view images"));`
- [`createToolBar()`](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/src/app/MainWindow.cpp#L190-L211)：
  `[Open Model] | [Capture Image] [点云重建] [几何拟合(禁用)] [AI 修改(禁用)]` ——
  `toolBar->addAction(m_reconstructToolAction);`，占位循环仅剩 `{几何拟合, AI 修改}`
- `createMenus()`：Tools 菜单 = `m_captureToolAction`、`m_reconstructToolAction`
- `connectUi()`：
  - `m_reconstructToolAction->toggled` → `m_propertyPanel->showReconstructPanel(visible)`
  - `ReconstructPanel::reconstructRequested` → `MainWindow::startReconstruction`
  - `ReconstructPanel::cancelRequested` → `ReconstructionController::cancel`
  - 控制器 `started/progressChanged/logMessage` → 面板 `setBusy/setProgress/setStatusText`、控制台、状态栏
  - 控制器 `failed` → `logError` + `setBusy(false)` + `QMessageBox::warning`
  - 控制器 `finished` → `MainWindow::onReconstructionFinished`
  - `ScenePanel::nodeSelected`：新增分支 `PointCloud` → `showReconstructPanel(true)` 并同步勾选 `m_reconstructToolAction`
- 新增槽：
  - `void startReconstruction(const ReconstructRequest& request);`
  - `void onReconstructionFinished(const QString& pointCloudPath, int pointCount, const QString& cameraMode, bool dense);`
    成功时：`m_viewer->loadPointCloud()` → `resetCameraToPointCloud()` → `m_scenePanel->updateNodeLabel(SceneNodeType::PointCloud, QFileInfo(path).fileName() + tr(" (%1 points)").arg(pointCount))`
    → `logInfo(tr("Point Cloud loaded: %1 points").arg(pointCount))`（**该字符串是自动化脚本的断言锚点，措辞固定**）
    → 若 `cameraMode == "estimated"`，追加 `logWarning`：点云位于 COLMAP 任意坐标系（尺度/朝向与 CAD 模型不一致）
- `createDockPanels()`：`propertyDock->setMinimumWidth(350)` → `400`（DPI 125% 下容纳新面板控件），新面板长路径控件加 `setToolTip`

### Step 8 — `CMakeLists.txt`

在 [源列表](file:///d:/code/ECE4500J/apps/cad_pointcloud_client/CMakeLists.txt#L48-L82) 中按分组就近插入 10 项（`app/ReconstructionController.*`、`io/CameraInfoBuilder.*`、`io/PlyReader.*`、`rendering/PointCloudActor.*`、`ui/panels/ReconstructPanel.*`）。
**不新增** `find_package` 组件、**不新增** `target_link_libraries` 条目（QProcess/QSettings 在 Qt6::Core，QProgressBar 在 Qt6::Widgets）。

### Step 9 — 构建与验证

1. 构建：`cmake --build "d:\code\ECE4500J\apps\cad_pointcloud_client\build\msvc-debug" --config Release --parallel 8`
   （Shell 需 `dangerouslyDisableSandbox: true`，windeployqt 要写 `...\Qt\qtlicd\cip.lock`）；要求 0 error，新增文件 0 warning
2. 新建 `build/reconstruct-check.ps1`（UI 自动化，复用 `ui-restructure-check.ps1` 已验证的 `FocusWin`/`DescByType`/`GrabScreen` 基建）：
   - 点工具栏 `点云重建` → 断言 Properties 出现 `Reconstruct` 页、且 `reconstructRunButton` 存在
   - 用 `ValuePattern` 把 `reconstructImageDir` 设为 `D:\Pictures\20260926_180805`
   - 点 `reconstructRunButton` → 轮询控制台文本直到出现 `Point Cloud loaded:`（超时按分钟量级给足，并支持中途截图）
   - 断言 Scene 树中 `PointCloud` 节点文本已变为 `cloud.ply (N points)`
   - `GrabScreen` 留证 + 断言视口中出现点云（与载入前截图做像素差）
   - 可选：取消路径验证（大图目录启动后立即 Cancel，断言回到非运行态且 `colmap.exe` 不残留）
3. 更新 `build/ui-restructure-check.ps1`：`$STAGE_NAMES` 改为 4 个（`点云重建` 启用 + 2 个禁用），启用数断言 `2 → 3`
4. **PlyReader 单测素材**（仓库已有真实产物，无需重跑 COLMAP）：
   - 读 `outputs/fountain_p11_sparse.ply` → 期望 **13668** 顶点、有颜色
   - 读 `outputs/fountain_p11_dense.ply` → 期望 **1632443** 顶点、有颜色（同时验证大点云渲染与解析耗时）
   - 用 app 载入二者之一，确认视口渲染正常（可加一个临时环境变量/命令行开关让 app 启动时载入指定 PLY；若不引入，则用 UI 自动化替代：直接以 `ReconstructionController` 的结果载入）
5. **约定交叉校验**：Step 0 的 Python 参考实现对 8 张 sidecar 生成的 `camera_info.json`，与 app 的 `CameraInfoBuilder` 产物做逐字段比对（数值容差 ~1e-9）
6. 回归：建模流程（`Rebuild succeeded | 4 features`）、拍摄流程（8 张环绕 + sidecar + manifest）不应受影响；
   sidecar 新增 `render` 字段后重新拍一组，确认 `ui-restructure-check.ps1` 中的 sidecar 断言仍通过
7. **候选补充素材（estimated 模式端到端）**：`data/fountain-p11/images`（11 张 6 MB 真实纹理图，本机既有产物证明该数据集在此机器上可成功重建）。
   该目录只读，输出写到 `build/reconstruct-check/` 下的临时目录。因全分辨率穷举匹配耗时较长，标记为**可选/慢速**验证。
8. `git status` 复核：所有改动必须落在 `apps/` 内；`core/`、`data/`、`outputs/`、`configs/` 零改动

---

## 假设与决策

| # | 决策 | 依据 |
| --- | --- | --- |
| 1 | 复用 `core/reconstruction/` CLI，不在 C++ 里重写 COLMAP 编排 | 编排层已存在且是纯标准库，重写无收益 |
| 2 | 不传 `--colmap`（依赖 pipeline 自解析 `deps/colmap/bin/colmap.exe`） | 避免把不透明依赖的路径硬编码进 app |
| 3 | 稠密开关同时开启 `--gpu`；稀疏默认纯 CPU | 与用户选择的 `--dense --gpu` 措辞一致；已实测本机 COLMAP 支持 CUDA |
| 4 | 匹配器默认 `Exhaustive` | 与 CLI 默认一致；8 张环绕图仅 28 对，开销可忽略；`Sequential` 留给视频序列 |
| 5 | CameraInfo 仅在"全目录都有透视 sidecar 且 ≥2 张"时启用，否则完全不传 | 规避 pipeline 对 `cameras.width/height` 与实际像素尺寸的严格校验；与用户选定语义一致；故意不使用未充分验证的 partial/seed 分支 |
| 6 | 位姿输出 `camera_to_world`（而非 `world_to_camera`/裸 `qvec`） | 转换公式最少、符号出错面最小，与 sidecar 的「相机中心 + 目标点」语义一一对应 |
| 7 | sidecar 增量补 `render {width,height}` 且 `version` 保持 1 | 内参 `fx` 必需；纯增量字段，旧 sidecar 可退化处理 |
| 8 | 自写 PLY 解析，不引入 VTK IOPLY | vendored VTK 无该模块（已实测确认）；顺带避开 `FiltersGeneral`/`vtkVertexGlyphFilter` 缺失 |
| 9 | Vertex cell 手工构造 | 链接清单无 `FiltersGeneral`，`vtkVertexGlyphFilter` 不可用 |
| 10 | Scene 中的 Point Clouds 节点**静态存在**，载入后只改标签 | 避免运行时重建整棵树带来的选中态副作用；与 `Camera01`/`Light01` 占位风格一致 |
| 11 | 载入后 `ResetCamera(点云 bounds)` 而非全体可见 actor | 与模型共存时点云可能位于另一坐标系（estimated），按点云取景才有意义 |
| 12 | 点云解析在 UI 线程同步执行 | 不引入 QtConcurrent 依赖；稀疏点云（万级）可忽略，稠密点云（百万级）会有可感知停顿，作为已知限制记录 |
| 13 | 取消用 `taskkill /F /T /PID` 终止进程树 | Windows 上 `QProcess::kill()` 不会终止 `colmap.exe` 子进程 |
| 14 | 规范化 run 目录：`<root>/<yyyyMMdd_HHmmss>/`，同秒加 `-2` | 与 `CaptureController::createGroupDirectory()` 同策略；且天然满足"output 不存在、workspace 为空"的硬约束 |
| 15 | ReconstructPanel 的路径框**可编辑** | 便于粘贴路径；也是 UI 自动化设值的必要前提 |

---

## 验证清单

- [x] Step 0 无头验证：`camera mode: provided`、`points > 0`、包围盒尺度合理（DemoModel 0 点已如实记录，见「实施记录」）
- [x] `PlyReader` 正确读出 `fountain_p11_sparse.ply` = 13668 点、`fountain_p11_dense.ply` = 1632443 点，颜色存在
- [x] `CameraInfoBuilder` 产出与 Python 参考实现逐字段一致（容差 1e-9，实测最大偏差 5.6e-17）
- [x] 工具栏 5 个按钮：`Open Model`/`Capture Image`/`点云重建` 启用，`几何拟合`/`AI 修改` 禁用，无数字前缀
- [x] Tools → 点云重建 可勾选，Properties 出现/移除 `Reconstruct` 页，参数在隐藏/再显示间保留
- [x] 设置图片目录 → 运行 → 控制台出现 `Point Cloud loaded: N points`，Scene 树 `PointCloud` 节点标签更新
- [x] 视口点位可见且自动适配视图；`Render`/`Camera`/`Object` 面板与点云共存无异常
- [x] 失败路径：目录错误 / 图片不足 / 空点云（CLI exit 2）→ 面板与控制台显示 Core 错误原文，不崩溃
- [x] 取消路径：运行中 Cancel 后 UI 回到非运行态，无 `colmap.exe`/`python.exe` 残留
- [x] Release 全量构建 0 error、新文件 0 warning
- [x] `ui-restructure-check.ps1` 更新后全部通过；拍摄/建模回归通过
- [x] `git status` 确认改动全部位于 `apps/` 内

---

## 风险与已知限制

| 风险 | 影响 | 应对 |
| --- | --- | --- |
| DemoModel 是平光低纹理立方体，SIFT 特征可能不足 | `point_count == 0` → CLI exit 2 | Step 0 先暴露；如实记录；真实用途（有纹理物体）不受影响；用户可在 Modeling 面板增加特征后再拍 |
| 外参转置/符号写错 | 位姿错 → 三角化失败或几何错乱 | `points > 0` + 包围盒尺度量级 + 重投影误差三重断言；Step 0 先行 |
| `captureSourceSize()` 与 `vtkWindowToImageFilter` 实际取图尺寸不一致 | `fx` 偏差 | 二者都反映渲染窗口像素尺寸；用「app 生成的 camera_info 与 Python 参考实现一致」+ 重投影误差兜底；若仍偏差，退化为 `fx = fy` |
| 稠密重建显存/耗时 | 可能数分钟或 OOM | 默认关闭；进度条 + 可取消；失败信息透传 |
| 稠密 PLY 百万级点的同步解析 | UI 短暂停顿 | 记录为已知限制；控制台打印点数让用户有预期 |
| `estimate` 模式点云坐标系与 CAD 模型不一致 | 视觉上"错位" | 自动适配视图按点云取景 + `logWarning` 明确提示 |
| DPI 125% 下 Properties 面板挤压 | 控件截断 | `propertyDock` 最小宽 350→400、长路径加 tooltip |
| 目标目录/输出目录含中文或空格 | 传参失败 | `setProgram`+`setArguments` 传参（不经 shell）；`PYTHONIOENCODING=utf-8` 且按 UTF-8 解码输出 |

---

## Non-goals（不做的事）

- 不修改 `core/`（含 `core/reconstruction/`）、`configs/`、`data/`、`docs/`、`outputs/`、`pipeline/` 下任何文件
- 不新增 COLMAP 参数透传界面（峰值阈值、`estimate_affine_shape` 等调参入口不在本次范围）
- 不做点云后处理（下采样/去噪/网格化）、不做点云与 CAD 模型的对齐/配准
- 不实现 pipeline 的 partial-pose（seed + `Mapper.fix_existing_frames`）路径
- 不改动 `captureImage()` 的拉伸行为（保持既有已验证的拍摄语义）
- 不把 `几何拟合` / `AI 修改` 变成可用功能

---

## 实施记录

实施于 2026-09-26。全部改动落在 `apps/` 内；`core/`、`configs/`、`data/`、`docs/`、`outputs/`、`pipeline/` 及仓库根文件零改动（`git status --porcelain` 复核）。

### Step 0 — 无头可行性验证结论

| 素材 | 相机模式 | 结果 |
| --- | --- | --- |
| DemoModel 8 张环绕渲染图（`D:\Pictures\20260926_180805`） | `provided` | **失败：0 点**。`sparse/triangulated/points3D.bin` 仅 8 字节、`seed_model/points3D.txt` 0 字节、`cloud.ply` 185 字节（只有 header）；CLI exit 2 |
| `outputs/fountain_p11_dense_workspace/images`（8 张真实纹理图，位姿由 COLMAP 自身反解后回灌为 sidecar） | `provided` | 成功：`point_count = 16293`，exit 0，`camera mode: provided` |
| `data/fountain-p11/images`（11 张真实纹理图，无 sidecar） | `estimated` | 成功，exit 0 |

**关于 DemoModel 的 0 点，如实记录如下**：该次运行 8 张图全部被 COLMAP 正常注册（`images.bin` 12184 字节 = 8 个位姿），即位姿与内参在本约定下被 COLMAP 接受；但三角化产出 0 点。原因是立方体为平光低纹理，SIFT 特征不足 —— 与「风险与已知限制」第 1 条预测一致。**未做任何调参，也未替换素材来"凑过"该断言**，而是按预案把端到端成功路径改用 `data/fountain-p11/images`。

`provided` 模式的数学约定（`camera_to_world` 的语义、`qvec` 的 world-to-camera 关系、`fx/fy` 的拉伸比）由 `build/headless-reconstruction-check.py` 的位姿往返断言先行锁定：由 `camera_to_world` 反算 `world_to_camera` 再与 COLMAP 原始 `qvec/tvec` 比对，误差 **1.1e-16**，证明外参转置与符号无误。

### Step 1–8 — 实现

按计划新增 5 组文件（`io/PlyReader`、`io/CameraInfoBuilder`、`rendering/PointCloudActor`、`app/ReconstructionController`、`ui/panels/ReconstructPanel`），修改 `VTKViewer`、`CaptureController`、`Scene`、`ScenePanel`、`PropertyPanel`、`MainWindow`、`CMakeLists.txt`、`README.md`。未新增 VTK 模块、Qt 组件或 `target_link_libraries` 条目。Release 全量构建 **0 error、0 warning**（含全部新增文件）。

### Step 9 — 验证结果

三个断言脚本全部通过。

**`build/ui-restructure-check.ps1`（按 Step 9.3 更新后）→ `ALL CHECKS PASSED`**
- 工具栏 5 个按钮、启用 3 个（`Open Model`/`Capture Image`/`点云重建`），`几何拟合`/`AI 修改` 禁用；分隔线为 `ControlType.Group`；View 菜单 7 项
- 拍摄回归：8 张环绕 + sidecar（含新增 `render {width,height}`）+ manifest 齐全
- 建模回归：`Rebuild succeeded | 4 features`

**`build/reconstruct-check.ps1`（Step 9.2 新建）→ `ALL CHECKS PASSED`**，10 段断言：
- `data/fountain-p11/images`（11 张图、0 个 sidecar）→ `estimated` 稀疏重建成功：`Point Cloud loaded: 14100 points`；`cloud.ply` 211689 字节、header 声明 14100 顶点，与控制台点数一致
- 目录无可用的拍摄元数据 → 按设计**未传** `--camera-info`（无 `camera_info.json` 产出），控制台给出尺度提示 `[WARNING] Camera poses were estimated by COLMAP; the point cloud scale is arbitrary ...`
- Scene 树节点更新为 `cloud.ply (14100 points)`；载入前后视口平均亮度差 **24.2033**（点云确实渲染出来）
- 失败路径：目录不存在 → 模态框 + 控制台 `[ERROR] Image directory does not exist: ...`（Core 错误原文），面板回到空闲态
- 取消路径：运行中 Cancel → 面板回空闲、`colmap.exe` 残留 0、`python.exe` 回到基线

**PlyReader（Step 9.4）**：临时 harness 置于 `build/io-harness/`（在 `build/` 内，未改动交付用 `CMakeLists.txt`）
- `outputs/fountain_p11_sparse.ply` → 13668 顶点、**3 ms**、颜色齐全
- `outputs/fountain_p11_dense.ply` → 1632443 顶点、**59 ms**、颜色齐全
- 结论：决策 12「稠密 PLY 百万级点的同步解析 → UI 短暂停顿」的风险实际可忽略（59 ms）

**约定交叉校验（Step 9.5）**：`CameraInfoBuilder` 与 Step 0 的 Python 参考实现逐字段比对，容差 1e-9，全部通过

| 素材 | 关键前提 | 结果 |
| --- | --- | --- |
| `D:\Pictures\20260926_192417` | 含 `render` 字段：`render` 733×853 → `output` 1920×1080，故 `fx = 2.0688 × fy` | `params` 偏差 0、`qvec` 最大 **5.6e-17**、`position` 偏差 0 |
| `D:\Pictures\20260926_180805` | 无 `render` 字段 → 退化为方形像素 `fx == fy` | 同上，全部一致 |

第二行同时验证了「旧 sidecar 缺 `render` 时退化处理」这条决策 7 的路径。

### 与计划的偏离（3 处，均为实现中暴露的真实缺陷）

| # | 偏离 | 原因 |
| --- | --- | --- |
| 1 | 取消：`startDetached(taskkill)` + 立即 `QProcess::kill()` → `QProcess::execute(taskkill)` 走完再 `kill()` | 原顺序存在竞态：Python 比 taskkill 开始遍历父子链还早退出，`colmap.exe` 被孤儿化并残留（实测残留 1 个）。taskkill 必须在 Python 仍存活时才能走完进程树。决策 13 的意图（用 taskkill 终止进程树）不变 |
| 2 | `ReconstructPanel::latestCaptureDirectory()`：取目录名字典序最后一个 → 只匹配 `^\d{8}_\d{6}(-\d+)?$` | 拍摄根目录是用户的「图片」文件夹，其中 `Camera Roll`/`Screenshots` 排在同一批时间戳目录之后，原实现实测会选到 `Screenshots`。修复后正确解析到 `D:/Pictures/20260926_192417` |
| 3 | 取消时不弹模态 `QMessageBox`（控制器新增 `wasCancelled()`，`MainWindow` 仅记 `logWarning`） | 取消是用户主动行为而非故障，弹窗既打断用户，也会卡住 UI 自动化 |

另修正两处实现细节：
- `PointCloudActor` 的 `vtkCellArray::Allocate` 弃用警告（`C4996`）→ 改 `AllocateEstimate`
- 两个自动化脚本自身的 4 个缺陷：Qt 把可勾选 `QToolButton` 暴露为 `ControlType.CheckBox`（漏枚举 → 工具栏只数到 4 个）；`ToolbarButtons` 的数组嵌套；`FindTopWindowByName` 用 `-match` 配通配符（前导 `*` 是非法正则，对每个窗口都抛异常 → 模态框永远找不到）；Qt 的模态框在 UIA 树中嵌在主窗口下而非桌面根

### 已知限制（保留）

- 低纹理/平光模型（如 DemoModel 立方体）拍出的图片仍可能产出 0 点 → CLI exit 2，面板与视口如实报错。该功能对真实带纹理物体有效。
- `estimated` 模式的点云与 CAD 模型不在同一坐标系，视觉上可能错位；载入后按点云 bounds 取景，并在控制台给出警告。
- 稠密模式（`--dense --gpu`）未做端到端 UI 验证（耗时较长，计划中已标注为可选/慢速）；稀疏路径已完整验证。
- `estimated` 模式的点数在多次运行间会有小幅波动（实测 14088 / 14100 / 14102），源于 COLMAP 的随机化三角化，故自动化断言只锚定固定字符串格式 `Point Cloud loaded: N points`，不锚定具体 N。