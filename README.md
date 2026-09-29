# Bochu Capstone：多视角图像到可编辑参数化 CAD

Bochu Capstone（桌面客户端名为 **ParamCAD Studio**）希望把零件的多视角图像逐步转换为可编辑的参数化 CAD，而不是只生成一个不可编辑的三角网格。

项目当前已经打通一条可验证的基础链路：

```text
STEP 模型 / 真实零件
        ↓
虚拟拍摄 / 现实拍摄得到的多视角图片
        ↓
COLMAP 稀疏或稠密点云（PLY）
        ↓
点云预处理 + 解析曲面检测
        ↓
长方体与贯穿孔识别
        ↓
参数化 Feature History + OpenCASCADE B-Rep
        ↓
ParamCAD Studio 中显示和继续编辑
```

最终目标还包括使用自然语言指导局部修改和重新拟合。**文本指导修改目前尚未实现**；当前可自动落地到参数化 CAD 的零件类型也主要是长方体及其贯穿孔，而不是任意机械零件。

## 1. 项目目标与设计思路

项目要解决的核心问题是：如何将图像和点云中的“观测结果”，转换为 CAD 中有明确类型、尺寸、位置和建模语义的特征。

为此，仓库将流程拆成相互独立的几层：

- 图像重建层只负责从图片生成点云，不判断零件是什么；
- 几何重建层保存点、法向、曲面证据和置信度，并识别参数化特征；
- 参数化建模层只接收结构化特征和命令，负责生成可靠的 B-Rep；
- 桌面客户端负责交互、流程编排、文件读写和可视化；
- 未来的文本指导层只应把自然语言转换成结构化修改指令，不直接操作点云或 B-Rep。

这使 COLMAP、CGAL、OpenCASCADE、VTK 等第三方实现都被限制在各自的适配边界内，层与层之间传递项目自己的数据类型。

## 2. 当前端到端流程

### 2.1 多视角拍摄

当前实现的“拍摄”主要是 **ParamCAD Studio 内的虚拟拍摄**，用于生成稳定、可复现的测试数据；项目也可以直接读取现实相机拍摄的图片，但尚未集成相机硬件控制。

虚拟拍摄流程如下：

1. 通过 `File → Open Model` 导入 STEP/STP 模型；
2. `StepModelLoader` 使用 OpenCASCADE 读取并保留原始 B-Rep；
3. 模型按体积质心（无封闭体时按包围盒中心）平移到原点，方便相机围绕物体旋转；
4. `OcctShapeTessellator` 将 B-Rep 临时离散为 VTK 显示网格，原始 B-Rep 不会被替换；
5. 可选地给模型投影纹理，以增加 COLMAP 可匹配的视觉特征；
6. `CaptureController` 控制 VTK 相机，从多个方向渲染 PNG；
7. 同时保存相机位置、朝向、视场角、投影方式和输出尺寸等元数据。

客户端的一键球面采样共生成 62 个视角：在纬度 `-60°、-30°、0°、30°、60°` 上各拍 12 张，再拍正上方和正下方各 1 张。也可以只拍当前视角，或在界面中自定义拍摄参数。

典型输出为：

```text
outputs/captures/<时间戳>/
├── shot_000.png
├── shot_000.json       # 单张图片的相机元数据
├── ...
└── manifest.json       # 整组拍摄的聚合清单，以 version 2 为准
```

`CameraInfoBuilder` 会在开始重建前，把透视相机元数据转换为 COLMAP 使用的 `PINHOLE` 内参和外参。若图片来自现实拍摄、元数据缺失或包含正交投影视角，客户端会放弃这些先验，让 COLMAP 自行估计相机。

这一阶段的主要依赖是：

- **Qt 6 Widgets**：拍摄面板、参数输入和流程控制；
- **VTK 9.7.0**：视口、相机控制、离屏截图和模型显示；
- **OpenCASCADE 8.0.1**：STEP 读取和 B-Rep 保存；
- 项目自己的轨道相机、拍摄清单和相机参数转换代码。

### 2.2 从图片生成点云

点云重建实现在 `core/reconstruction/`，是一个无界面的 Python 调度层。客户端通过 `QProcess` 启动该脚本，脚本再调用本地的 COLMAP 4.2.0 命令行程序。

流程分为以下步骤：

1. `feature_extractor` 提取每张图片的 SIFT 特征；
2. `exhaustive_matcher` 对普通多视角图片做穷举匹配，或用 `sequential_matcher` 处理按顺序采集的帧；
3. 根据相机信息选择重建方式：
   - 没有外参：由 `mapper` 估计相机并完成 SfM；
   - 全部外参已知：固定给定位姿，用 `point_triangulator` 三角化；
   - 只有部分外参：先三角化种子模型，再固定已有位姿注册其余图片；
4. 稀疏模式用 `model_converter` 导出彩色 PLY；
5. 稠密模式继续运行 `image_undistorter`、`patch_match_stereo` 和 `stereo_fusion`，生成稠密融合点云；
6. 写入 `reconstruction.json`，记录输入、点数、相机模式、坐标系和尺度单位。

典型输出为：

```text
outputs/reconstructions/<时间戳>/
├── cloud.ply
├── camera_info.json           # 使用虚拟拍摄元数据时生成
└── workspace/
    ├── database.db
    ├── sparse/ 或 dense/
    ├── colmap.log
    └── reconstruction.json
```

没有相机外参时，SfM 点云只有相对尺度，清单中的单位为 `arbitrary`；使用带毫米平移量的虚拟相机外参时，点云继承毫米尺度。后续生成 CAD 前，未知尺度必须由用户明确确认，不能被静默当作毫米。

这一阶段的主要依赖是：

- **Python 3**：仅使用标准库编排流程、校验 CameraInfo、更新 COLMAP 数据库和记录清单；
- **COLMAP 4.2.0**：特征提取、匹配、SfM、三角化和稠密重建；
- **CUDA 与支持的 NVIDIA GPU（仅稠密模式）**：当前客户端启用稠密模式时会同时启用 GPU；
- **Qt `QProcess`**：桌面端启动、取消和监控重建进程。

命令行也可以独立运行，具体参数见 [`core/reconstruction/README.md`](core/reconstruction/README.md)。

### 2.3 点云预处理、曲面检测与特征拟合

在客户端中加载 PLY 后，`Reconstruct CAD` 会进入 `core/geometric_reconstruction/`。该模块不依赖 Qt 或 VTK，输入和输出均为项目自有数据类型。

当前流程为：

1. `PlyReader` 读取点坐标、颜色以及可选的 `nx/ny/nz` 法向；
2. 从相邻的 `reconstruction.json` 读取单位；米制输入先转换为毫米；
3. 使用体素降采样减少重复点，并保留可追踪的原始 `PointId`；
4. 使用 K 近邻平均距离做统计离群点过滤；
5. 对缺失法向的点做局部 PCA 法向估计，同时计算曲率；
6. CGAL Efficient RANSAC 在同一次检测中竞争性地寻找平面、圆柱、球、圆锥和圆环面；
7. 将第三方检测结果转换为项目自己的 `PrimitiveEvidence`，保存参数、支持点、残差、覆盖率和置信度；
8. 对平面做 PCA 精修和重复层抑制，对圆柱、球的半径使用全部支持点精修；
9. 从六个平面中寻找三组互相垂直的平行面，恢复带位置和朝向的长方体；
10. 检查圆柱是否与长方体轴对齐、位于边界内、具有足够角度覆盖并贯穿两个相对外表面；满足条件时识别为贯穿孔；
11. 将“长方体 + 贯穿孔”作为一个原子 `ModelPatch` 提交给参数化建模核心。

这里需要区分两种能力：

- **曲面检测**已经支持平面、圆柱、球、圆锥和圆环面；
- **完整 CAD 特征识别**目前只完成了完整长方体和长方体上的贯穿孔。

因此，检测到球面或圆锥面不等于已经能够自动生成对应的完整 CAD 零件。当前长方体识别还要求获得恰好六个有效平面候选；缺面、遮挡、复杂组合体和多零件分割仍是后续工作。

这一阶段的主要依赖是：

- **CGAL 6.2.1 Shape Detection**：Efficient RANSAC 解析曲面提议；
- **Eigen3**：CGAL 及数值计算支持；
- **OpenCASCADE 8.0.1**：通过下游建模核心验证并生成实体；
- 项目自己的 `PointStore`、预处理、证据类型、长方体识别和贯穿孔识别代码。

> CGAL Shape Detection 采用 GPL-3.0-or-later 或商业许可。若要分发闭源版本，需要先确定合适的许可方案。

算法边界、测试方法和后续计划见 [`core/geometric_reconstruction/README.md`](core/geometric_reconstruction/README.md)。

### 2.4 参数化建模与显示

`core/parametric_modeling/` 保存的是参数化特征历史，而不是一次性生成后就丢失语义的网格。

当前建模核心支持：

- 点、线、矩形、圆等草图实体；
- 拉伸的新建、合并、切除和求交；
- 普通切除与贯穿切除；
- 基准面、偏移基准面和显式平面；
- 直接语义化的长方体与贯穿孔特征；
- 特征抑制、依赖查询、安全/级联删除；
- 带 revision 检查的原子 `ModelPatch`；
- 从 Feature History 重建 OpenCASCADE B-Rep；
- 核心层 STEP 导出。

自动拟合成功后，`ModelPatch` 会一次性写入长方体和孔特征；任一命令或重建失败时回滚。客户端随后将新的 B-Rep 三角化并交给 VTK 显示。当前桌面端尚未接通 STEP 导出入口，持久化 Feature Graph、稳定拓扑命名和完整 undo/redo 也未完成。

这一阶段的主要依赖是：

- **OpenCASCADE 8.0.1**：草图轮廓、拉伸、布尔运算、B-Rep 和 STEP；
- **VTK 9.7.0**：只负责显示由 B-Rep 派生出的网格；
- **Qt 6 Widgets**：建模面板、属性编辑和命令编排。

建模核心的详细能力见 [`core/parametric_modeling/README.md`](core/parametric_modeling/README.md)。

### 2.5 文本指导修改（未实现）

预期的文本指导流程是：

```text
用户文本 + 当前参数化特征 + 当前选择对象
                    ↓
          文本解析 / LLM Agent
                    ↓
     结构化 ReconstructionCommand / ModelPatch
                    ↓
       参数校验、局部重拟合或特征修改
                    ↓
          ModelingCore 原子重建 B-Rep
```

例如“把这个孔的直径改为 8 mm”应先被转换成带目标特征 ID 和数值参数的结构化命令，再由建模核心校验和执行。LLM 不应直接生成或修改 OpenCASCADE Shape。

当前仓库中尚无文本输入面板、LLM/Agent 接入、结构化命令协议或 SolidWorks MCP 集成，也没有相关运行时依赖。现阶段只能通过已有界面手动修改部分草图和建模参数，或由几何重建模块提交 `ModelPatch`。

## 3. 仓库层级架构

```text
Bochu_Capstone/
├── apps/
│   └── ParamCAD Studio/          Windows 桌面客户端
├── core/
│   ├── reconstruction/           图片 → PLY 点云的 Python/COLMAP 流程
│   ├── geometric_reconstruction/ PLY → 曲面证据 → 参数化特征
│   └── parametric_modeling/       参数化 Feature History → OCCT B-Rep/STEP
├── data/                          示例数据与性能验证样例
├── docs/                          架构、实现计划和依赖安装说明
├── outputs/                       默认运行输出目录
├── tools/                         输出清理、性能验证等辅助脚本
├── packaging/                     Windows full/bootstrap 打包与安装脚本
└── deps/                          本地二进制 SDK/运行时，不提交到 Git
```

各部分职责如下：

| 部分 | 主要职责 | 不负责的内容 |
| --- | --- | --- |
| `apps/ParamCAD Studio` | Qt 界面、控制器、文件 I/O、VTK 视口、各核心模块编排 | 不在 UI 中实现几何算法 |
| `core/reconstruction` | 调用 COLMAP，把图片和可选相机信息转换为点云 | 不解释点云中的 CAD 语义 |
| `core/geometric_reconstruction` | 点云预处理、解析曲面证据、长方体/贯穿孔识别、生成 `ModelPatch` | 不依赖 Qt/VTK，不直接维护界面状态 |
| `core/parametric_modeling` | 保存特征历史、执行建模命令、重建 B-Rep、STEP 导出 | 不读取图片或运行点云检测 |
| `packaging` | 收集客户端 DLL、Python 和 COLMAP，生成 Windows 包 | 不参与算法运行 |
| `deps` | 放置固定版本的本地 SDK 和运行时 | 不是项目源码，不应递归扫描或提交 |

桌面客户端内部再按职责分为：

- `src/app`：拍摄、重建、建模等流程控制器；
- `src/ui`：各功能面板和视口交互；
- `src/io`：STEP、PLY、CameraInfo 等边界适配；
- `src/rendering`：B-Rep/点云/草图到 VTK actor 的显示适配；
- `src/core`：客户端自身的场景、相机和草图坐标逻辑。

## 4. 主要技术栈与依赖关系

| 依赖 | 当前版本/要求 | 使用位置 | 作用 |
| --- | --- | --- | --- |
| C++ | C++17 | 客户端、几何重建、参数化建模 | 主体实现 |
| MSVC | Visual Studio 2022 x64 | Windows 构建 | 保证 Qt/VTK/OCCT ABI 一致 |
| CMake | 3.24+ | C++ 工程 | 配置、构建、测试和 DLL 收集 |
| Qt | 6.5+，MSVC x64 | `apps/ParamCAD Studio` | Widgets UI、进程管理、JSON/文件处理 |
| VTK | 9.7.0 | 客户端 | 三维显示、相机交互、截图 |
| OpenCASCADE | 8.0.1 | 客户端、建模核心 | STEP、B-Rep、几何和布尔运算 |
| CGAL | 6.2.1 | 几何重建 | Efficient RANSAC 曲面检测 |
| Eigen3 | vcpkg 包 | 几何重建 | 线性代数支持 |
| Python | 3.x；发布包固定 3.13.10 | 图像重建 | 无界面流程调度，仅依赖标准库 |
| COLMAP | 4.2.0 | 图像重建 | SfM/MVS 与 PLY 输出 |
| CUDA | 可选 | COLMAP 稠密重建 | PatchMatch Stereo 加速 |

本地 VTK、OpenCASCADE 和 COLMAP 放在 `deps/` 下；CGAL 与 Eigen3 通过 vcpkg 提供。所有 C++ 二进制依赖必须使用兼容的 **MSVC 2022 + x64 + Release + dynamic CRT** 组合，不能混用 MinGW 构建。

## 5. 当前项目进度

| 能力 | 状态 | 说明 |
| --- | --- | --- |
| ParamCAD Studio 桌面框架与 VTK 视口 | 已完成基础版 | 支持场景、属性、相机、渲染、建模和日志面板 |
| STEP/STP 导入 | 已完成 | 保留 OCCT B-Rep，并生成显示网格 |
| 虚拟多视角拍摄 | 已完成 | 单张、自定义环绕和 62 视角球面采样，带完整相机元数据 |
| 现实照片输入 | 已支持 | 可直接选择图片目录；未集成相机硬件采集 |
| COLMAP 稀疏点云 | 已完成 | 支持未知、部分已知和全部已知相机参数 |
| COLMAP 稠密点云 | 已完成基础版 | 需要支持的 GPU/CUDA 环境 |
| 点云读取与尺度处理 | 已完成基础版 | 支持 PLY 法向、单位清单和未知尺度确认 |
| 点云预处理 | 已完成 | 体素降采样、统计离群点过滤、PCA 法向与曲率 |
| 五类解析曲面检测 | 已完成基础版 | 平面、圆柱、球、圆锥、圆环面 |
| 长方体拟合 | 已完成基础版 | 支持带朝向的完整六面长方体 |
| 贯穿孔识别 | 已完成基础版 | 从圆柱证据识别长方体上的轴向贯穿孔 |
| 通用机械零件特征恢复 | 进行中 | 缺面、遮挡、多零件、盲孔、凸台、口袋等尚未完成 |
| 参数化建模核心 | 已完成 V0 | 草图、拉伸、切除、长方体、贯穿孔、原子补丁与重建 |
| 客户端 STEP 导出 | 未接通 | 核心已有导出能力，桌面端暂无操作入口 |
| 文本/LLM 指导修改 | 未开始 | 无 UI、Agent、命令协议和模型服务接入 |
| SolidWorks MCP | 未实现 | 当前实际 CAD 后端为项目内的 OpenCASCADE 建模核心 |
| Windows 打包 | 已有基础流程 | 支持包含全部运行时的 full 包和按需下载依赖的 bootstrap 包 |

当前最重要的后续工作是：处理不完整可见面的点云、对象分割、primitive 鲁棒精修与全局几何约束，扩展盲孔/凸台/阶梯轴/口袋等特征，然后在稳定的结构化命令与 Feature Graph 基础上接入文本指导修改。

## 6. 第一次进入仓库时从哪里开始

**第一次克隆后，请先按 [`docs/development_setup.md`](docs/development_setup.md) 完成开发环境安装、依赖检查、Release 编译和客户端启动。**

1. 想运行桌面客户端：先阅读 [`apps/ParamCAD Studio/README.md`](apps/ParamCAD%20Studio/README.md)。
2. 想了解图片如何变成点云：阅读 [`core/reconstruction/README.md`](core/reconstruction/README.md)。
3. 想开发点云拟合与特征识别：阅读 [`core/geometric_reconstruction/README.md`](core/geometric_reconstruction/README.md)。
4. 想开发草图、特征和 B-Rep：阅读 [`core/parametric_modeling/README.md`](core/parametric_modeling/README.md)。
5. 想查看本地二进制依赖的目录约定：阅读 [`deps/README.md`](deps/README.md) 和 [`docs/colmap_installation.md`](docs/colmap_installation.md)。
6. 想制作或安装 Windows 发布包：阅读 [`packaging/PACKAGE_README.md`](packaging/PACKAGE_README.md)。

在仓库根目录可以先执行非递归依赖检查：

```powershell
& .\deps\verify.ps1
```

开发时请把运行结果留在 `outputs/`，不要提交 `deps/` 下的大型 SDK、COLMAP 工作区或本机构建产物。
