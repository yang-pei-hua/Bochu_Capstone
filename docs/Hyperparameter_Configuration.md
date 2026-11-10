# 客户端超参数外置与模块 CLI

本文记录 `apps/ParamCAD Studio` 超参数外置的当前实现、配置口径、命令行入口，以及已经执行和尚未执行的验证。实现目标是把原先散落在 `*Options` 结构体和客户端代码里的默认值集中到 `config/*.conf`，算法本身不改变。

## 1. 配置文件

配置文件位于 `apps/ParamCAD Studio/config/`，构建或安装后复制到可执行文件旁的 `config/`。每个模块一份，模块内部按二级标题分组：

| 模块 | 文件 | 当前键数 | 二级分组 |
| --- | --- | --- | --- |
| 拍摄 | `capture.conf` | 16 | 输出、轨道环 |
| 点云预处理 | `preprocessing.conf` | 13 | 降采样、离群过滤、法向与曲率、单位 |
| 曲面检测 | `detection.conf` | 67 | 通用、多尺度、尺度自适应、平面、圆柱、球、圆锥、圆环 |
| 曲面关系 | `analysis.conf` | 31 | 连接、关系推断、关系优化、局部恢复、界面默认容差 |
| 常规识别 | `recognition.conf` | 122 | 分析配额、箱体、通孔、曲面实体、组合体、混合特征、引导模板 |
| 装配分解 | `assembly.conf` | 43 | 采样、容差、面片与闭合、局部恢复、投影回退 |
| 投影拟合 | `projection.conf` | 89 | 输入与候选轴、端面与草图、两轴求交、孔与分段、评分与搜索 |

配置文件支持 `#` 注释、空行、UTF-8 BOM。未知键、重复键、越界值和跨字段冲突都会在加载时报错，并带上文件名与行号。

### 1.1 本轮补齐的内部常量

第一版只外置了已有 `*Options` 字段和客户端尺度系数。本轮把原先写在实现里的质量门槛、采样预算和尺度倍率补进了配置：

- **混合特征**：宿主面判定、孔壁内向比例、已知曲面判定、连通距离、局部点数、端盖、凸台分箱与覆盖率、矩形切槽壁面与跨度。
- **引导模板**：模板最低点数、箱体模板支持点、边界容差与法向余弦、支持比例、局部关系倍率、曲面扩展端面/侧壁阈值、复验匹配比例。
- **曲面实体**：端面检测、拟合判定倍率、锥顶观测、边界分箱、圆柱支持点、圆锥起点、残差评分权重。
- **组合体**：最低点数、方向/层支持点、框架与层合并倍率、正交方向、边界层数、斜切面数与覆盖率、材料单元采样数。
- **装配**：部件面片上下限、闭合预算、焊接/连接容差、闭合采样范围、曲面闭合覆盖率、冲突门槛、局部恢复的父部件/轮廓/样本门槛、配对数量。
- **曲面关系**：相邻/派生置信度、人工关系重叠比例、优化尝试与活跃曲面上限、残差采样目标、约束权重、内点与复验比例、迭代与阻尼、局部恢复 halo 与连通倍率。
- **曲面检测**：残余多尺度轮数、区域法向对齐余弦、聚类近邻数、最低支持点下限。
- **预处理**：方向传播距离代价权重。

纯数学/结构常量（π、`1e-12` 退化保护、6 个面、3 个轴、数组长度、循环上界）仍保留在代码中，不作为配置项。

## 2. 加载优先级

对任意模块 `M`，实际生效值按以下顺序确定：

1. 命令行 `--set` 覆盖（模块 CLI 接受裸 `key=value`；客户端接受 `module.key=value`）；
2. 命令行 `--config <file>` 显式指定的文件；
3. 环境变量 `PARAMCAD_<MODULE>_CONFIG`，例如 `PARAMCAD_DETECTION_CONFIG`；
4. 可执行文件旁的 `config/<module>.conf`；
5. 源码树 `apps/ParamCAD Studio/config/<module>.conf`（开发构建的兜底）。

未提供的键使用 C++ 结构体默认值；所有已跟踪的 `.conf` 都写全了键，因此正常部署不依赖兜底默认值。

投影模块保留旧的裸键覆盖语义：`--set maximum_axes=5` 仍然只作用于 `projection.conf`。客户端新增的覆盖必须写成 `模块名.键=值`，避免跨模块歧义。

## 3. 设置界面

顶栏 **Settings → 超参数** 现在列出全部 7 个模块。左侧为模块列表，右侧为滚动表单，每个二级分组渲染为一个 `QGroupBox`。保存行为：

- 只写回被修改过的模块，未修改的文件不会被重写；
- 写回前对整份配置做范围与跨字段校验，任一模块失败则整体不保存；
- 使用 `QSaveFile` 原子写入；
- 读取失败的模块会显示错误，但仍可编辑并保存为一份完整有效配置；
- 运行时 `--set` 覆盖不会写回文件，界面展示的是文件中的值。

## 4. 模块 CLI

新增 6 个独立可执行文件，与客户端一起构建（`CAD_CLIENT_BUILD_MODULE_CLIS`，默认 `ON`）：

| 命令 | 作用 | 主要输出 |
| --- | --- | --- |
| `pcad_preprocess` | 读取 PLY，执行降采样、离群过滤、法向估计 | 处理后的 PLY + JSON 报告 |
| `pcad_detect` | 曲面基元检测 | 证据列表 JSON |
| `pcad_analyze` | 连通面片与关系推断 | 面片、关系 JSON |
| `pcad_recognize` | 常规参数化实体识别 | 候选统计 JSON |
| `pcad_assemble` | 装配面片分解、边界与闭合 | 部件与候选 JSON |
| `pcad_project` | 投影拉伸拟合，可选 STEP 导出 | 拟合质量与操作 JSON |

公共参数：

```text
--in <cloud.ply>          输入点云
--synthetic box           内置 60x40x20 mm 合成盒（冒烟测试）
--out <file.ply>          仅 pcad_preprocess
--report <file.json>      报告路径；省略时输出到标准输出
--step <file.step>        仅 pcad_project
--unit mm|m|arbitrary     覆盖 PLY 单位推断
--config <file>           指定该模块配置文件
--set key=value           覆盖单项，可重复
--print-config            打印实际生效配置后退出
```

示例：

```powershell
$exe = ".\apps\ParamCAD Studio\build\msvc-debug\cloud-only\Release"

& "$exe\pcad_preprocess.exe" --synthetic box --out .\outputs\box.ply --report .\outputs\preprocess.json

& "$exe\pcad_detect.exe" --synthetic box --set plane_maximum_planes=3 --report .\outputs\detect.json

& "$exe\pcad_recognize.exe" --in .\outputs\box.ply --unit mm `
    --set box_angular_tolerance_deg=7 --report .\outputs\recognize.json

& "$exe\pcad_project.exe" --synthetic box --path two-axis-intersection `
    --set intersection_alpha_scale=1.5 --report .\outputs\project.json
```

客户端本身也接受启动时覆盖：

```powershell
& ".\ParamCAD Studio.exe" --config-dir .\config `
    --set detection.plane_maximum_planes=10 `
    --set projection.maximum_axes=8
```

`--performance-capture` 的 62 视角计划和 1920x1080 输出尺寸改为读取 `capture.conf`。

## 5. 已执行的验证

构建环境：Windows，MSVC 2022 x64，Release，`apps/ParamCAD Studio/build/msvc-debug`。

| 验证 | 命令/对象 | 结果 |
| --- | --- | --- |
| 客户端构建 | `cmake --build ... --target CadPointCloudClient` | 通过 |
| 6 个模块 CLI 构建 | `--target pcad_preprocess ... pcad_project` | 通过 |
| 校验工具构建 | `--target cad_reconstruction_validation` | 通过 |
| 配置层单测 | `module_settings_test.exe` | 通过 |
| 客户端设置界面测试 | `parameter_editing_test.exe` | 通过 |
| 点云 I/O 回归 | `point_cloud_io_test.exe` | 通过 |
| 点云分析回归 | `point_cloud_analysis_test.exe` | 通过 |
| 曲面分析回归 | `surface_analysis_test.exe` | 通过 |
| 装配分解回归 | `assembly_reconstruction_test.exe` | 通过 |
| CAD 后台任务回归 | `cad_reconstruction_task_test.exe` | 通过 |
| 6 个 CLI 合成盒冒烟 | `--synthetic box`，输出到 `outputs/hyperparameter-cli-smoke-20261110/` | 全部退出码 0 |
| 单项覆盖冒烟 | `plane_maximum_planes=3`、`box_angular_tolerance_deg=7`、`maximum_axes=5` | 报告中均为覆盖后的值 |
| `--print-config` | 6 个 CLI | 全部退出码 0，输出可被自身重新加载 |
| 跟踪配置解析 | 6 个 `.conf` | `module_settings_test` 全部加载并通过校验 |

`module_settings_test` 覆盖：注释、BOM、部分键、未知键、重复键、越界、跨字段冲突（投影端面点数与拟合点数）、序列化回环、点号覆盖、裸投影覆盖、环境变量路径与取值优先级。

`parameter_editing_test` 新增覆盖：设置对话框列出 7 个模块、包含二级分组、投影与预处理两个模块可同时修改并原子保存、保存结果可被默认加载器读回。

## 6. 尚未验证或未覆盖的部分

- **GUI 手工交互**：未在真实桌面会话中逐页点击 7 个模块并核对布局、滚动与长文本换行。
- **真实照片 / COLMAP 链路**：本次不涉及 Python 与 COLMAP；`reconstruction` 的图像重建参数仍由 `reconstruct.py` 的命令行控制。
- **拍摄模块没有独立 CLI**：离屏拍摄依赖客户端 VTK 视口，因此 `capture.conf` 的命令行入口是 `ParamCAD Studio.exe --performance-capture ... --config-dir ... --set capture.image_width=...`，而不是单独的 `pcad_capture`。
- **建模核心**：用户选择的范围是"C++ 几何链路 + 拍摄"，`core/parametric_modeling` 目前没有 `Options` 结构，未新增 `modeling.conf`。
- **混合特征识别器**：已新增 `MixedFeatureRecognitionOptions`，宿主、孔壁、局部点数、端盖、凸台、切槽阈值均已外置。
- **引导模板拟合**：已新增 `GuidedRecognitionOptions`，模板点数、边界容差、支持比例、扩展与复验阈值均已外置。
- **装配局部恢复的辅助函数**：`strongestCapPair`、`channelWindows`、`sampledCapPlanes`、`residualCaps` 仍直接接收 `tolerance`，其中的面观测门槛（48/80/100）、深度倍率（12/4）、`0.9999` 共面余弦、`0.6/0.30` 配对比例、`voteBudget=16384` 以及 6/4/12/24/8 的局部尝试预算尚未外置；`recoverLocalProfiles` 主流程的门槛已外置。
- **环形管/解析轮廓闭合**：`AnnularTubeClosure.cpp`、`AnalyticProfileClosure.cpp` 中的 96/100/400/48/16/24、`0.92/0.85/0.75/0.7/0.55/0.45` 等专用阈值仍为内部常量。
- **关系优化的少量实现细节**：圆锥开口角上下限 `0.001/1.56`、有限差分步长 `1e-6`、阻尼增减倍率 `0.5/10` 仍保留在代码中。
- **识别修复路径的关系参数**：`recognitionAnalysisOptions` 目前只接收识别配额，`analysis.conf` 中的关系优化细化参数主要作用于“识别表面/更新关系”面板路径；若要让 CAD 重建的修复事务也读取这些键，需要继续扩展 `RecognitionAnalysisSettings`。
- **发布包**：未重新执行 `packaging/package-client.ps1`，未验证离线包的 `config/` 内容。
- **客户端可执行文件重链接**：本轮验证时客户端正在运行，`ParamCAD Studio.exe` 被占用，最终链接被拒绝；测试目标均已链接最新核心并运行通过。关闭客户端后重新执行 `--target CadPointCloudClient` 即可。
- **完整回归**：未运行几何重建的全部 CTest 用例和带真值的性能验收。

## 7. 已知问题与维护注意

1. **构建目录中的配置不会被覆盖。** `StageModuleConfigs.cmake` 只在目标文件不存在时复制，避免覆盖用户运行期修改。修改源码 `.conf` 后，需要删除 `build/.../Release/config/` 或手动复制，才能让旧构建目录读到新默认值。
2. **开发构建的路径兜底。** 若可执行文件旁没有 `config/`，加载器会回退到源码树路径。发布包必须确保 `config/` 被复制，否则删除源码树后无法启动相关功能。
3. **没有版本与迁移机制。** 配置键一旦重命名，旧文件会因"未知键"报错。后续如需演进，建议加入 `schema_version` 与旧键迁移。
4. **拍摄轨道固定 7 个槽位。** 目前用 `ring1` 到 `ring7` 表达默认 62 视角计划；超过 7 环需要扩展 schema。
5. **球形/圆环最大半径共用圆柱倍率。** 为减少键数量，`sphere/torus` 的自动上限暂用 `cylinder_maximum_radius_diagonal_fraction`；如果后续需要独立控制，应拆分专用键。
6. **设置界面只显示文件值。** 客户端 `--set` 的临时覆盖不会显示在界面中，也不会被保存；这是为了避免把一次调试覆盖固化成默认配置。
7. **CLI 报告是诊断摘要。** 识别与装配 JSON 目前记录计数、覆盖率和错误，不包含可回放的完整特征参数；需要完整结果时仍应使用 `cad_reconstruction_validation` 的既有报告入口。
