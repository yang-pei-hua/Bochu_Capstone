# pcad 命令行工具

`pcad` 是仓库工具的统一入口，使用 Python 3.10 或更新版本，仅依赖标准库。

## STEP 点云与逐面识别验证

已有验证工具现在支持直接从 STEP 表面采样，无需图片或 COLMAP：

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python tools/run_step_surface_validation.py data/real-workpieces-step/cover-plate/cover-plate.step --output outputs/cover-plate-validation
```

输出目录必须新建或为空。生成带法向的毫米点云、独立 STEP 参考清单、核心和客户端逐面报告、
着色分区 PLY 及 `summary.json`。每个参考面要求同类型候选支持率和实际显示标签覆盖率均达到 98%；
任一面失败，脚本返回非零。`--noise 0.005 --seed 719` 可重复验证位置噪声，
`--skip-client` 只跑核心。算法不接收参考标签；参考清单仅在检测完成后用于核对。
使用相同参数加 `--replay` 可重用已保存点云，之前的报告和着色点云会保留在 `history/`。

这是曲面识别验证，不代表已经恢复封闭、可编辑的 CAD 实体或导出重建 STEP。
实体统计表示该参考实体的所有面均通过检查，不表示自动恢复了装配实例的身份。

底层命令也可单独重放：

```powershell
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" sample source.step reference.json cloud.ply 1.0
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" analyze cloud.ply analysis.json reference.json 0.05
& "apps/ParamCAD Studio/build/msvc-debug/Release/cad_reconstruction_validation.exe" analyze-client cloud.ply client.json reference.json 0.05
```

`sample` 每面保留 256–2048 个样本，在三角网格的 UV 区域中采样并核对实际裁剪边界，
再计算解析位置和有向法向。间距是目标值；逐面实际密度会记录在参考清单中。
检测读取的 PLY 只有位置和法向，单位 sidecar 也不含面或实体标签。

## 仅点云的 CAD 重建

客户端与批处理均只使用点云坐标、法向、单位及用户手动指定的模板。
原 STEP 的自动发现、读取、逐面对应、拓扑引导和相关重建命令已删除。
已有工程的参数重建、普通 STEP 导入／导出继续保留。

```powershell
python tools/run_point_cloud_cad.py --input scan=path/to/scan.ply --unit mm --output outputs/cloud-cad-new
```

脚本只复制 PLY 和单位信息到隔离输入目录，再调用客户端同一个后台重建任务。
支持重复 `--input NAME=FILE.ply`，每个案例单独记录结果；没有对应表或原模型输入参数。
输出目录必须新建或为空，`--timeout 300` 控制每个案例的计算时限。

成功时输出可编辑 PCAD、参数 JSON 和 STEP，并验证保存重开及导出回读。
成功仅表示当前识别出的几何能形成有效 CAD，不证明恢复了未知原模型的全部部件。
多部件重建还输出 `segmented.ply` 与 `parts.json`，记录候选归属、有限边界、连接和未闭合原因。
仅部分闭合时报告 `cad_exported_partial`；没有可验证实体时报告 `unsupported`，仍保留分离诊断。
分离流程的 `surface_coverage` 是归属于闭合候选的观测点比例，不是原模型恢复率；`coverage_basis` 明确语义。
不会读取旁边的模型文件来补齐。四份案例结果和客户端用法见 [纯点云 CAD 说明](../docs/Real_Workpieces_Editable_CAD.md)。

前一节的 STEP 采样／逐面评测仅用于离线制作与评估测试数据；
参考拓扑只在曲面检测完成后用于评测，不进入本节的重建任务。

## 安装与使用

在仓库根目录运行一次，将 `tools` 加入当前用户的 PATH，无需管理员权限：

```powershell
& .\tools\install_pcad.ps1
```

安装脚本会立即更新执行它的 PowerShell 会话。其他已打开的终端需要重新打开；
如果终端宿主仍继承旧 PATH，也可以在该终端重新运行安装脚本。重复安装不会添加重复条目。
仓库移动后重新运行安装脚本，并从用户 PATH 中移除旧目录。

```powershell
pcad --help
pcad studio
pcad studio --dry-run
pcad studio --exe "D:\custom\ParamCAD Studio.exe"
pcad clean output --dry-run
pcad clean output
```

`studio` 优先寻找 `cloud-only/Release` 本轮客户端，再尝试标准 Release、Debug 与其他
已使用的构建目录；启动后终端立即返回。路径与当前工作目录无关。

`clean output` 删除本仓库 `outputs/` 下生成的文件与目录，保留 `README.md` 和
`.gitignore`。包括生成的点云、截图、日志及重建结果；需要保留的结果请提前备份。
`--dry-run` 仅列出将删除的顶层条目。清理范围固定，符号链接和 Windows junction
只移除链接本身；若 `outputs/` 本身为链接或缺少说明文件则拒绝清理。

无需安装也可以直接运行：

```powershell
.\tools\pcad.cmd studio
python .\tools\pcad.py clean output --dry-run
```

原有 `python tools/clean_outputs.py [--dry-run]` 入口继续有效，转发到同一份实现。
退出码：`0` 成功，`1` 部分清理失败，`2` 参数、路径或启动错误。

## 代码组织

```text
tools/
├── pcad.cmd              Windows 命令入口，调用 Python
├── pcad.py               Python 统一入口
├── install_pcad.ps1      注册当前用户 PATH
└── pcad_cli/
    ├── cli.py            参数解析和命令分发
    ├── paths.py          仓库路径
    ├── studio.py         Studio 启动
    └── clean.py          输出清理
```

新增 CLI 功能时，将具体实现放入 `pcad_cli/`，在 `cli.py` 注册子命令并设置
`handler`，由 `run(args)` 返回退出码。现有性能验证脚本仍可独立运行。

在仓库根目录执行相关测试：

```powershell
python -m unittest discover -s tools/tests -p test_pcad_cli.py -v
```
