# pcad 命令行工具

`pcad` 是仓库工具的统一入口，使用 Python 3.10 或更新版本，仅依赖标准库。

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

`studio` 优先寻找客户端标准构建目录中的 Release 程序，再尝试 Debug 与其他
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
