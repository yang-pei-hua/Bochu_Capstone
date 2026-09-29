# COLMAP 无界面重建环境安装与复现

本文档说明如何在 Windows x64 上复现本项目 Layer 2 的 COLMAP 运行环境。
项目调用 COLMAP 命令行程序，不使用其 GUI。当前固定版本为：

- COLMAP：`4.2.0`
- 上游提交：`be5e29168d4aff238409d60424812df66aac919f`
- 运行包：`colmap-x64-windows-cuda.zip`
- Python：`3.10` 或更高版本

官方来源：

- 发布页：<https://github.com/colmap/colmap/releases/tag/4.2.0>
- 源码仓库：<https://github.com/colmap/colmap>

## 1. 前置条件

在项目根目录打开 PowerShell。以下命令应返回项目路径：

```powershell
Get-Location
```

使用 CUDA 稠密重建需要 Windows x64、受支持的 NVIDIA GPU 和可正常工作的
NVIDIA 驱动。检查驱动：

```powershell
nvidia-smi
```

如果只需要默认的稀疏点云，可以改用官方
`colmap-x64-windows-nocuda.zip`，不要求 NVIDIA GPU。

## 2. 安装 Python

先检查现有版本：

```powershell
python --version
```

若没有 Python，可通过 `winget` 安装：

```powershell
winget install --id Python.Python.3.13 -e
```

重新打开 PowerShell 后再次运行 `python --version`。本项目的 Python 适配层仅使用
标准库，包括 `argparse`、`json`、`pathlib`、`sqlite3` 和 `subprocess`，因此：

> 不需要安装任何额外的 pip 依赖，也不需要安装 `pycolmap`。

可选地创建隔离环境；即使没有第三方依赖，这也能固定所用的 Python 解释器：

```powershell
python -m venv .venv
& .\.venv\Scripts\python.exe --version
```

后续命令中的 `python` 均可替换为 `& .\.venv\Scripts\python.exe`。

## 3. 下载并安装 COLMAP 预编译包

推荐使用官方预编译包，无需 Visual Studio、vcpkg 或 CUDA Toolkit。下面的命令会
把固定版本下载到系统临时目录，并安装到项目统一依赖目录
`deps\colmap`。该目录已被 Git 和项目搜索规则忽略。

```powershell
$ProjectRoot = (Get-Location).Path
$ColmapVersion = "4.2.0"
$DownloadUrl = "https://github.com/colmap/colmap/releases/download/$ColmapVersion/colmap-x64-windows-cuda.zip"
$TempRoot = Join-Path ([System.IO.Path]::GetTempPath()) ("ece4500j-colmap-" + [guid]::NewGuid().ToString("N"))
$ArchivePath = Join-Path $TempRoot "colmap-x64-windows-cuda.zip"
$ExtractPath = Join-Path $TempRoot "extracted"
$InstallPath = Join-Path $ProjectRoot "deps\colmap"

if (Test-Path -LiteralPath $InstallPath) {
    throw "COLMAP target already exists: $InstallPath"
}

New-Item -ItemType Directory -Path $TempRoot, $ExtractPath -Force | Out-Null
Invoke-WebRequest -Uri $DownloadUrl -OutFile $ArchivePath
Expand-Archive -LiteralPath $ArchivePath -DestinationPath $ExtractPath

if (-not (Test-Path -LiteralPath (Join-Path $ExtractPath "bin\colmap.exe"))) {
    throw "Unexpected archive layout: bin\colmap.exe was not found"
}

New-Item -ItemType Directory -Path (Split-Path -Parent $InstallPath) -Force | Out-Null
Move-Item -LiteralPath $ExtractPath -Destination $InstallPath
```

如果已经手动下载并解压到其他目录，例如
`D:\Downloads\colmap-x64-windows-cuda`，可以直接移动：

```powershell
$SourcePath = "D:\Downloads\colmap-x64-windows-cuda"
$InstallPath = Join-Path (Get-Location).Path "deps\colmap"

if (Test-Path -LiteralPath $InstallPath) {
    throw "COLMAP target already exists: $InstallPath"
}

New-Item -ItemType Directory -Path (Split-Path -Parent $InstallPath) -Force | Out-Null
Move-Item -LiteralPath $SourcePath -Destination $InstallPath
```

## 4. 验证 COLMAP

直接运行版本检查：

```powershell
& .\deps\colmap\bin\colmap.exe version
```

CUDA 包的期望输出类似：

```text
COLMAP 4.2.0 (Commit be5e291 on 2026-08-31 with CUDA)
```

验证 Python 适配层能够自动发现 COLMAP：

```powershell
$env:PYTHONPATH = (Resolve-Path ".\core\reconstruction").Path
python -c "from colmap_reconstruction.pipeline import find_colmap_executable; print(find_colmap_executable())"
```

期望路径结尾为：

```text
deps\colmap\bin\colmap.exe
```

## 5. 克隆固定版本源码（仅审计或自行编译时需要）

预编译包已经足够运行重建。若还需要保留对应源码，在全新项目副本中执行：

```powershell
git clone --depth 1 --branch 4.2.0 `
  https://github.com/colmap/colmap.git `
  .\core\reconstruction\third_party\colmap
```

如需自行编译无 GUI 版本，先安装 Visual Studio C++、CMake、Ninja、vcpkg 以及
COLMAP 所需 C++ 依赖，然后执行：

```powershell
# CPU/SfM 版本
.\core\reconstruction\build_colmap.ps1

# CUDA/MVS 版本；还要求本机安装 CUDA Toolkit
.\core\reconstruction\build_colmap.ps1 -Cuda
```

通常不建议为了运行本项目而自行编译；官方预编译包更容易复现。

## 6. 验证 Python 模块

运行不依赖真实图片的单元测试：

```powershell
$env:PYTHONPATH = (Resolve-Path ".\core\reconstruction").Path
$env:PYTHONDONTWRITEBYTECODE = "1"
python -m unittest discover -s .\core\reconstruction\tests -v
```

查看命令行参数：

```powershell
python .\core\reconstruction\reconstruct.py --help
```

## 7. 执行重建

不提供相机信息，自动估计内参和位姿并输出稀疏点云：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\images `
  --output .\data\cloud_sparse.ply
```

提供相机内参和全部或部分位姿：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\images `
  --camera-info .\data\camera_info.json `
  --input-unit mm `
  --output .\data\cloud_sparse.ply
```

使用 CUDA 输出稠密点云：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\images `
  --camera-info .\data\camera_info.json `
  --input-unit mm `
  --dense `
  --gpu `
  --output .\data\cloud_dense.ply
```

CameraInfo 格式见
`core\reconstruction\camera_info.example.json`，完整接口说明见
`core\reconstruction\README.md`。

## 8. Git 与本地二进制

COLMAP 预编译包体积较大，不应提交到 Git。项目根 `.gitignore` 已包含：

```gitignore
/deps/colmap/
```

安装后可在 Git 仓库中用下列命令确认忽略状态：

```powershell
git check-ignore -v .\deps\colmap\bin\colmap.exe
```

如果当前目录尚未初始化为 Git 仓库，该验证命令会失败；初始化后再执行即可。

## 9. 常见问题

### 找不到 COLMAP executable

确认文件存在：

```powershell
Test-Path .\deps\colmap\bin\colmap.exe
```

若把 COLMAP 放在其他目录，可在重建命令中显式指定：

```powershell
--colmap "D:\path\to\colmap\bin\colmap.exe"
```

### 稀疏重建成功，但 `--dense` 失败

确认版本输出包含 `with CUDA`，并确认 `nvidia-smi` 正常工作。`--dense` 中的
PatchMatch Stereo 需要具备 GPU 后端的 COLMAP；仅 CPU 的预编译包只能运行稀疏
重建流程。

### 工作目录非空

适配层为避免覆盖数据，只接受空工作目录。删除旧工作目录前请先确认其中没有要
保留的结果，或通过 `--workspace` 指定一个新的目录。
