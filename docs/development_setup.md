# Windows 开发环境：克隆、安装依赖、编译与启动

本文面向第一次克隆 Bochu Capstone 的开发者，给出从空白 Windows x64 环境到启动 ParamCAD Studio 的完整步骤。

## 1. 先说明当前仓库能自动安装什么

仓库已有以下入口：

| 入口 | 用途 |
| --- | --- |
| `packaging/Install-Dependencies.ps1` | 下载并校验固定版本的嵌入式 Python 与 COLMAP |
| `deps/verify.ps1` | 非递归检查本地 VTK、OpenCASCADE 和 COLMAP 的关键文件 |
| `apps/ParamCAD Studio/CMakePresets.json` | 配置和编译桌面客户端 |
| `packaging/package-client.ps1` | 构建并制作 full/bootstrap Windows 发布包 |

目前没有一条命令能从公开仓库自动准备全部开发依赖，原因是：

- Qt 需要通过 Qt 安装器选择 MSVC x64 版本；
- 仓库使用带 Qt `GUISupportQt` 的项目本地 VTK 9.7.0 SDK，当前没有公开下载清单；
- OpenCASCADE 使用官方二进制 SDK，需要先取得并解压对应归档；
- 这些大型 SDK 被 `.gitignore` 排除，不会随 `git clone` 下载。

因此，Python、COLMAP、CGAL 和 Eigen 可以通过下面的命令安装；Qt、VTK 和 OpenCASCADE 需要先按约定放到正确位置。待三个 SDK 就位后，配置、编译和启动都可以直接使用命令完成。

## 2. 支持的开发环境

- Windows 10/11 x64；
- Visual Studio 2022，安装“使用 C++ 的桌面开发”；
- CMake 3.24 或更高版本；
- Qt 6.5 或更高版本，**MSVC 2022 64-bit** 构建；
- VTK 9.7.0，MSVC x64 Release，并启用 Qt 6 `GUISupportQt`；
- OpenCASCADE 8.0.1，MSVC x64 Release；
- vcpkg `x64-windows`；
- Python 3.13.10 与 COLMAP 4.2.0（可由仓库脚本安装）。

Qt、VTK、OpenCASCADE 和通过 vcpkg 安装的库必须采用兼容的 MSVC x64 ABI，不能混用 MinGW 包。

## 3. 克隆仓库

在 PowerShell 中执行：

```powershell
git clone https://github.com/yang-pei-hua/Bochu_Capstone.git
Set-Location .\Bochu_Capstone
$ProjectRoot = (Get-Location).Path
```

正常编译和运行使用预编译 COLMAP，不需要初始化 COLMAP 源码子模块。只有审计或自行编译 COLMAP 时才执行：

```powershell
git submodule update --init core/reconstruction/third_party/colmap
```

## 4. 安装系统工具

如果机器尚未安装 Git、CMake 和 Visual Studio，可使用 `winget`：

```powershell
winget install --id Git.Git -e
winget install --id Kitware.CMake -e
winget install --id Microsoft.VisualStudio.2022.Community -e `
  --override "--wait --passive --add Microsoft.VisualStudio.Workload.NativeDesktop --includeRecommended"
```

安装完成后重新打开 PowerShell，并检查：

```powershell
git --version
cmake --version
```

CMake 使用 `Visual Studio 17 2022` generator 时会自动定位 Visual Studio，`msbuild` 不必预先出现在普通 PowerShell 的 `PATH` 中。

## 5. 安装 Qt、VTK 和 OpenCASCADE

### 5.1 Qt

使用 Qt 官方安装器安装 Qt 6.5+ 的 **MSVC 2022 64-bit** 组件。下面以 Qt 6.11.2 为例：

```text
D:\Qt\6.11.2\msvc2022_64
```

记录安装根目录，后续命令会使用：

```powershell
$QtRoot = "D:\Qt\6.11.2\msvc2022_64"
Test-Path "$QtRoot\lib\cmake\Qt6\Qt6Config.cmake"
```

检查结果必须为 `True`。Qt 版本还应与所使用的 VTK SDK 兼容。

### 5.2 OpenCASCADE

取得官方 `opencascade-8.0.1-vc14-64-combined.zip`，将其中的 `opencascade-8.0.1-vc14-64` 目录解压并重命名为：

```text
deps\occt-8.0.1
```

关键文件应位于：

```text
deps\occt-8.0.1\cmake\OpenCASCADEConfig.cmake
deps\occt-8.0.1\win64\vc14\lib\TKernel.lib
deps\occt-8.0.1\win64\vc14\bin\TKernel.dll
```

归档校验值和目录说明见 [`../deps/README.md`](../deps/README.md)。

### 5.3 VTK

从项目维护者处取得与当前 Qt/MSVC 环境匹配的 VTK 9.7.0 Release SDK，放到：

```text
deps\vtk-9.7.0
```

关键文件应位于：

```text
deps\vtk-9.7.0\lib\cmake\vtk-9.7\vtk-config.cmake
```

该 VTK 必须启用 Qt 6 `GUISupportQt`。普通的不带 Qt 支持的 VTK 包不能构建当前客户端。仓库目前没有该 SDK 的自动下载地址；在项目补充固定归档与 SHA-256 清单前，这是全新克隆无法完全一键安装的部分。

## 6. 安装 vcpkg、CGAL 和 Eigen3

以下示例使用项目当前验证过的 vcpkg revision，它提供 CGAL 6.2.1：

```powershell
$VcpkgRoot = "D:\dev\vcpkg"
git clone https://github.com/microsoft/vcpkg.git $VcpkgRoot
git -C $VcpkgRoot checkout 07f4812200df3d3c931c0c8a6081d3b21fe2bf9f
& "$VcpkgRoot\bootstrap-vcpkg.bat" -disableMetrics
& "$VcpkgRoot\vcpkg.exe" install cgal:x64-windows eigen3:x64-windows
```

验证安装结果：

```powershell
& "$VcpkgRoot\vcpkg.exe" list | Select-String "^(cgal|eigen3|gmp):x64-windows"
Test-Path "$VcpkgRoot\installed\x64-windows\bin\gmp-10.dll"
```

最后一项必须为 `True`。客户端运行需要 CGAL 间接依赖的 GMP DLL，CMake 会在构建后把它复制到程序目录。

如果已经有 vcpkg，可直接设置 `$VcpkgRoot` 并运行安装命令；不要在包含未提交修改的 vcpkg checkout 中强制切换 revision。

## 7. 安装 Python 和 COLMAP

仓库脚本会从 `packaging/dependencies.json` 读取固定版本、下载归档、校验 SHA-256，并安装到 `deps/python` 和 `deps/colmap`。

只需要稀疏点云、或机器没有支持的 NVIDIA GPU时，推荐 CPU 版：

```powershell
& "$ProjectRoot\packaging\Install-Dependencies.ps1" `
  -InstallRoot $ProjectRoot `
  -ColmapFlavor nocuda
```

需要稠密重建且机器具有支持的 NVIDIA GPU时使用 CUDA 版：

```powershell
& "$ProjectRoot\packaging\Install-Dependencies.ps1" `
  -InstallRoot $ProjectRoot `
  -ColmapFlavor cuda
```

该脚本安装的是重建运行时，不会安装 Qt、VTK、OpenCASCADE、CGAL 或 Visual Studio。

## 8. 安装完成后的统一检查

先检查项目本地 SDK 和 COLMAP：

```powershell
& "$ProjectRoot\deps\verify.ps1"
```

再检查脚本未覆盖的 Qt、Python 和 vcpkg 文件：

```powershell
$RequiredFiles = @(
  "$QtRoot\lib\cmake\Qt6\Qt6Config.cmake",
  "$VcpkgRoot\scripts\buildsystems\vcpkg.cmake",
  "$VcpkgRoot\installed\x64-windows\share\cgal\CGALConfig.cmake",
  "$VcpkgRoot\installed\x64-windows\share\eigen3\Eigen3Config.cmake",
  "$VcpkgRoot\installed\x64-windows\bin\gmp-10.dll",
  "$ProjectRoot\deps\python\python.exe"
)
$RequiredFiles | ForEach-Object {
  [pscustomobject]@{ Path = $_; Exists = Test-Path -LiteralPath $_ }
}
```

所有结果都应为 `True`。

## 9. 第一次配置和编译客户端

从仓库根目录执行：

```powershell
Push-Location "$ProjectRoot\apps\ParamCAD Studio"
cmake --preset msvc-debug `
  -DCMAKE_TOOLCHAIN_FILE="$VcpkgRoot\scripts\buildsystems\vcpkg.cmake" `
  -DCMAKE_PREFIX_PATH="$QtRoot"
cmake --build --preset msvc-release --parallel
Pop-Location
```

虽然配置 preset 名为 `msvc-debug`，它生成的是同时支持 Debug 和 Release 的 Visual Studio 构建树；`msvc-release` build preset 会编译项目当前使用的 Release 配置。项目本地的 VTK 和 OpenCASCADE 会由 CMake 自动发现。

成功后程序位于：

```text
apps\ParamCAD Studio\build\msvc-debug\Release\ParamCAD Studio.exe
```

CMake 的构建后步骤会把 Qt、VTK、OpenCASCADE、GMP 及 Qt plugins 放到可执行文件旁边。

## 10. 后续重新编译

普通增量编译：

```powershell
Push-Location "$ProjectRoot\apps\ParamCAD Studio"
cmake --build --preset msvc-release --parallel
Pop-Location
```

需要先清理目标再完整重编时：

```powershell
Push-Location "$ProjectRoot\apps\ParamCAD Studio"
cmake --build --preset msvc-release --clean-first --parallel
Pop-Location
```

更换 Qt、vcpkg、VTK 或 OpenCASCADE 路径后，应重新执行第 9 节的 `cmake --preset msvc-debug ...` 配置命令。若确认 CMake cache 已损坏，再移除 `apps/ParamCAD Studio/build/msvc-debug` 后重新配置；删除前先确认路径，避免误删其他构建目录。

## 11. 启动客户端

从仓库根目录执行：

```powershell
& "$ProjectRoot\apps\ParamCAD Studio\build\msvc-debug\Release\ParamCAD Studio.exe"
```

不要只复制 `ParamCAD Studio.exe` 到其他目录。它需要与构建后收集的 DLL 和 Qt plugins 保持在一起。

如果重新打开了 PowerShell，需要先恢复三个变量：

```powershell
$ProjectRoot = "D:\code\Bochu_Capstone"
$QtRoot = "D:\Qt\6.11.2\msvc2022_64"
$VcpkgRoot = "D:\dev\vcpkg"
```

将示例路径替换为本机实际路径。

## 12. 可选验证

验证 Python 重建模块：

```powershell
$env:PYTHONPATH = "$ProjectRoot\core\reconstruction"
$env:PYTHONDONTWRITEBYTECODE = "1"
& "$ProjectRoot\deps\python\python.exe" -m unittest discover `
  -s "$ProjectRoot\core\reconstruction\tests" -v
```

查看重建命令是否可用：

```powershell
& "$ProjectRoot\deps\python\python.exe" `
  "$ProjectRoot\core\reconstruction\reconstruct.py" --help
```

## 13. 常见问题

### CMake 找到的是 MinGW Qt

确认 `$QtRoot` 指向包含 `msvc2022_64` 的 Qt，而不是 `mingw_64`，然后重新配置。MSVC 构建不能链接 MinGW Qt。

### 找不到 `GUISupportQt`

当前 VTK SDK没有启用 Qt，或者 VTK 所用 Qt 与 `$QtRoot` 不兼容。换用项目要求的 VTK 9.7.0 SDK。

### 启动时报缺少 DLL

确认从 `build/msvc-debug/Release` 原位启动，并重新运行 Release 构建。构建后步骤负责收集 Qt、VTK、OpenCASCADE 和 GMP 运行时。

### 稠密重建失败

先运行：

```powershell
& "$ProjectRoot\deps\colmap\bin\colmap.exe" version
nvidia-smi
```

COLMAP 版本信息需要包含 CUDA，且 NVIDIA 驱动必须正常。没有可用 GPU 时改装 `nocuda` 版本并使用稀疏重建。
