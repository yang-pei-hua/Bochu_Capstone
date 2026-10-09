# Windows 完整离线 Release 打包指南

默认只生成一个完整依赖版 ZIP。客户端、Python、COLMAP、Qt/VTK/OpenCASCADE 和 MSVC 运行库随包分发，启动和安装不触发依赖下载。

本文件供维护者打包和排错；发布包内的 `README.md` 来自 [PACKAGE_README.md](PACKAGE_README.md)。只修改本指南时，无需重新打包已有 Release。

## 1. 下次直接怎么做

在项目根目录执行。先关闭从默认构建目录启动的客户端，避免 Windows 锁住待更新的 EXE/DLL。已有构建缓存和本地运行时的情况下，不需要重新配置依赖，也不需要手动整理文件。

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1
```

默认版本为 `0.2.1`。指定新包版本：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1 -Version 0.2.1
```

`-Version` 设置包名、输出目录和 manifest 的版本，不会修改客户端源码中的产品版本。产品版本有变化时，应同步修改 `apps/ParamCAD Studio/CMakeLists.txt` 的项目版本、`src/main.cpp` 的 applicationVersion、`src/app/MainWindow.cpp` 的 About 显示，以及 `package-client.ps1` 的默认版本。

需要保留日志时：

```powershell
New-Item -ItemType Directory -Path .\artifacts -Force | Out-Null
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1 *> .\artifacts\packaging-full.log
if ($LASTEXITCODE -ne 0) {
    Get-Content -LiteralPath .\artifacts\packaging-full.log -Tail 60
    throw '打包失败，请按日志中的第一个错误排查'
}
Get-Content -LiteralPath .\artifacts\packaging-full.log -Tail 20
```

成功后应看到 `Created ...zip`、大小、SHA-256 和 `No bootstrap package was created`。Release 上传以下两个文件：

```text
artifacts/releases/<版本>/
  ParamCAD-Studio-<版本>-windows-x64-full-cuda.zip
  SHA256SUMS.txt
```

脚本只生成本地产物，不上传 Release。同版本、同 flavor 再次打包会替换对应 ZIP，并重写该输出目录中的 `SHA256SUMS.txt`。

## 2. 这次时间主要花在哪里

2026-10-08 这次主要工作是首次改造打包流程、定位失败原因，并多轮验证修复后的包。没有逐阶段计时记录，不能准确分摊每一步耗时。

| 工作 | 具体内容 | 下次如何减少重复工作 |
| --- | --- | --- |
| 整理旧打包流程 | 将原来的 Full/Bootstrap 流程改为只产出完整离线包，去掉包内下载入口，重新划分目录 | 直接运行现有 `package-client.ps1` |
| 修复依赖检查 | 处理 Windows 系统 CRT 误判、OCCT 开发工具引入的 Tcl 依赖，补齐可分发的 MSVC 运行库 | 保留 `Runtime-Helpers.ps1` 中的规则，不再逐个手工复制 DLL |
| 清理发布内容 | 本次从复制后的 payload 中清理了 166 个测试/调试文件 | 由脚本自动筛除，无需手工遍历本地依赖 |
| 修复 ZIP 和安装入口 | 修正 ZIP 条目路径，以及 PowerShell 7 经 CMD 启动 Windows PowerShell 的模块兼容问题 | 保留现有压缩逻辑和 CMD 环境处理 |
| 重复生成和验证 | 多轮复制运行时、计算哈希、压缩，再验证解压和安装；最终 payload 约 720 MiB | 修复已固化；通常一次默认打包，再验证实际解压结果即可 |
| 安装与升级验证 | 验证中文/空格路径、离线安装、保留 `outputs/`、损坏包拒绝安装 | 改动安装脚本或目录布局时再执行完整安装回归 |

现在常规打包仍会执行增量构建、运行时复制、校验和压缩；耗时受需要重新编译的文件数量、磁盘和 CPU 影响。此前反复失败和重新打包的过程不应成为每次发布的固定步骤。

## 3. 打包机需要的本地输入

以下是打包机的要求，使用者无需安装这些开发环境。

| 输入 | 默认位置或发现方式 |
| --- | --- |
| CMake、Visual Studio 2022 x64 C++ 构建工具 | `cmake` 在 PATH 中；脚本使用 Visual Studio 17 2022 生成器 |
| Git、Windows PowerShell | 记录构建来源、执行发布校验 |
| Qt | 优先 `-QtRoot` / `QTDIR`，否则读取已有 CMake 缓存；本机为 `D:/Qt/6.11.2/msvc2022_64` |
| vcpkg 已安装的 x64-windows 依赖 | 优先 `-VcpkgRoot` / `VCPKG_ROOT`，否则读取已有 CMake 缓存；本机为 `D:/dev/vcpkg` |
| VTK SDK | `deps/vtk-9.7.0`，CMake 配置入口为 `lib/cmake/vtk-9.7` |
| OpenCASCADE SDK | `deps/occt-8.0.1`，CMake 配置入口为 `cmake` |
| 完整 COLMAP 运行时 | `deps/colmap`；版本检查只执行确切路径 `deps/colmap/bin/colmap.exe version` |
| Python 嵌入式 ZIP | `artifacts/cache/python-3.13.10-embeddable-amd64.zip`，校验值必须匹配 `dependencies.json` |
| MSVC 可分发运行库 | 从 `VCToolsRedistDir` 或 vswhere 自动找到 `x64/Microsoft.VC143.CRT`；也可传 `-VCRuntimeRoot` |

首次构建、没有 CMake 缓存时，显式传入 Qt 和 vcpkg：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1 -QtRoot "D:\Qt\6.11.2\msvc2022_64" -VcpkgRoot "D:\dev\vcpkg"
```

所有离线输入应预先准备好。缺失或校验失败时脚本直接报错，不自动下载替代品。仓库中的 `Install-Dependencies.*` 是旧的依赖准备入口，当前打包脚本不调用它们，也不把它们放入发布包。

遵守仓库的依赖目录约束：不递归列出、搜索或读取本地 `deps/colmap/`、`deps/vtk-9.7.0/`、`deps/occt-8.0.1/` 的内容。COLMAP 按不透明运行时整体复制，清理和发布检查在 staging 副本上进行；VTK/OCCT 只访问构建、链接、运行验证及许可证复制所需的确切文件。

## 4. 常用参数和加速方式

| 参数 | 用途 |
| --- | --- |
| `-Version` | 包版本，默认 `0.2.1` |
| `-BuildDirectory` | 构建目录，默认 `apps/ParamCAD Studio/build/msvc-debug`；实际构建配置始终为 `Release` |
| `-OutputDirectory` | ZIP 和校验文件的输出目录，必须位于项目目录内 |
| `-QtRoot`、`-VcpkgRoot`、`-VCRuntimeRoot` | 显式指定打包机工具和运行库来源 |
| `-LocalColmapPath`、`-PythonArchive` | 指定已有的完整 COLMAP 目录及 Python 嵌入式 ZIP |
| `-ColmapFlavor nocuda` | 要求输入确实是不带 CUDA 的 COLMAP；不会下载或转换现有 CUDA 运行时 |
| `-Variant Offline` | `Full` 的同义选项，不产生另一种包 |
| `-SkipBuild` | 使用已有 Release 二进制；仍进行复制、依赖检查、启动验证、哈希和压缩 |
| `-Clean` | 删除指定构建目录后重新配置和构建；会失去增量构建优势，日常打包不需要 |
| `-SkipSmokeTest` | 跳过 GUI/Python/COLMAP 启动验证；manifest 会记录为未执行，正式发布不建议使用 |

只调整打包脚本或包内说明，且已确认 Release EXE 是最新时，可以使用：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\package-client.ps1 -SkipBuild
```

改过客户端 C++ 或 CMake 后使用默认构建，避免把旧 EXE 放入新包。只需诊断现有 staging 时，直接运行 `Test-Package.ps1`，无需重建和压缩：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\Test-Package.ps1 -PackageRoot ".\artifacts\staging\release\ParamCAD-Studio-0.2.0-windows-x64-full-cuda" -VerifyHashes
```

## 5. 自动打包具体做什么

1. 检查本地输入、Python ZIP 的 SHA-256、COLMAP 版本和 CUDA flavor。
2. 配置并增量构建 `CadPointCloudClient` 的 Release 目标。
3. 复制 GUI、Qt 插件、Python、COLMAP、重建脚本、离线安装脚本和第三方说明到 `artifacts/staging/release/<包名>/`。
4. 清理 staging 中的测试 EXE、PDB、LIB、EXP、OBJ、PYC；不复制客户端测试程序或 SDK。
5. 将可分发的 MSVC CRT DLL 放入 GUI、Python 和 COLMAP 的运行目录。
6. 检查 GUI 和插件的 PE 普通导入、延迟导入依赖；从指定的 Qt/vcpkg/CRT 目录补齐缺失 DLL，移除 GUI 不需要的 OCCT 开发工具 DLL。
7. 在只含 Windows 系统路径的 PATH 下验证 Python 模块、包内 COLMAP 查找、重建 CLI `--help`、COLMAP `version` 和 GUI 启动。GUI 保持运行并响应约 4 秒后，由验证脚本关闭。
8. 写入 `package-manifest.json`，记录 Git 提交、工作区修改状态、依赖版本、校验状态及 payload 文件哈希；再执行包结构、文件哈希和 GUI DLL 检查。
9. 生成仅有一个顶层产品目录的 ZIP，检查条目路径，成功后替换正式 ZIP，并生成 `SHA256SUMS.txt`。

普通打包自动验证的是 staging；实际 ZIP 的解压测试和安装升级测试需按下一节单独执行。启动检查通过不等于已经跑过完整的图片重建或所有 CAD 操作。

## 6. 实际 ZIP 的发布验证

在项目根目录运行，版本应与本次打包一致。测试目录每次使用新名字，可避免旧文件影响结果：

```powershell
$releaseVersion = '0.2.0'
$packageName = "ParamCAD-Studio-$releaseVersion-windows-x64-full-cuda"
$zipPath = Join-Path (Get-Location) "artifacts/releases/$releaseVersion/$packageName.zip"
$validationRoot = Join-Path (Get-Location) ("artifacts/release-validation/解压 空格-" + [guid]::NewGuid().ToString('N'))

$actualHash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
$sumPath = Join-Path (Split-Path -Parent $zipPath) 'SHA256SUMS.txt'
$sumText = Get-Content -LiteralPath $sumPath -Raw
if (-not $sumText.StartsWith("$actualHash  $packageName.zip")) { throw 'ZIP 校验值不匹配' }

New-Item -ItemType Directory -Path $validationRoot -Force | Out-Null
Add-Type -AssemblyName System.IO.Compression.FileSystem
[IO.Compression.ZipFile]::ExtractToDirectory($zipPath, $validationRoot)
$packageRoot = Join-Path $validationRoot $packageName
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\Test-Package.ps1 -PackageRoot $packageRoot -VerifyHashes
if ($LASTEXITCODE -ne 0) { throw '解压后的包验证失败' }
```

成功应看到 `Full offline package validation passed`。保留中文及空格路径测试，有助于发现路径拼接和引用问题。

修改安装脚本、目录布局或运行库部署后，再用隔离目录验证安装和升级；沿用上面的 `$validationRoot`、`$packageRoot`：

```powershell
$installRoot = Join-Path $validationRoot 'Installed ParamCAD Studio'
& (Join-Path $packageRoot 'Install-ParamCADStudio.cmd') -Destination $installRoot -NoShortcut
if ($LASTEXITCODE -ne 0) { throw '离线安装失败' }

$outputsRoot = Join-Path $installRoot 'outputs'
New-Item -ItemType Directory -Path $outputsRoot -Force | Out-Null
$probePath = Join-Path $outputsRoot 'upgrade-preservation.txt'
[IO.File]::WriteAllText($probePath, 'keep-on-upgrade')
& (Join-Path $packageRoot 'Install-ParamCADStudio.cmd') -Destination $installRoot -NoShortcut
if ($LASTEXITCODE -ne 0) { throw '离线升级失败' }
if ([IO.File]::ReadAllText($probePath) -ne 'keep-on-upgrade') { throw '升级未保留 outputs' }

powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\packaging\Test-Package.ps1 -PackageRoot $installRoot -VerifyHashes
if ($LASTEXITCODE -ne 0) { throw '安装后的包验证失败' }
```

`-NoShortcut` 避免测试修改用户开始菜单。需要测试损坏包时，只修改测试解压副本中的一个已记录文件，再调用安装入口：应报 `Package integrity check failed`，原安装和 `outputs/` 应保持不变。随后恢复副本文件；不要修改正式 ZIP 或 staging 来做损坏测试。

测试副本每份约需 720 MiB，验证结束后可删除本次创建的隔离测试目录。发布前同时检查 manifest 中的 `workingTreeModified`：未提交的代码也会进入包，manifest 的 Git 提交号不能单独代表这些修改。

## 7. 本次遇到的问题与已固化的解决方案

| 现象或错误 | 原因 | 现有解决方案 / 下次排查方式 |
| --- | --- | --- |
| 旧包目录杂乱、同时生成 Bootstrap，并带下载脚本 | 将构建输出整体复制到包根，旧入口支持安装时下载依赖 | 只产出 Full；按 `app/deps/core/installer/docs` 分组；包内不带 `Install-Dependencies.*` |
| `Missing runtime msvcrt.dll` | DLL 分类规则把 Windows 自带的 `msvcrt.dll` 误当成需单独分发的 VS CRT | `Test-SystemImport` 对 `msvcp`、`msvcr` 使用带数字的前缀匹配；系统 `msvcrt.dll` 按 Windows 组件处理 |
| `Missing runtime tcl86.dll (imported by ...TKDraw...)` | 开发构建复制了 OCCT 整个 bin 目录，包含客户端不用的 Draw/Tcl 工具 | 从 GUI EXE、Qt 插件及已知动态加载库出发检查依赖，只保留需要的 DLL；不为开发工具额外引入 Tcl |
| `Release contains developer artifacts or a dependency downloader` | 复制的依赖 payload 含测试 EXE、PDB 等；本次发现 149 个测试 EXE、17 个 PDB | 在 staging 副本清理，随后由 `Test-Package.ps1` 再检查；不要递归扫描原始 opaque 目录 |
| `Unexpected archive root` | Windows PowerShell 的 .NET Framework ZIP helper 生成反斜杠条目，与统一的 ZIP 路径检查不一致 | 使用 `ZipArchive` 显式流式写入条目，统一 `/` 分隔符，并加单个顶层产品目录；不要直接改回旧 helper |
| 经 `.cmd` 安装时报 `Get-FileHash` 找不到，直接调用 PowerShell 却正常 | PowerShell 7 经 CMD 启动 Windows PowerShell 时继承了不兼容的 `PSModulePath` | `Install-ParamCADStudio.cmd` 在 `setlocal` 内清空 `PSModulePath`，让 Windows PowerShell 使用自己的模块 |
| 开发机启动正常，担心换机器缺 DLL | 开发环境 PATH、Qt 或系统中已安装的 VS CRT 可能掩盖包内缺失 | 部署 app-local CRT，并保留清洁 PATH 启动验证；若出现 `Missing runtime`，按错误中的 DLL 和来源二进制检查，不关闭校验绕过 |

目录迁移时还同步处理了以下路径配置，后续改变布局必须一起维护：

- Qt 插件迁到 `app/plugins/` 后，`app/qt.conf` 必须保留 `Prefix=.`、`Plugins=plugins`。
- 嵌入式 Python 的 `python*._pth` 加入 `../../core/reconstruction`，使其能导入包内模块。
- GUI 放入 `app/` 后，包根通过 `core/reconstruction/reconstruct.py` 标记定位；不要只移动或分发 GUI EXE。
- 启动入口设置工作目录为包根；安装后快捷方式指向 `app/ParamCAD Studio.exe`，工作目录为安装根。

其他常见失败：输入缺失时准备本地文件或指定对应参数；Python 哈希不符时换回与 `dependencies.json` 匹配的完整 ZIP；COLMAP flavor 不符时提供正确运行时。构建出现文件占用错误时先关闭对应客户端，重新执行默认命令。不要用未通过检查的旧 ZIP 作为本轮成功产物。

## 8. 发布包布局和使用说明

```text
ParamCAD-Studio-<版本>-windows-x64-full-cuda/
├── Start-ParamCADStudio.cmd       直接启动
├── Install-ParamCADStudio.cmd     可选的离线安装入口
├── README.md                     用户使用说明
├── package-manifest.json         版本、构建来源、依赖信息、文件校验值
├── app/                          客户端 EXE、运行库和 qt.conf
│   └── plugins/                  Qt 插件
├── deps/
│   ├── python/                   嵌入式 Python
│   └── colmap/                   COLMAP 运行时
├── core/reconstruction/          图片到点云的脚本
├── installer/                    离线安装 PowerShell 脚本
└── docs/                         第三方说明和许可证
```

使用者完整解压后双击 `Start-ParamCADStudio.cmd`，或运行 `app/ParamCAD Studio.exe`。可选双击 `Install-ParamCADStudio.cmd`，默认安装到 `%LOCALAPPDATA%/Programs/ParamCAD Studio` 并创建开始菜单快捷方式，无需管理员权限。自定义位置：

```powershell
.\Install-ParamCADStudio.cmd -Destination "D:\Applications\ParamCAD Studio"
```

安装只校验和复制已有文件，不调用网络下载或独立运行库安装程序。升级前关闭客户端；安装脚本保留已有 `outputs/`，通过临时目录和备份目录完成替换，操作失败时尝试恢复原安装。用户工程应保存在 `outputs/` 或安装目录之外。

本包使用 CUDA 版 COLMAP，稠密图片重建需要兼容的 NVIDIA GPU 和驱动；包内不含硬件驱动。更详细的客户端功能说明见 [PACKAGE_README.md](PACKAGE_README.md)。

## 9. 脚本分工与本次验证记录

| 文件 | 职责 |
| --- | --- |
| [package-client.ps1](package-client.ps1) | 构建、组包、校验、压缩、生成外部 SHA-256 |
| [Runtime-Helpers.ps1](Runtime-Helpers.ps1) | 路径边界检查、PE 依赖检查、清洁 PATH 运行验证；仅在打包机使用 |
| [Test-Package.ps1](Test-Package.ps1) | 验证 staging、解压目录或安装目录；仅在打包机使用 |
| [Install-ParamCADStudio.ps1](Install-ParamCADStudio.ps1) | 包内离线安装、升级、保留 outputs 和失败恢复 |
| [Install-ParamCADStudio.cmd](Install-ParamCADStudio.cmd) | 包根安装入口，处理 Windows PowerShell 模块环境 |
| [Start-ParamCADStudio.cmd](Start-ParamCADStudio.cmd) | 包根启动入口，设置工作目录和 Qt 插件环境 |
| [dependencies.json](dependencies.json) | 预期依赖版本、Python 离线输入文件名和校验值；完整打包不使用其中的下载 URL |
| [PACKAGE_README.md](PACKAGE_README.md)、[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md) | 发布包内使用说明和第三方说明 |

2026-10-08 首次打包（0.1.0）的验证结果，保留作排错记录：

- 包：`ParamCAD-Studio-0.1.0-windows-x64-full-cuda.zip`，366,864,685 字节，349.87 MiB；解压 payload 约 720 MiB。
- SHA-256：`67185f52489f426530ad0354ee6a035edecbc7d375133534d66b33fbbb2ee89f`。这是本次产物的记录；下次重打包应使用新生成的 `SHA256SUMS.txt`。
- ZIP 335 个条目，manifest 校验 334 个 payload 文件，manifest 本身不列入自身哈希。
- 清洁 PATH 下 GUI、Python、重建 CLI 和 COLMAP 启动检查通过。
- 含中文、空格路径的实际 ZIP 解压及校验通过。
- 通过 CMD 入口离线安装、升级保留 `outputs/` 通过。
- 损坏测试副本被拒绝安装，已有安装和 `outputs/` 保留。
- PowerShell 脚本语法检查和 `git diff --check` 通过。

本地日志位于 `artifacts/`：`packaging-full.log`、`release-extract-final-test.log`、`release-upgrade-test.log`、`release-integrity-rejection-test.log`、`release-installed-final-test.log`。日志和包不纳入 Git；后续运行可能覆盖日志，排查时以本轮执行结果为准。

2026-10-08 更新为 0.2.0 的验证结果：

- 已同步客户端 CMake、applicationVersion、About 显示及打包脚本默认版本，并重新构建 Release；客户端 `--version` 返回 `ParamCAD Studio 0.2.0`。
- 当前包：`artifacts/releases/0.2.0/ParamCAD-Studio-0.2.0-windows-x64-full-cuda.zip`，366,864,683 字节，349.87 MiB。
- SHA-256：`c1279faa220ec67fe0e40bd50ec3407e229a882911ab2c7d0a874ba4ae1ab265`。
- 实际 ZIP 解压到含中文和空格的目录后，334 个 payload 文件哈希、GUI DLL 依赖及清洁 PATH 启动检查通过。
- 在隔离安装目录完成 `0.1.0 → 0.2.0` 离线升级，manifest 版本更新为 `0.2.0`，已有 `outputs/` 保留，安装后的完整校验通过。
- 本轮日志：`artifacts/packaging-0.2.0.log`、`release-extract-0.2.0-test.log`、`release-upgrade-0.2.0-test.log`、`release-installed-0.2.0-test.log`，其中后三个文件同样位于 `artifacts/`。
