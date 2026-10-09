# ParamCAD Studio 完整离线版

适用于 Windows 10/11 x64。GUI、Qt/VTK/OpenCASCADE、MSVC 运行库、Python 和 COLMAP 已全部包含在包内。启动和安装均不下载任何依赖，不需要额外安装 Python、Qt、COLMAP 或开发 SDK。

## 启动

完整解压 ZIP 后，双击根目录的 **Start-ParamCADStudio.cmd**。也可以直接运行 `app/ParamCAD Studio.exe`。保留各文件夹的相对位置，不要仅复制 EXE。

## 安装（可选）

双击 **Install-ParamCADStudio.cmd**。安装程序只验证、复制包内文件并创建开始菜单快捷方式，无需管理员权限。

默认安装位置：`%LOCALAPPDATA%\Programs\ParamCAD Studio`。

自定义安装位置：

```powershell
.\Install-ParamCADStudio.cmd -Destination "D:\Applications\ParamCAD Studio"
```

升级前关闭已运行的客户端。升级保留安装目录中的 `outputs/`；操作失败时保留或恢复原安装。

## 目录说明

```text
ParamCAD-Studio-<版本>-windows-x64-full-cuda/
├── Start-ParamCADStudio.cmd       启动入口
├── Install-ParamCADStudio.cmd     可选的离线安装入口
├── README.md                     使用说明
├── package-manifest.json         版本、构建来源、依赖信息及文件校验值
├── app/                          客户端 EXE、DLL、Qt 插件与路径配置
│   └── plugins/                  Qt 平台、图像格式等插件
├── deps/
│   ├── python/                   内置 Python，不需要 pip 安装
│   └── colmap/                   完整 COLMAP 运行时
├── core/reconstruction/          图片到点云的重建脚本
├── installer/                    离线安装脚本
└── docs/                         第三方说明与许可证
```

运行时生成的截图、拍摄图片和重建结果位于根目录的 `outputs/`。请解压或安装到可写位置。

## 常用功能

- **File → Open Point Cloud**：打开 PLY 点云。
- 工具栏 **Reconstruct CAD**：由点云重建参数化 CAD。
- **Properties → 点云**：选点、管理集合、分析关系及调整容差。
- 点击左侧 **CAD 特征**：在 **Properties → 特征** 查看实际拟合参数。
- **File → Save / Export STEP**：保存工程或导出 CAD。
- **Reconstruct Point Cloud**：由多视图图片重建点云。

此包使用 CUDA 版 COLMAP，稠密图片重建需要兼容的 NVIDIA GPU 和驱动；点云查看、曲面识别及 CAD 重建使用客户端自身的运行库，不需要 Python 安装命令。包内不包含硬件驱动。

ZIP 外的 `SHA256SUMS.txt` 用于核对下载文件。`package-manifest.json` 记录本包实际构建信息；安装程序会校验包内文件。

## 参数编辑与面上草图

在 CAD 特征列表选择 Box、贯穿孔或圆柱，进入 Properties 的特征页编辑尺寸；
支持参照面/中心、应用/取消、当前会话内 CAD 撤销/重做。手动修改会阻止自动重拟合覆盖。
选中当前参数模型的 Box 平面或圆柱端面后创建草图，草图及下游拉伸/切除会随上游尺寸更新。
导入 STEP 和归属不唯一的面使用固定平面，状态栏会提示。
`.pcad` v2 保存语义面引用、尺寸与手动修改标记，可读取旧 v1 工程；旧客户端不能读取 v2。
