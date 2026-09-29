# Layer 2：无界面多视角点云重建

本目录实现 `docs/project_definition.md` 中的 Layer 2：输入多视角图片和可选
`CameraInfo`，输出彩色 PLY 点云。重建引擎为官方 COLMAP 4.2.0，固定于提交
`be5e29168d4aff238409d60424812df66aac919f`，源码以浅克隆方式保存在
`third_party/colmap`。运行路径只有命令行，不依赖 COLMAP GUI。

## 能力

- 不提供相机信息：COLMAP 自动估计内参和外参。
- 只提供内参：使用给定标定，自动估计外参。
- 提供全部外参：固定给定相机位姿并三角化点云。
- 提供部分外参（至少两张）：先三角化种子模型，再固定已有位姿继续注册其余图片。
- 默认输出稀疏彩色 PLY；`--dense` 输出带法向量的稠密融合 PLY。

没有给定外参时，SfM 的尺度是不确定的。给定外参时，点云沿用外参平移的单位；
如果项目采用毫米，请传 `--input-unit mm`。每次运行还会生成
`reconstruction.json`，记录坐标系、单位、点数和输入路径。

## 构建 COLMAP

Windows 预编译包的完整下载、安装与验证流程见
[`docs/colmap_installation.md`](../../docs/colmap_installation.md)。

COLMAP 的第三方依赖较多。Windows 推荐使用 Visual Studio + vcpkg；Linux 可先按
`third_party/colmap/doc/install.rst` 安装系统依赖。随后在 PowerShell 中运行：

```powershell
# CPU 版：支持特征、匹配、SfM 和稀疏点云
.\core\reconstruction\build_colmap.ps1

# CUDA 版：在上述能力之外支持 --dense
.\core\reconstruction\build_colmap.ps1 -Cuda
```

构建脚本明确设置 `GUI_ENABLED=OFF`、`OPENGL_ENABLED=OFF`，不会构建交互页面。
也可以不构建本目录的源码，直接通过 `--colmap` 指向兼容的 COLMAP 4.2 可执行文件。

## 使用

自动估计相机并输出稀疏点云：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images `
  --output .\data\part_sparse.ply
```

提供内参和全部/部分外参：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images `
  --camera-info .\data\camera_info.json `
  --input-unit mm `
  --output .\data\part_sparse.ply
```

有 CUDA/HIP 支持的 COLMAP 可输出稠密点云：

```powershell
python .\core\reconstruction\reconstruct.py `
  --images .\data\part_images `
  --camera-info .\data\camera_info.json `
  --input-unit mm `
  --dense --gpu `
  --output .\data\part_dense.ply
```

有顺序的视频帧可加 `--matcher sequential`。默认关闭 GPU 特征计算，保证 CPU 版
也能运行；`--gpu` 同时开启 SIFT 特征提取和匹配的 GPU 路径。

## CameraInfo JSON

参考 `camera_info.example.json`。图像名必须是相对于 `--images` 的路径，且集合与
目录中 COLMAP 可读取的图像完全一致。内参模型和参数顺序遵循 COLMAP，例如：

- `SIMPLE_PINHOLE`: `[f, cx, cy]`
- `PINHOLE`: `[fx, fy, cx, cy]`
- `OPENCV`: `[fx, fy, cx, cy, k1, k2, p1, p2]`

位姿可逐张省略。支持两种明确的坐标约定：

- `world_to_camera`: COLMAP 原生的 Hamilton 四元数 `qvec=[qw,qx,qy,qz]` 和
  `tvec`，满足 `x_camera = R * x_world + t`。
- `camera_to_world`: 相机朝向四元数和世界坐标中的相机中心 `position`；适合多数
  渲染器导出的相机变换，适配层会转换为 COLMAP 外参。

COLMAP 相机局部坐标为 X 向右、Y 向下、Z 向前。项目要求右手坐标系；若上游使用
Y 向上或 Z 向上的渲染坐标，应在生成 CameraInfo 时先做轴变换。

## 输出与故障排查

- 点云：`--output` 指定的 `.ply`。
- 清单：工作目录中的 `reconstruction.json`。
- 完整引擎日志：工作目录中的 `colmap.log`。
- 中间数据库、稀疏模型、稠密深度图：均保留在工作目录中，便于复现。

为防止误覆盖，工作目录只允许为空。重复运行时请指定新的 `--workspace`。
如果重建失败，首先检查图片是否有足够纹理和视角重叠、相机内参尺寸是否准确、
外参约定是否正确。稠密模式还要求 COLMAP 编译时启用了 CUDA 或 HIP。

## 测试

```powershell
$env:PYTHONPATH = ".\core\reconstruction"
python -m unittest discover -s .\core\reconstruction\tests -v
```

## 第三方许可

COLMAP 使用 BSD 3-Clause 许可证，原始许可证位于
`third_party/colmap/LICENSE`。实际分发二进制时还需检查 Ceres、PoseLib、SiftGPU、
VLFeat、FAISS、OpenImageIO 等依赖的许可证和引用要求。
