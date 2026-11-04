# 纯点云的部件分离、有限边界与部分 CAD 重建

重建只接收位置、可选法向、单位以及用户手动指定的模板／关系。
不读取原 STEP、参考标签、原装配实例、原裁剪边界或邻接拓扑；没有按文件名、案例尺寸选择算法的分支。
普通 STEP 导入／导出、已有 PCAD 参数模型重建继续保留。STEP 回读仅用于检查刚导出的结果。

## 当前功能

打开 PLY，点击 **Reconstruct CAD** 后：

1. 空间邻域、局部法向和拟合平面约束产生曲面片及部件候选。候选点集着色显示在右侧「点云 → 集合」。
2. 平面点集通过二维 Delaunay／alpha 边界恢复有限轮廓，可保留采样支持的凹口与孔洞。相邻平面交线约束边界，连接图记录曲面关系。
3. 对受观测支撑的有界凸平面组，使用半空间及平面交点恢复共同顶点；缺少盖面、存在无界方向或内部观测冲突时拒绝闭合。
4. 有限边图检查开放边、非流形边、方向和共享顶点；通过后构造 B-Rep 并验证有效性。局部解析曲面还会检查圆周与端盖支撑。
5. 通过闭合验证的部分成为独立 CAD 特征，边界显示绿色；其余保留为点云候选和橙色边界，不凭空补盖面。

平面壳是可编辑的 `Profile Body`，可修改整体 X/Y/Z 尺寸与姿态，撤销编辑、保存 PCAD 并导出 STEP。
这不等于恢复原始设计历史或每个孔、倒角的独立特征。任意有限边界尚未提供逐顶点／逐面交互编辑。
简单解析零件继续使用已有尺寸、半径等参数编辑器。

## 四份真实点云的实测

最终验证结果：`outputs/assembly-point-cloud/verified/`。隔离输入目录只有 `cloud.ply` 与单位 JSON。
候选数不代表原装配零件数量：

| 点云 | 输入点数 | 分离候选 | 曲面片 | 验证闭合的 CAD 实体 | 未闭合候选 |
| --- | ---: | ---: | ---: | ---: | ---: |
| box-wall | 799439 | 55 | 661 | 8 | 47 |
| box-wall-1 | 633154 | 52 | 556 | 6 | 46 |
| cover-plate | 921272 | 171 | 926 | 5 | 166 |
| oil-tank | 518552 | 51 | 340 | 0 | 51 |

前三份输出 `model.pcad`、`model.features.json`、`model.step`，均通过有效 B-Rep、STEP 回读与 PCAD 保存重开检查。
oil-tank 仍只输出分离与边界诊断，没有借用历史模型补齐。
整批状态为 `incomplete`，前三份为 `cad_exported_partial`，oil-tank 为 `unsupported`。

每份还包括 `segmented.ply`（原点与法向、候选颜色和 `part_id`，0 为未归属）、`parts.json`（候选点 ID、曲面片、有限轮廓、连接、开放边及失败原因，几何单位 mm）和 `report.json`。

报告历史字段 `surface_coverage` 在分离流程中表示「归属于闭合候选的观测点比例」，前三份约为 12.16%、11.51%、4.36%。
它不是点到曲面误差、原模型恢复率或几何精度；新版报告用 `coverage_basis` 与 `assigned_observation_fraction` 明确语义。
无源模型时，`source_completeness_verified` 始终为 false。
此前 134 个实体通过的 STEP 辅助实验是历史结果，不能作为纯点云能力的证明。

## 启动与复现

在仓库根目录运行 `python tools/pcad.py studio`，启动器优先选择本轮构建：

```powershell
& "D:\code\Bochu_Capstone\apps\ParamCAD Studio\build\msvc-debug\cloud-only\Release\ParamCAD Studio.exe"
```

1. **File → Open Point Cloud**，打开 `outputs/assembly-point-cloud/verified/cover-plate/input/cloud.ply` 或自己的 PLY。
2. 点击 **Reconstruct CAD**，等待日志出现候选与闭合数量。
3. 「点云 → 集合」可选择各部件候选；绿色为闭合边界，橙色为开放或未验证边界。
4. 在左侧 CAD 列表选择 `Profile Body`，在「特征」面板编辑整体尺寸／姿态，保存 PCAD 或导出 STEP。
5. 只想看部分 CAD，可打开同目录的 `model.pcad`。批处理生成的工程只含 CAD；客户端自行保存的工程还包含点云和候选归属。

保存工程保留候选点归属及闭合状态。边界叠加线当前是即时派生显示，重开工程后需再次重建才能恢复叠加线。
PLY 无尺度时需声明单位；单位 sidecar 只需 `{"unit":"mm"}` 或 `{"unit":"m"}`。
`reference.json`、`summary.json` 和邻近原 STEP 不作为重建输入。

任意扫描点云可用同一后台任务批处理：

```powershell
python tools/run_point_cloud_cad.py --input scan=D:/scans/scan.ply --unit mm --output outputs/scan-cad-new
```

可重复 `--input NAME=FILE.ply`，`--timeout 300` 控制单份时限；输出目录需新建或为空。
复现四份案例可分别添加 `outputs/point-cloud-only-real-workpieces/<案例>/input/cloud.ply`。
若任何案例未生成 CAD，批处理返回 1，但保留已成功的部分与所有诊断。

## 验证与限制

核心测试直接生成观测点，覆盖分离与旋转盒体、乱序及改点 ID、缩放平移、任意三维旋转、噪声、窄间隙、无输入法向、非正交棱柱、孔洞／凹边界、缺盖面及局部圆柱。
客户端测试实际触发按钮，检查完整与缺面部件混合输入、删除点后的 ID 映射、候选集合、边界叠加、参数修改／撤销、保存重开与导出。
相邻损坏的参考表与失效原 STEP 路径不影响重建。

```powershell
ctest --test-dir "apps/ParamCAD Studio/build/msvc-debug" -C Release -R "^(assembly_reconstruction_test|cad_reconstruction_task_test|parameter_editing_test)$" --output-on-failure
```

可选真实点云客户端验收不提供固定部件数量或尺寸：

```powershell
$env:PCAD_REAL_CLOUD="D:/code/Bochu_Capstone/outputs/assembly-point-cloud/verified/cover-plate/input/cloud.ply"
$env:PCAD_REAL_GUI_OUTPUT="D:/code/Bochu_Capstone/outputs/assembly-point-cloud/gui-cover-plate"
& "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/parameter_editing_test.exe"
```

验收保存 `separation.png`、包含观测及候选的 `observed.pcad`，以及通过验证部分的 `observed.step`。
本轮 cover-plate 客户端验收已通过：171 个候选、5 个实体、4608 条边界显示线段，并完成实际参数编辑、撤销、导出与工程重开。
最终批处理还逐点核对了候选归属与着色 PLY，检查无重复归属、已归属及未归属总数等于原始点数，输入校验和一致。

接触／重叠装配仍可能合并，复杂凹件可能被拆开，薄壁及大曲率变化处可能漏分／碎分，存在未归属点。
任意相交曲面／自由曲面的裁剪与全局闭合尚未完成；圆角、螺纹和完整孔系也未普遍恢复。
本轮完成不依赖原 STEP 的基本分离、有限边界、连接与可验证部分的闭合，不承诺完整还原复杂装配。
