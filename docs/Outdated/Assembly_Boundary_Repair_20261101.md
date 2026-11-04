# 边界有效性与候选显示修复：2026-11-01

## Git 基线与范围

修改前的可构建基线和上一轮诊断已提交为 `7f824e6`：
`chore(reconstruction): save baseline and four-workpiece boundary diagnostics`。
`yht` 分支保留该基线；修复在 `codex/fix-assembly-boundaries` 分支实施。
原始诊断见 [问题分析](Assembly_Boundary_Investigation_20261101.md)。

本轮修复轮廓自交/退化、静默丢环及界面全局杂线叠加。
部件分组和已有 CAD 参数保持不变，尚未提高复杂工件闭合数量。
没有改大生产 alpha 半径或焊接容差，也没有读取参考 STEP/标签补面。

## 实现

- 将平面轮廓几何操作集中到私有 `recognition/assembly/BoundaryGeometry.h/.cpp`，
  不向公共接口暴露 CGAL 类型。
- alpha 边界沿顶点自接触时拆成独立简单环，保留有向面积；无法形成简单环的结果记录诊断。
- 折线简化改用点到有限线段的距离，检查简单性、方向和面积变化。
  简化面积变化不超过 5%，失败时减小简化尺度或保留原环。
- 短边与长边均检查吸附方向及两个端点到交线的距离；交点必须靠近当前角点，
  不能凭靠近轮廓另一端的样本授权外推。
- 吸附出现平行冲突、自交、退化、方向翻转或大幅面积变化时，回退整环。
  约束合并后不足三条线时也保留原环，不再直接丢弃。
- 循环直线段的首尾合并使用实际过渡点，避免起点落在边中间时错误拒绝角点恢复。
- 简化和吸附均检查多环之间的包含及接触关系，避免孔跑到外环之外或跨环相交。
- 界面默认显示已闭合部件和当前选中集合的边界；“显示全部候选边界”保留完整诊断入口。
  在远小于拟合容差的数值显示精度下对公共边去重，保持点云/几何不变。
- 编辑点云或切换工程后清理旧边界；显示范围切换不改点归属，不触发重建。

`boundary_recovery` 中的 `non_simple_*` 统计候选尝试，失败尝试即使已回退也会计入；
不能把该计数当作最终发布轮廓的缺陷数。新增 fallback/split 计数说明实际采用的回退。

## 四份工件的最终结果

最终后台复测：`outputs/boundary-repair-20261101/final/summary.json`。
前后对比与一致性验证：同级 `comparison.json`。
GUI 实测：`outputs/boundary-repair-20261101/gui-final/summary.json`。

| 工件 | 严格自交轮廓：基线 → 修复 | 闭合实体：基线 → 修复 | 原始轮廓线段：基线 → 修复 | 最终默认显示边段 | 最终全部边段（去重） |
| --- | ---: | ---: | ---: | ---: | ---: |
| box-wall | 19 → 0 | 8 → 8 | 5533 → 12932 | 96 | 12836 |
| box-wall-1 | 14 → 0 | 6 → 6 | 3564 → 10689 | 84 | 10605 |
| cover-plate | 15 → 0 | 5 → 5 | 4608 → 9513 | 60 | 9452 |
| oil-tank | 10 → 0 | 0 → 0 | 4039 → 11431 | 0 | 11431 |

原始线段总数增加：不再用退化压缩或丢环减少边界，一部分不适合简化的采样轮廓会保留更多边段。
它与默认渲染数是不同指标。全部边界模式仍可能密集，不应声称边界恢复已经完全准确。
壳开放边数分别由 2983、2358、2705、2744 变为 6065、6785、5943、9158；
保留更多有效观测轮廓并没有解决它们之间的公共边拼接。

四份点云的候选点 ID、面片归属和连接图与基线完全相同。
前三份导出的 `model.features.json` 与基线逐字段相同；B-Rep 有效性、STEP 回读和 PCAD 重开均通过。
oil-tank 明确返回 unsupported，未生成虚假 CAD；整批仍是 incomplete。

输入文件头标记为实际工件 STEP 的解析采样点云，不代表实物扫描噪声验收。

## 客户端验收与使用

四份点云均实际触发窗口的 Reconstruct CAD 按钮，验证：

1. 默认边界显示，选中开放候选，以及切换全部边界。
2. 候选未因显示过滤而消失，开放/闭合状态正确保留。
3. 可闭合部分的实际参数修改、撤销、STEP 导出及项目保存重开。
4. oil-tank 保留候选和诊断，不输出实体。

各案例目录包含 `separation.png`、`selected-boundaries.png`、`all-boundaries.png`、
`boundary-display.json` 和测试日志；前三份另有 GUI 导出的 STEP，四份都有保存的 PCAD。
已保存工程的边界叠加仍是即时状态，重开后需重新重建才会恢复。

当前原客户端进程占用 `ParamCAD Studio.exe`，未关闭或覆盖它。
可运行同目录的本轮独立构建：

```powershell
python tools/pcad.py studio --exe "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/ParamCADStudioBoundaryFix.exe"
```

正常主程序源代码与测试窗口使用同一套实现；独立 exe 通过 MSBuild 改 TargetName 构建，
复用已有运行库，禁用重复部署事件以避免覆盖正在运行的 DLL。

## 测试

- `assembly_reconstruction_test`：新增自接触环拆分、薄壁吸附退化回退、真实交线角点、
  轮廓起点变化、过度简化和孔包含关系检查；既有缺盖面、窄间隙、旋转、噪声、曲面实体回归继续通过。
- `cad_reconstruction_task_test`、`parameter_editing_test`：通过。
- 两项 `test_assembly_boundary_diagnostics.py`：通过。
- 同一个 `parameter_editing_test.exe` 再对四份输入分别执行完整 GUI 验收，四份退出码均为 0。

```powershell
ctest --test-dir "apps/ParamCAD Studio/build/msvc-debug" -C Release `
  -R "^(assembly_reconstruction_test|cad_reconstruction_task_test|parameter_editing_test)$" --output-on-failure
python -m unittest discover -s tools/tests -p test_assembly_boundary_diagnostics.py -v

$env:PCAD_REAL_CLOUD="D:/code/Bochu_Capstone/outputs/boundary-repair-20261101/final/cover-plate/input/cloud.ply"
$env:PCAD_REAL_GUI_OUTPUT="D:/code/Bochu_Capstone/outputs/boundary-repair-repeat-gui"
& "apps/ParamCAD Studio/build/msvc-debug/cloud-only/Release/parameter_editing_test.exe"
```

## 后续闭合工作

下一步需要以相邻曲面的共同交线及观测区间建立共享端点、共享边和 T 接头，
再接入圆柱/圆锥等有限裁剪环。当前多数开放候选缺少这种跨面拓扑，
部分候选本身只有一个平面；本轮没有把这些观测强行封盖。
