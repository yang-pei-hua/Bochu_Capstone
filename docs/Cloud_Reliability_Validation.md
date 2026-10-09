# 点云可靠性阶段：2026-10-10

本阶段建立独立输入、独立 CAD 真值和失败诊断，并修复已测出的预处理问题。工作区内尚无带 CAD 真值的真实机械扫描；本阶段的通过结果仅代表下述合成和虚拟拍摄数据。

## 测试范围与验收

`tools/run_cloud_reliability.py` 使用解析方程独立采样，调用原有验证程序独立生成 STEP 真值，再调用生产重建入口。真值用于比较结果，不用于补点、补面、恢复法向或设置拟合参数。固定采样/RANSAC 种子默认为 4500。

覆盖长方体、旋转球/圆柱/圆台/圆环体、带加强筋的 L 支架和带贯穿孔长方体，每类包含七种输入：

| 条件 | 输入改变 | 验收要求 |
| --- | --- | --- |
| clean | 完整表面与定向法向 | STEP 回读、尺寸和对称差体积比较通过 |
| noise | 坐标独立高斯噪声，σ=0.02 mm | 同上，不放宽原有真值容差 |
| outliers | 增加约 1% 无法向点，坐标范围为 ±100 倍真值对角线 | 同上 |
| missing-normals | 固定种子随机移除约 80% 法向 | 同上，保持贯穿孔内壁和凹面语义 |
| no-normals | PLY 完全不含法向属性 | 准确重建或明确拒绝；当前实现明确拒绝 |
| nonuniform | x<0 的半侧仅保留约 30% 观测 | 同上，仍保留所有面观测 |
| incomplete | 移除完整外侧面/端面，或仅保留半球/半环 | 明确拒绝，并且本轮没有 STEP 输出 |

长方体/组合体继续使用原有 0.5 mm 尺寸与 5% 对称差容差；独立曲面体使用 0.2 mm 外形、0.1 mm 图元尺寸及 3% 对称差容差。错误尺寸、超时、非零退出、原生崩溃、残留旧 STEP 均不能算作通过。报告记录点云/执行程序 SHA256、输入来源、种子、耗时、预处理统计和精度指标。重跑时旧输出归档，避免把上次成功结果当作本轮结果。

## 结果

先在修改前的 0.2.1 验证程序上运行五类基础样例，共 35 项；同一输入和同一容差在修复后重复验证。

| 基础条件 | 修改前 | 修改后 |
| --- | --- | --- |
| 完整、噪声、非均匀采样 | 15/15 精度通过 | 15/15 精度通过 |
| 远处离群点 | 0/5 精度通过 | 5/5 精度通过 |
| 80% 法向缺失 | 0/5 精度通过 | 5/5 精度通过 |
| 无法向及缺面 | 10/10 明确拒绝 | 10/10 明确拒绝 |
| 全部验收契约 | 25/35 | 35/35 |

追加 L 支架/贯穿孔的 14 项通过：10 项精度通过、4 项明确拒绝。合成集共 **49/49** 符合契约，其中 35 项精度通过、14 项明确拒绝，意外 STEP 输出为零。

已有两份虚拟拍摄 → COLMAP 稠密重建点云也精度通过，原始输入分别为 675,791 和 544,302 点。这两项单独标记 `virtual-capture`，不能用于宣称真实机械照片/扫描泛化通过。

本机报告位于 `artifacts/validation/cloud-reliability-before/summary.json`、`cloud-reliability-after/summary.json` 和 `cloud-reliability-virtual/summary.json`。构建和回归涵盖客户端 18 项 CTest、独立几何核心测试以及工具层 30 项 Python 测试。

## 修复与边界

客户端使用去掉两端各 1% 坐标观测后的范围选择体素尺寸，避免远处噪点把零件压缩为少数体素。这一步只选择分辨率，原始点仍进入常规过滤。统计过滤增加基于近邻距离中位数/MAD 的阈值上限，避免极远噪点掩盖较近的离群点；用户模板选中的点保持受保护。

PCA 不再给重合点和共线邻域制造法向。已有输入法向只归一化，保留方向；缺失法向的方向沿近邻图从已有定向观测传播。对齐、切向偏差和距离/间距比例均为可配置的无量纲阈值；图搜索用 `PointId` 打破等距平局，保持源点身份。没有定向锚点或无法可靠连通的法向保留未定向状态，自动实体入口明确拒绝。薄壁两侧、凹面和孤立区域不能通过“朝向世界轴/点云质心”来猜测材料方向。

输入法向的方向仍被视为可信，当前不修正全部反向或相互矛盾的输入法向。本阶段没有恢复看不见的面，也没有完成任意遮挡、任意非均匀密度、任意离群比例或任意噪声强度下的重建。现实相机/扫描噪声需要实测数据继续校准。

## 复现

先构建带 `CAD_CLIENT_BUILD_VALIDATION_TOOLS=ON` 的 Release 客户端，默认验证程序位于 `apps/ParamCAD Studio/build/msvc-debug/Release/`，也可用 `--engine` 指定。

```powershell
python tools/run_cloud_reliability.py --output artifacts/validation/cloud-reliability-after
python tools/run_cloud_reliability.py --manifest data/cloud-reliability/virtual-capture.json --output artifacts/validation/cloud-reliability-virtual
```

`--kinds`、`--conditions` 可选择子集，`--seed` 可指定采样和 RANSAC 种子。合成参数、完整拟合报告、STEP、日志和每项契约判断均保存在各个样例目录。

真实扫描可使用如下清单；三个文件路径相对于清单解析，`id` 必须为单个目录名，必须有独立 STEP 真值和尺寸/验收定义，才能进行精度判断：

```json
{
  "schema": "paramcad.recorded-cloud-dataset.v1",
  "cases": [{
    "id": "measured-part-01",
    "provenance": "real-capture",
    "source": "填写采集设备、日期和实际拍摄/扫描来源",
    "cloud": "measured/cloud.ply",
    "reference": "measured/reference.step",
    "definition": "measured/case.json",
    "expectation": "accurate"
  }]
}
```

尺寸/验收定义沿用 `data/performance-validation/*/case.json` 格式，`unit` 为 `mm` 或 `m`，尺寸验收项以毫米记录。参考 STEP 使用现有客户端加载逻辑按体积质心居中，点云必须预先对齐到相同位置和朝向；评测不使用真值做自动配准。没有 CAD 真值的户外 Fountain-P11 可用于 SfM/曲面检测探针，不能替代机械实体精度基准。

下一步需加入带已知尺度和独立真值的真实机械采集数据；在现有结构化参数编辑入口之上实现最小文本修改命令时，应继续保留这套精度与拒绝回归。
