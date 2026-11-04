# Cover-plate STEP 曲面识别验证

## 输入与验收

输入：`data/real-workpieces-step/cover-plate/cover-plate.step`。
展开实例后有 45 个实体、1007 个面：261 个平面、490 个圆柱面、256 个圆锥面。
STEP 文本中只有 22 个实体定义；重复装配实例不能按定义数漏计。

采样生成 921272 个毫米单位观测，包含解析位置和有向法向。每面保留
256–2048 点，避免大平面耗尽内存或小倒角缺乏支持。三角网格仅用于
在 UV 域分配采样位置，实际点与法向由解析面计算，并通过裁剪面分类器验证。
本次拒绝了 94 个落在网格近似边界外的 UV 样本。

算法只读点的位置、法向和毫米单位元数据。面/实体身份保存在独立参考清单中，
在识别结束后读取。每个参考面同时要求同类型候选覆盖率和最终显示标签覆盖率
至少达到 98%；大面的覆盖率不能补偿小面的漏检。

## 算法修正

- 新增 `detectMultiScalePrimitiveEvidence()`：分别提出全局平面、圆柱和圆锥，
  再从法向相容的局部区域及剩余观测中恢复小曲面。球面和环面在有限局部区域中提出。
  全局候选配额不会阻止其他局部区域继续被识别。
- CGAL 后端使用项目自有圆锥工厂，从法向圆估计轴，从切平面交点估计顶点，
  避免用窄轴向带上的三个生成方向估计轴所导致的病态。候选仍由 CGAL
  Efficient RANSAC 提出；SDK 和 CGAL 安装文件均未修改。
- 对圆锥的全部相容支持重新拟合，并在完整输入上重评支持，保留稳定原始点 ID。
- 在完整连续区域上比较曲面类别，再移除冲突类别的局部近似支持。
  防止大半径环面抢走圆锥标签，或切平面抢走带噪圆柱标签。
- Studio 的分析体素和最小圆柱半径跟随指定拟合容差，避免约 7.6 m
  的整体对角线把 0.05 mm 容差下的小特征消除。全局表面分析不再把不同
  采样密度直接作为离群证据；孤立观测仍必须满足检测支持要求。
- 用法向一致性参与显示标签评分，添加有限分区包围盒筛选，并在关系推断前
  用包围盒排除不可能相邻的区域。
- 分析设置指纹包含局部恢复开关，防止复用不匹配的缓存结果；取消不发布部分结果。

## 结果

距离容差 0.05 mm，种子 4500：

| 路径 | 通过的面 | 所有面均通过的参考实体 | 同类型点覆盖率 |
| --- | --- | --- | --- |
| 核心多尺度检测 | 1007/1007 | 45/45 | 99.99978% |
| Studio 预处理与唯一显示标签 | 1007/1007 | 45/45 | 99.99967% |

位置高斯噪声标准差 0.005 mm、种子 719 的独立重放，核心仍通过
1007/1007 个面及 45/45 个参考实体，点覆盖率 99.99891%。
噪声重放保持原始法向；未据此宣称对照片点云、法向噪声或遮挡同样完整。

当前机器核心一次约 42 s，Studio 分析约 65 s，包含输出报告和着色点云。
实际时间受机器负载影响。生成物：

- `outputs/cover-plate-reconstruction/verified/cloud.ply`
- `outputs/cover-plate-reconstruction/verified/analyze-client.segmented.ply`
- `outputs/cover-plate-reconstruction/verified/summary.json`
- `outputs/cover-plate-reconstruction/noisy-seed-719/summary.json`

逐面明细和候选参数见同目录报告；中间失败报告保留在 `history/`。

## 复现

```powershell
cmake --build "apps/ParamCAD Studio/build/msvc-debug" --config Release --target cad_reconstruction_validation
python tools/run_step_surface_validation.py data/real-workpieces-step/cover-plate/cover-plate.step --output outputs/cover-plate-new
python tools/run_step_surface_validation.py data/real-workpieces-step/cover-plate/cover-plate.step --output outputs/cover-plate-noisy-new --noise 0.005 --seed 719 --skip-client
```

用同样输入参数加 `--replay` 可复用已保存点云，并归档旧报告和着色点云。
源 STEP 和点云的 SHA-256 被记录且在重放时检查。任一面未达到标准时脚本返回非零。

回归包含窄圆锥带、随机点顺序、非连续点 ID、无效法向、局部检测配额、
多类别竞争及取消；客户端包含大平面旁半径 2 mm 的圆柱和原始点映射验证。
既有曲面导出、旋转与位置噪声、不完整曲面拒绝及固定种子复现测试也已验证。

## 范围

完成的是解析曲面检测和观测点分区。45/45 是参考实体的逐面完整性统计，
不表示从点云独立恢复了 45 个装配实例的身份或接触拓扑。
本报告的 `complete_cad_fit` 保持 `false`，因为该阶段仅验证曲面与分区。
源模型辅助的 CAD 重建路径已移除。上述源 STEP 只用于离线采样和事后逐面评测，
重建入口仅接受点云。当前 CAD 使用方式见 [仅点云重建说明](Real_Workpieces_Editable_CAD.md)。
