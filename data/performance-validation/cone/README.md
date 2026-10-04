# cone

完整圆锥：底面半径 12 mm，底面直径 24 mm，高度 30 mm，顶端半径 0。

单位为毫米。质心位于原点，轴沿 Z 方向；底面 z = -7.5 mm，顶点 z = 22.5 mm。

`case.json` 记录尺寸、实体数量和验收阈值，`model.step` 为独立构造的标准实体，`model_report.json` 记录有效性、解析体积和 STEP 回读检查。

使用 `python tools/generate_standard_models.py --cases cone` 重新生成。运行 `python tools/run_curved_fit_tests.py --cases cone` 验证重建；生成的点云、重建 STEP、诊断和日志位于 `outputs/curved-fit-tests/`。
