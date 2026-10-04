# torus

完整圆环体：大半径 25 mm，管半径 6 mm；外径 62 mm，内径 38 mm，轴向厚度 12 mm。

单位为毫米。圆环中心与质心位于原点，轴沿 Z 方向。

`case.json` 记录尺寸、实体数量和验收阈值，`model.step` 为独立构造的标准实体，`model_report.json` 记录有效性、解析体积和 STEP 回读检查。

使用 `python tools/generate_standard_models.py --cases torus` 重新生成。运行 `python tools/run_curved_fit_tests.py --cases torus` 验证重建；生成的点云、重建 STEP、诊断和日志位于 `outputs/curved-fit-tests/`。
