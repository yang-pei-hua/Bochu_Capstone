# sphere

球：半径 15 mm，直径 30 mm。

单位为毫米。球心与质心位于原点。

`case.json` 记录尺寸、实体数量和验收阈值，`model.step` 为独立构造的标准实体，`model_report.json` 记录有效性、解析体积和 STEP 回读检查。

使用 `python tools/generate_standard_models.py --cases sphere` 重新生成。运行 `python tools/run_curved_fit_tests.py --cases sphere` 验证重建；生成的点云、重建 STEP、诊断和日志位于 `outputs/curved-fit-tests/`。
