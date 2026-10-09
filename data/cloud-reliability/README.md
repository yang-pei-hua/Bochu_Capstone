# 点云可靠性数据入口

`virtual-capture.json` 引用工作区内已保存的两份虚拟拍摄 → COLMAP 稠密重建点云。文件本体位于被 Git 忽略的 `outputs/`；它们不是现实拍摄数据。缺少本地文件时，评测会在运行前报告缺失路径。

新增实测数据清单需声明 `provenance: real-capture`、采集来源 `source`、点云 `cloud`、独立 STEP 真值 `reference`、尺寸/验收定义 `definition` 和预期 `expectation`。路径相对于清单文件解析。每份输入必须注明毫米或米，坐标系须与真值加载后的坐标系一致。

完整样例、运行命令与结果见 [评测说明](../../docs/Cloud_Reliability_Validation.md)。当前仓库没有带独立 CAD 真值的真实机械扫描基准。
