# 系统架构

下图概括项目的端到端流程。

```mermaid
flowchart LR
    images[多视角图像] --> cloud[点云重建]
    cloud --> recognition[几何与特征识别]
    recognition --> cad[参数化 CAD]
```

后续在此补充生产流程、数据交接和坐标系约定。
