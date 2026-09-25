# 多视角图像到 CAD 重建系统架构说明

## 1. 项目目标

本项目目标是实现从**零件多视角图片**到**可编辑 CAD 模型**的自动重建流程。

系统采用分层架构，各层彼此独立，仅通过标准化输入输出进行连接。

整体流程：

```text
模型 / 真实零件
        ↓
多视角图像
        ↓
三维点云
        ↓
参数化几何描述
        ↓
CAD 操作
        ↓
SolidWorks 模型
```

同时支持通过自然语言对中间建模结果进行修改，并触发局部或整体重建。

---

## 2. 核心设计原则

系统各层需要满足以下要求：

* 每层只依赖上一层的标准输出，不依赖其内部实现。
* 各层可以独立开发、测试和替换。
* 几何识别与 CAD 软件操作解耦。
* AI 只负责将自然语言转换为结构化修改指令，不直接执行几何计算。
* 所有模块统一坐标系、单位和对象 ID 规范。

推荐统一：

* 几何单位：mm
* 三维坐标系：统一右手坐标系
* 结构化信息：标准序列化数据
* 大型数据：图像、点云、模型文件独立存储

---

# 3. Layer 1：虚拟数据生成

## 3.1 功能

根据已有三维模型、贴图和摄像机参数，生成接近真实拍摄效果的多视角图片。

该层主要用于：

* 数据集生成
* 测试数据生成
* 后续模块验证

具体实现可以使用 Unity、Unreal Engine、Blender 或其他渲染工具。

## 3.2 输入

* 三维模型
* 材质 / 贴图
* 摄像机位置
* 摄像机朝向
* 摄像机内参
* 渲染参数

## 3.3 输出

### ImageSet

多视角图像集合。

### CameraInfo

与每张图片对应的摄像机信息，包括：

* 摄像机内参
* 摄像机外参
* 图像尺寸
* 摄像机 ID

## 3.4 数据接口

```text
3D Model + Texture + CameraConfig
                ↓
            Layer 1
                ↓
ImageSet + CameraInfo
```

---

# 4. Layer 2：多视角三维重建

## 4.1 功能

根据多视角图片恢复物体的三维点云。

该层主要基于现有开源三维重建算法或项目实现。

输入数据既可以来自 Layer 1，也可以来自现实摄像机。

## 4.2 输入

### ImageSet

多视角图片。

### CameraInfo

摄像机信息。

摄像机信息可以：

* 已知并直接提供
* 部分已知
* 完全未知，由 Layer 2 自动估计

## 4.3 输出

### PointCloud

三维点云数据。

基本数据包括：

* 三维坐标 XYZ

可选数据包括：

* RGB
* 法向量
* 置信度

## 4.4 数据接口

```text
ImageSet + CameraInfo
          ↓
       Layer 2
          ↓
      PointCloud
```

---

# 5. Layer 3：点云几何拟合与重建

## 5.1 功能

根据三维点云识别物体结构，并将其拟合为参数化几何模型。

该层是整个系统的核心几何处理模块。

支持的基础几何类型可包括：

* 长方体
* 正方体
* 球
* 圆柱
* 圆锥
* 三棱柱
* 平面

后续可以继续扩展新的几何类型。

## 5.2 输入

### PointCloud

Layer 2 输出的点云。

### ReconstructionConstraint

可选的重建约束，例如：

* 几何体数量
* 几何体类型
* 候选几何类型
* 已知尺寸
* 已知结构关系
* 指定区域重新拟合

约束可以为空。

因此 Layer 3 需要支持三种模式：

```text
类型和数量已知
类型部分已知
类型和数量均未知
```

## 5.3 输出

### ReconstructionSpec

参数化几何描述，是整个系统的核心中间数据结构。

主要包含：

* 对象 ID
* 几何类型
* 位置
* 朝向
* 几何参数
* 对应点云区域
* 拟合置信度
* 对象间几何关系

例如对象关系可以包括：

* 平行
* 垂直
* 同轴
* 共面
* 接触
* 包含
* 组合
* 减去

## 5.4 数据接口

```text
PointCloud
    +
ReconstructionConstraint
          ↓
       Layer 3
          ↓
ReconstructionSpec
```

---

# 6. Layer 4：AI 辅助修改

## 6.1 功能

接收用户自然语言，将其转换为结构化的重建修改指令。

Layer 4 不直接处理点云，也不直接操作 SolidWorks。

主要流程：

```text
自然语言
   ↓
AI / Agent
   ↓
结构化修改指令
   ↓
Layer 3
```

## 6.2 输入

### UserPrompt

用户自然语言描述。

例如：

```text
物体 A 应该继续拆成两个部分。
```

或者：

```text
物体 A 应该是一个长方体。
```

### ReconstructionSpec

当前模型结构。

### SelectedObject

可选的当前选中对象。

## 6.3 输出

### ReconstructionCommand

结构化重建指令。

核心操作包括：

* 拆分对象
* 合并对象
* 修改几何类型
* 修改尺寸
* 重新拟合
* 删除对象
* 添加对象
* 添加几何关系
* 删除几何关系

Layer 3 根据该指令重新生成新的 `ReconstructionSpec`。

## 6.4 数据接口

```text
UserPrompt
    +
ReconstructionSpec
          ↓
       Layer 4
          ↓
ReconstructionCommand
          ↓
       Layer 3
```

---

# 7. Layer 5：CAD / SolidWorks MCP

## 7.1 功能

将参数化几何描述转换为 CAD 建模操作，并通过 MCP 驱动 SolidWorks。

该层不负责判断物体是什么，只负责根据已经确定的几何结构进行建模。

## 7.2 输入

### ReconstructionSpec

Layer 3 生成的参数化模型描述。

## 7.3 中间数据

### CADPlan

表示 CAD 建模步骤，例如：

```text
创建草图
绘制轮廓
添加尺寸
拉伸
旋转
切除
布尔运算
圆角
倒角
```

## 7.4 输出

### CAD Model

最终 CAD 模型。

主要输出包括：

* SolidWorks 模型
* STEP
* STL
* 执行日志

## 7.5 数据接口

```text
ReconstructionSpec
        ↓
     Layer 5
        ↓
     CADPlan
        ↓
       MCP
        ↓
   SolidWorks
```

---

# 8. 核心数据类型

整个系统主要围绕以下数据类型进行连接：

| 数据类型                     | 作用         |
| ------------------------ | ---------- |
| Model                    | 原始三维模型     |
| CameraConfig             | 虚拟摄像机配置    |
| ImageSet                 | 多视角图片      |
| CameraInfo               | 摄像机内外参     |
| PointCloud               | 三维点云       |
| ReconstructionConstraint | 重建约束       |
| ReconstructionSpec       | 参数化几何描述    |
| UserPrompt               | 用户自然语言     |
| ReconstructionCommand    | AI 生成的重建指令 |
| CADPlan                  | CAD 操作规划   |
| CAD Model                | 最终 CAD 模型  |

---

# 9. 层间数据对齐

系统的主要数据流为：

```text
Layer 1
Model
+
CameraConfig
↓
ImageSet
+
CameraInfo
```

```text
Layer 2
ImageSet
+
CameraInfo
↓
PointCloud
```

```text
Layer 3
PointCloud
+
Optional Constraints
↓
ReconstructionSpec
```

```text
Layer 4
UserPrompt
+
ReconstructionSpec
↓
ReconstructionCommand
↓
Layer 3
```

```text
Layer 5
ReconstructionSpec
↓
CADPlan
↓
CAD Model
```

---

# 10. 系统核心架构

整体架构可以概括为：

```text
        Virtual Model
             ↓
          Layer 1
             ↓
        ImageSet
             ↓
          Layer 2
             ↓
        PointCloud
             ↓
          Layer 3
             ↓
   ReconstructionSpec
        ↙          ↘
   Layer 4        Layer 5
 AI Editing       CAD MCP
      ↓              ↓
Rebuild         SolidWorks
```

现实场景中也可以直接：

```text
真实零件
   ↓
现实拍摄
   ↓
ImageSet + CameraInfo
   ↓
Layer 2
```

因此 Layer 1 属于数据生成和测试模块，不是实际运行流程中的必要部分。

---

# 11. 核心中间表示

整个系统最重要的数据结构是：

**`ReconstructionSpec`**

它是三维视觉与 CAD 系统之间的统一中间表示。

可以将整个项目理解为：

```text
现实图像
   ↓
三维重建
   ↓
几何理解
   ↓
ReconstructionSpec
   ↓
AI 修改 / CAD 转换
   ↓
SolidWorks
```

各层只需要遵守统一的数据接口，即可独立进行算法和实现替换。
