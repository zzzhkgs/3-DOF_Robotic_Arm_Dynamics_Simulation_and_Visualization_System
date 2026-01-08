# 🤖 3-DOF 机械臂动力学仿真与控制平台 (All-in-One Version)

这是一个集成了 **数学建模**、**动力学仿真** 与 **3D 可视化** 的三自由度（3-DOF）机械臂全流程开发项目。

本项目采用了 **"All-in-One"** 的设计理念，通过单个 MATLAB 脚本 (`main.mlx`) 即可自动完成动力学方程推导、轨迹规划、Simulink 闭环控制仿真及数据导出，并配合 Python 实现高质量的 3D 动画与误差分析。

---

## ✨ 核心功能

| 模块 | 功能描述 |
| :--- | :--- |
| **🚀 一键自动化** | `main.mlx` 脚本自动串联 Lagrange 推导 -> 轨迹生成 -> 仿真执行 -> 数据导出全流程。 |
| **📐 动力学建模** | 自动符号推导并生成惯性矩阵 $M(q)$、科里奥利力 $C(q, \dot{q})$ 及重力项 $G(q)$ 的函数文件。 |
| **🛣️ 轨迹规划** | 基于五次多项式的笛卡尔空间多点连续轨迹规划 (Stop-and-Go 模式)。 |
| **🧠 智能控制** | Simulink 模型集成了 **PID 控制** 与 **自适应 RBF 神经网络滑模控制** 的对比实验。 |
| **📊 高级可视化** | Python 脚本读取仿真数据，生成交互式 3D 机械臂动画 (GIF) 及控制误差对比图表。 |

---

## 📂 目录结构 (File Tree)

基于项目实际文件结构：

```text
3-DOF-Robot-Simulation/
├── MATLAB Core
│   ├── main.mlx                      # [核心入口] 主程序：推导、规划、仿真、导出一条龙
│   ├── Inverse_solution.slx          # [核心模型] Simulink 动力学与控制模型
│   ├── robot_plant.m                 # S-Function 机械臂物理对象描述
│   ├── get_M_matrix.m                # (自动生成) 动力学惯性矩阵
│   ├── get_C_matrix.m                # (自动生成) 科里奥利矩阵
│   └── get_G_vector.m                # (自动生成) 重力向量
│
├── Python Visualization
│   ├── Drawing_script.py             # 3D 轨迹动画生成脚本 (输出 .gif)
│   ├── Error_calculation_comparison.py # PID vs RBF 误差分析脚本 (输出 .png)
│   └── robot_simulation_data.csv     # MATLAB 导出的仿真结果数据
│
├── README.md                         # 项目说明文档
└── LICENSE                           # 许可证
```

🛠️ 环境依赖
MATLAB

    版本：MATLAB R2023b 或更高

    工具箱：Simulink, Symbolic Math Toolbox

Python

    必需库：
    Bash

    pip install numpy matplotlib pandas

🚀 快速开始 (Workflow)

现在的操作流程非常简单，分为 MATLAB 计算 和 Python 画图 两大步：
Step 1: 运行 MATLAB 主程序

直接打开并运行 main.mlx。脚本将自动按顺序执行以下任务：

    环境清理：自动执行 bdclose, clear, clc。

    符号推导：计算 Lagrange 动力学方程，并在当前目录更新 get_*.m 函数文件。

    轨迹生成：规划笛卡尔空间的多点路径，生成 ts_q (期望角度) 等时间序列数据。

    启动仿真：自动调用 Inverse_solution.slx 运行 Simulink，带有详细的进度条与错误捕捉。

    数据导出：仿真结束后，自动将 PID 与 RBF 的对比数据写入 robot_simulation_data.csv。

✅ 成功标志：MATLAB 命令行提示 >> [完成] 数据已导出至: robot_simulation_data.csv。
Step 2: 运行 Python 可视化

在终端或 IDE 中运行 Python 脚本进行结果展示。

生成 3D 动画：
Bash

python Drawing_script.py

    输出：弹窗显示 3D 动态轨迹，并在目录下生成 robot_smooth_trail.gif。

生成误差对比图：
Bash

python Error_calculation_comparison.py

    输出：生成 RBF 与 PID 控制器的跟踪误差对比分析图。

📊 仿真结果包含的数据

robot_simulation_data.csv 文件包含以下列，供自定义分析使用：

    Time: 仿真时间戳

    Hope: 期望的关节角度轨迹 (Reference)

    RBF: RBF 神经网络控制下的实际角度

    PID: 传统 PID 控制下的实际角度

⚠️ 常见问题

    仿真报错 "File not found"：

        确保 main.mlx 和 Inverse_solution.slx 在同一目录下，且 MATLAB 的“当前文件夹”已指向该目录。

    Python 读取 CSV 乱码：

        main.mlx 默认使用标准 CSV 格式导出。如果 Python 报错，请检查 pandas 读取时是否需要指定编码（通常默认即可）。

    修改机械臂参数：

        直接在 main.mlx 的 "第一部分：动力学符号推导" 中修改质量 m 或长度 a，重新运行即可自动更新所有模型参数。

📝 作者

    Project: 3-DOF Robot Arm Simulation Platform

    Update: 2026.1.8 (Refactored to All-in-One workflow)
