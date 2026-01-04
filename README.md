# 🤖 3-DOF 机械臂动力学仿真与可视化系统

这是一个基于 **MATLAB/Simulink** 和 **Python** 的三自由度（3-DOF）机械臂全流程仿真项目。本项目实现了从动力学建模、轨迹规划、逆运动学解算、Simulink 闭环仿真到 Python 高质量 3D 可视化的完整工作流。

## 📋 项目功能

* **动力学建模**
    基于 **Lagrange** 方法自动推导机械臂的动力学方程，包括惯性矩阵 $M(q)$、科里奥利力与离心力矩阵 $C(q, \dot{q})$ 及重力项 $G(q)$。
* **轨迹规划**
    支持关节空间插值（Point-to-Point）和笛卡尔空间直线/多点规划。
* **逆运动学 (IK)**
    基于几何法的逆解算法，自动处理多解情况（如 Elbow Up/Down 配置）。
* **闭环仿真**
    在 Simulink 中构建 S-Function 动力学模型，实现力矩控制与系统响应仿真。
* **3D 可视化**
    使用 Python (`matplotlib`) 绘制包含轨迹拖尾、地面投影和操作平面的高质量动画。

---

## 📂 文件结构预览

```text
Project_Root/
├── MATLAB_Scripts/
│   ├── derive_robot_dynamics.m       # 动力学推导脚本
│   ├── robot_plant.m                 # 物理对象模型
│   ├── Inverse_solution.slx          # Simulink 仿真模型
│   ├── Single-point_IK.mlx           # 单点轨迹生成
│   ├── Post-processing_script.mlx    # 数据导出脚本
│   └── generated/                    # 自动生成的函数 (get_M, get_C, get_G)
├── Python_Visualization/
│   ├── Drawing_script.py             # 3D 绘图与动画脚本
│   └── robot_data.csv                # 仿真导出的数据文件
└── README.md
```

---

## 🛠️ 环境依赖

### MATLAB / Simulink
* **版本**：MATLAB R2023b 或更高版本
* **工具箱**：
    * Simulink
    * Symbolic Math Toolbox (用于动力学方程推导)

### Python
* **版本**：Python 3.8+
* **必需库**：
    ```bash
    pip install numpy matplotlib pandas
    ```

---

## 🚀 快速开始 (Quick Start)

请严格按照以下步骤顺序操作，以确保数据流的正确性：

### Step 1: 配置与生成参数
运行脚本 `derive_robot_dynamics.m`。
* **作用**：定义机械臂的物理参数（质量 $m$、长度 $l$、转动惯量 $I$），并利用符号计算推导动力学方程。
* **输出**：将在当前目录下生成或更新 `get_M_matrix.m`, `get_C_matrix.m`, `get_G_vector.m` 文件。
* *> 注意：如果物理参数未修改，此步骤只需运行一次。*

### Step 2: 生成轨迹
运行轨迹生成脚本（如 `Single-point_inverse_kinematics_solution.mlx` 或 `Multi-point_inverse_kinematics_solution.mlx`）。
* **作用**：
    1.  定义起点和终点（或多个路点）。
    2.  进行逆运动学解算。
    3.  生成位置 $q$、速度 $\dot{q}$、加速度 $\ddot{q}$ 的时间序列数据。

### Step 3: Simulink 仿真
打开并运行模型 `Inverse_solution.slx`。
* **配置**：确保模型中的 `From Workspace` 模块读取的是上一步生成的 `ts_q` 等变量。
* **运行**：点击 **Run**。Simulink 将调用 `robot_plant.m` 进行物理引擎解算。
* **结果**：仿真结束后，工作区会生成仿真结果变量 `sim_q_actual` (通常包含在 `out` 对象中)。

### Step 4: 导出数据
运行数据后处理脚本 `Post-processing_script.mlx`。
* **作用**：将 Simulink 的仿真结果清洗并导出为标准格式（如 CSV），供 Python 读取。

### Step 5: Python 画图与动画
运行 Python 可视化脚本 `Drawing_script.py`。
* **输入**：自动读取目录下的 `robot_data.csv`。
* **输出**：弹出一个交互式 3D 窗口，显示机械臂运动动画，并自动保存为 GIF 文件（`robot_smooth_trail.gif`）。

---

## 📝 作者与致谢

* **Project created for:** Robotic Course Design
* **Methodology:** Standard DH parameters & Lagrangian dynamics
* **Update:** zzzhkgs 2026.1.4 (Doc Optimization)