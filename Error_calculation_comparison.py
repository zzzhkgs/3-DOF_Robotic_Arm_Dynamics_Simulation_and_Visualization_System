
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ==========================================
# 1. 读取数据
# ==========================================
filename = 'robot_data.csv'
try:
    # 格式: [t, q_hope(3列), q_rbf(3列), q_pid(3列)]
    data = pd.read_csv(filename, header=None)
    print(f"成功读取文件: {filename}, 数据维度: {data.shape}")
except FileNotFoundError:
    print(f"错误: 找不到文件 {filename}。")
    exit()

# 提取列
t = data.iloc[:, 0].values # 时间
q_hope = data.iloc[:, 1:4].values # 期望 [N x 3]
q_rbf  = data.iloc[:, 4:7].values # RBF  [N x 3]
q_pid  = data.iloc[:, 7:10].values # PID  [N x 3]

# ==========================================
# 2. 计算误差 (Error Calculation)
# ==========================================
# 误差 = 期望 - 实际
err_rbf = q_hope - q_rbf
err_pid = q_hope - q_pid

# 计算绝对误差的平均值 (MAE) 用于柱状图
mae_rbf = np.mean(np.abs(err_rbf), axis=0)
mae_pid = np.mean(np.abs(err_pid), axis=0)

# ==========================================
# 3. 绘图 1: 轨迹跟踪 (Trajectory)
# ==========================================
plt.style.use('seaborn-v0_8-paper') 
joints = ['Joint 1', 'Joint 2', 'Joint 3']

fig1, axes1 = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
fig1.suptitle('Robot Trajectory Tracking Comparison', fontsize=16, fontweight='bold')

for i in range(3):
    ax = axes1[i]
    ax.plot(t, q_hope[:, i], 'k--', linewidth=2, label='Reference')
    ax.plot(t, q_pid[:, i], 'b-', linewidth=1.5, alpha=0.6, label='PID')
    ax.plot(t, q_rbf[:, i], 'r-', linewidth=2, label='RBF-SMC')
    ax.set_ylabel(f'{joints[i]} (rad)')
    ax.legend(loc='upper right')
    ax.grid(True, linestyle=':', alpha=0.6)

axes1[-1].set_xlabel('Time (s)')
plt.tight_layout()
plt.savefig('Figure1_Trajectory.png', dpi=300)

# ==========================================
# 4. 绘图 2: 误差对比 (Error Analysis) - 【核心新增】
# ==========================================
fig2 = plt.figure(figsize=(12, 10))
fig2.suptitle('Tracking Error & Magnitude Comparison', fontsize=16, fontweight='bold')

# 创建布局：左边竖着放3个误差随时间变化的图，右边放一个总的柱状图对比
gs = fig2.add_gridspec(3, 2, width_ratios=[3, 1])

# --- 左侧：误差随时间变化曲线 (Error vs Time) ---
for i in range(3):
    ax = fig2.add_subplot(gs[i, 0])
    
    # 画 PID 误差（通常较大，用蓝色填充或细线）
    ax.plot(t, err_pid[:, i], 'b-', linewidth=1, alpha=0.5, label='PID Error')
    # 填充 PID 误差区域，让对比更强烈
    ax.fill_between(t, err_pid[:, i], 0, color='blue', alpha=0.1)
    
    # 画 RBF 误差（通常较小，用红色鲜明线）
    ax.plot(t, err_rbf[:, i], 'r-', linewidth=1.5, label='RBF Error')
    

    # 设置图例（只在第一个图显示）
    if i == 0:
        ax.legend(loc='upper right')
    
    # 设置Y轴范围对称，方便看震荡
    # 自动获取最大误差幅度并稍微放大
    max_e = max(np.max(np.abs(err_pid[:, i])), np.max(np.abs(err_rbf[:, i])))
    ax.set_ylim(-max_e*1.1, max_e*1.1)

ax.set_xlabel('Time (s)')

# --- 右侧：误差大小统计 (MAE Bar Chart) ---
ax_bar = fig2.add_subplot(gs[:, 1]) # 占满右侧
x_pos = np.arange(3)
width = 0.35

rects1 = ax_bar.bar(x_pos - width/2, mae_pid, width, label='PID', color='blue', alpha=0.6)
rects2 = ax_bar.bar(x_pos + width/2, mae_rbf, width, label='RBF', color='red', alpha=0.8)

ax_bar.set_ylabel('Mean Absolute Error (rad)')
ax_bar.set_title('Overall Accuracy Comparison')
ax_bar.set_xticks(x_pos)
ax_bar.set_xticklabels(joints)
ax_bar.legend()

# 在柱子上标数值
def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        ax_bar.annotate(f'{height:.4f}',
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=9)

autolabel(rects1)
autolabel(rects2)

plt.tight_layout()
plt.savefig('Figure2_ErrorAnalysis.png', dpi=300)

print("绘图完成！")
print("生成了 'Figure1_Trajectory.png' (轨迹对比)")
print("生成了 'Figure2_ErrorAnalysis.png' (误差曲线 + 误差大小柱状图)")
plt.show()