import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pandas as pd

# ==========================================
# 1. 读取数据
# ==========================================
try:
    df = pd.read_csv('robot_data.csv', header=None)
    data = df.values
    t = data[:, 0]
    q = data[:, 4:7] 
    print(f"成功读取数据，共 {len(t)} 帧")
except FileNotFoundError:
    print("错误：找不到 robot_data.csv，请先在 MATLAB 中导出！")
    exit()

# ==========================================
# 2. 机器人正运动学
# ==========================================
d1 = 0.1; a2 = 0.4; a3 = 0.3

def forward_kinematics(q_now):
    q1, q2, q3 = q_now
    c1, s1 = np.cos(q1), np.sin(q1)
    T01 = np.array([[c1, 0, s1, 0], [s1, 0, -c1, 0], [0, 1, 0, d1], [0, 0, 0, 1]])
    
    c2, s2 = np.cos(q2), np.sin(q2)
    T12 = np.array([[c2, -s2, 0, a2*c2], [s2, c2, 0, a2*s2], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    c3, s3 = np.cos(q3), np.sin(q3)
    T23 = np.array([[c3, -s3, 0, a3*c3], [s3, c3, 0, a3*s3], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    
    return [T01[:3, 3], T02[:3, 3], T03[:3, 3]] # 返回 p1, p2, p3

# ==========================================
# 3.预计算所有末端位置
# ==========================================
# 这一步是为了让拖尾（Trail）即使在跳帧播放时也能保持平滑
print("正在预计算全轨迹坐标...")
all_p3 = [] # 存储所有时刻的末端坐标
for val in q:
    pts = forward_kinematics(val)
    all_p3.append(pts[-1]) # 只存末端 p3
all_p3 = np.array(all_p3) # 转为 numpy 数组 (N x 3)
print("预计算完成。")

# ==========================================
# 4. 场景设置
# ==========================================
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')

# 地面
def plot_checkerboard(ax, z=0, width=0.8, num=10):
    x = np.linspace(-width, width, num+1)
    y = np.linspace(-width, width, num+1)
    X, Y = np.meshgrid(x, y)
    ax.plot_wireframe(X, Y, np.full_like(X, z), color='gray', alpha=0.2, linewidth=0.5)
plot_checkerboard(ax)

# 机械臂
link1, = ax.plot([], [], [], '-', lw=8, color='#34495e', solid_capstyle='round')
link2, = ax.plot([], [], [], '-', lw=6, color='#3498db', solid_capstyle='round')
link3, = ax.plot([], [], [], '-', lw=4, color='#e74c3c', solid_capstyle='round')
joints, = ax.plot([], [], [], 'o', markersize=9, color='k', mec='w', mew=1.5)

# --- 轨迹线优化 ---
# 1. 全局路径 (浅灰色细线，展示完整路线)
ax.plot(all_p3[:,0], all_p3[:,1], all_p3[:,2], '-', color='gray', alpha=0.3, lw=0.8, label='Full Path')

# 2. 动态拖尾 (高亮橙色，只显示最近一段)
trail_line, = ax.plot([], [], [], '-', lw=2, color='orange', alpha=1.0, label='Active Trail')

# 3. 地面投影 (Shadow)
shadow_line, = ax.plot([], [], [], '--', lw=1.5, color='gray', alpha=0.6)

# 4. 垂线
drop_line, = ax.plot([], [], [], ':', lw=1, color='gray', alpha=0.8)

# 5. 操作平面
plane_verts = [[(0,0,0)]*4]
arm_plane = Poly3DCollection(plane_verts, alpha=0.15, facecolor='#66ccff')
ax.add_collection3d(arm_plane)

# 6. 文本
text_info = ax.text2D(0.02, 0.90, "", transform=ax.transAxes, fontsize=11, family='monospace')

# 起止点
p_s, p_e = all_p3[0], all_p3[-1]
ax.scatter([p_s[0]], [p_s[1]], [p_s[2]], c='g', s=150, alpha=0.6, edgecolors='k', label='Start')
ax.scatter([p_e[0]], [p_e[1]], [p_e[2]], c='r', s=150, alpha=0.6, edgecolors='k', label='End')
ax.legend(loc='upper right')

# 视图
ax.set_xlim(-0.7, 0.7); ax.set_ylim(-0.7, 0.7); ax.set_zlim(0, 0.9)
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('Robot Trajectory (Smooth Trail Fix)', fontsize=14, pad=15)
ax.view_init(elev=25, azim=45)
ax.xaxis.pane.fill = False; ax.yaxis.pane.fill = False; ax.zaxis.pane.fill = False; ax.grid(False)

# ==========================================
# 5. 动画更新
# ==========================================
# 拖尾长度 (点数)
# 注意：这里指的是原始数据的点数，而不是帧数。这样保证线条顺滑。
trail_length_data_points = 2000 

step = max(1, len(t) // 120) 
indices = range(0, len(t), step)

def update(frame_idx):
    # 1. 计算连杆 (保持原样)
    q_now = q[frame_idx]
    pts = forward_kinematics(q_now)
    p1, p2, p3 = pts # p0 是原点
    p0 = [0,0,0]
    
    link1.set_data([p0[0], p1[0]], [p0[1], p1[1]]); link1.set_3d_properties([p0[2], p1[2]])
    link2.set_data([p1[0], p2[0]], [p1[1], p2[1]]); link2.set_3d_properties([p1[2], p2[2]])
    link3.set_data([p2[0], p3[0]], [p2[1], p3[1]]); link3.set_3d_properties([p2[2], p3[2]])
    
    joints.set_data([0, p1[0], p2[0], p3[0]], [0, p1[1], p2[1], p3[1]])
    joints.set_3d_properties([0, p1[2], p2[2], p3[2]])

    # 2. 【修复】使用切片更新拖尾
    # 直接从预计算好的 all_p3 中截取一段。包含中间被跳过的帧！
    start_idx = max(0, frame_idx - trail_length_data_points)
    end_idx = frame_idx + 1
    
    trail_segment = all_p3[start_idx : end_idx] # 取出这一段所有的高精度点
    
    # 绘制空间拖尾
    trail_line.set_data(trail_segment[:, 0], trail_segment[:, 1])
    trail_line.set_3d_properties(trail_segment[:, 2])
    
    # 绘制地面投影 (也使用高精度数据)
    shadow_line.set_data(trail_segment[:, 0], trail_segment[:, 1])
    shadow_line.set_3d_properties(np.zeros(len(trail_segment)))
    
    # 垂线
    drop_line.set_data([p3[0], p3[0]], [p3[1], p3[1]]); drop_line.set_3d_properties([p3[2], 0])

    # 3. 操作平面
    R_max = 0.8; theta = q_now[0]
    v = [[0,0,0], [0,0,0.8], 
         [R_max*np.cos(theta), R_max*np.sin(theta), 0.8], 
         [R_max*np.cos(theta), R_max*np.sin(theta), 0]]
    arm_plane.set_verts([v])
    
    text_info.set_text(f"Time: {t[frame_idx]:.2f} s\nZ: {p3[2]:.3f} m")
    return link1, link2, link3, joints, trail_line, shadow_line, drop_line, arm_plane, text_info

# ==========================================
# 6. 保存
# ==========================================
print("正在生成动画...")
ani = animation.FuncAnimation(fig, update, frames=indices, interval=40, blit=False)
ani.save('robot_smooth_trail.gif', writer='pillow', fps=25)
print("完成！")
plt.show()