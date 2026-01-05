% =========================================================================
% 3自由度机械臂动力学推导脚本 (基于 Lagrange 方法)
% 对应文档：基于自适应RBF神经网络滑模控制的三至四轴机械臂伺服系统
% =========================================================================
clear; clc;

%% 1. 定义符号变量
% q: 关节角, dq: 关节角速度, ddq: 关节角加速度
syms q1 q2 q3 real
syms dq1 dq2 dq3 real
syms ddq1 ddq2 ddq3 real
syms g real % 重力加速度

% 定义物理参数 (符号形式，方便后续替换数值)
syms m1 m2 m3 real  % 连杆质量 [cite: 109]
syms a2 a3 real     % 连杆长度 [cite: 110]
syms d1 real        % 基座高度 
% 假设连杆质心位于连杆几何中心 (简化假设，可根据实际修改)
% r1, r2, r3 分别为各连杆质心距离上一关节的距离
r1 = 0;      % 假设基座质心在轴线上
r2 = a2 / 2; 
r3 = a3 / 2;

% 惯性张量 (简化为细杆/点质量模型，仅保留主对角线分量用于示例)
% 实际工程中应导入CAD计算出的惯性张量
syms Izz1 Izz2 Izz3 real 

%% 2. D-H 变换矩阵 
% D-H 参数顺序: theta, d, a, alpha
% 变换矩阵函数定义 (标准的后置乘法)
DH = @(th, d, a, alp) [ ...
    cos(th), -sin(th)*cos(alp),  sin(th)*sin(alp), a*cos(th);
    sin(th),  cos(th)*cos(alp), -cos(th)*sin(alp), a*sin(th);
    0,        sin(alp),          cos(alp),         d;
    0,        0,                 0,                1];

% 建立各连杆坐标系变换矩阵
T01 = DH(q1, d1, 0, pi/2);  % 关节 1 
T12 = DH(q2, 0, a2, 0);     % 关节 2 
T23 = DH(q3, 0, a3, 0);     % 关节 3 

% 计算各连杆相对于基座(世界坐标系)的变换矩阵
T02 = T01 * T12;
T03 = T02 * T23;

%% 3. 计算质心位置与雅可比矩阵
% 提取旋转矩阵 R 和 位置向量 P
% 质心位置 (假设质心在局部坐标系的X轴上距离原点 r 处)
P_c1 = T01 * [0; 0; 0; 1]; % 连杆1质心 (假设在原点)
P_c2 = T02 * [-a2/2; 0; 0; 1]; % 连杆2质心 
% 定义连杆质心在各自局部坐标系下的位置向量
pc1_local = [0; 0; 0; 1];       
pc2_local = [-a2/2; 0; 0; 1];   % 连杆2中心
pc3_local = [-a3/2; 0; 0; 1];   % 连杆3中心

% 转换到基座坐标系
P_cm1 = T01 * pc1_local;
P_cm2 = T02 * pc2_local;
P_cm3 = T03 * pc3_local;

% 提取位置坐标 (前3行)
p1 = P_cm1(1:3);
p2 = P_cm2(1:3);
p3 = P_cm3(1:3);

% 计算线性速度雅可比矩阵 Jv (Jv * dq = v)
q_vec = [q1; q2; q3];
Jv1 = jacobian(p1, q_vec);
Jv2 = jacobian(p2, q_vec);
Jv3 = jacobian(p3, q_vec);

% 计算角速度雅可比矩阵 Jw (Jw * dq = omega)
% z轴在基座坐标系下的方向
z0 = [0; 0; 1];
z1 = T01(1:3, 3);
z2 = T02(1:3, 3);

Jw1 = [z0, [0;0;0], [0;0;0]];
Jw2 = [z0, z1, [0;0;0]];
Jw3 = [z0, z1, z2];

%% 4. 能量计算 (Lagrange 函数)
% 动能 K = 0.5 * m * v' * v + 0.5 * w' * I * w
dq_vec = [dq1; dq2; dq3];

% 连杆1动能
K1 = 0.5 * m1 * (Jv1*dq_vec).' * (Jv1*dq_vec) + 0.5 * (Jw1*dq_vec).' * diag([0,0,Izz1]) * (Jw1*dq_vec);
% 连杆2动能
K2 = 0.5 * m2 * (Jv2*dq_vec).' * (Jv2*dq_vec) + 0.5 * (Jw2*dq_vec).' * diag([0,0,Izz2]) * (Jw2*dq_vec);
% 连杆3动能
K3 = 0.5 * m3 * (Jv3*dq_vec).' * (Jv3*dq_vec) + 0.5 * (Jw3*dq_vec).' * diag([0,0,Izz3]) * (Jw3*dq_vec);

K_total = simplify(K1 + K2 + K3);

% 势能 P = m * g * h
P1 = m1 * g * p1(3);
P2 = m2 * g * p2(3);
P3 = m3 * g * p3(3);
P_total = simplify(P1 + P2 + P3);

% Lagrange 函数
L = K_total - P_total;

%% 5. 推导动力学方程 M, C, G
fprintf('正在计算惯性矩阵 M(q)...\n');
% M 矩阵可以通过动能表达式提取：K = 0.5 * dq' * M * dq
M = jacobian(jacobian(K_total, dq_vec).', dq_vec);
M = simplify(M);

fprintf('正在计算重力向量 G(q)...\n');
% G 向量是势能对 q 的梯度
G = jacobian(P_total, q_vec).';
G = simplify(G);

fprintf('正在计算科里奥利矩阵 C(q, dq)...\n');
% 使用 Christoffel 符号计算 C 矩阵
n = 3;
C = sym(zeros(n, n));
for k = 1:n
    for j = 1:n
        temp = 0;
        for i = 1:n
            % Christoffel 符号公式
            c_ijk = 0.5 * (diff(M(k,j), q_vec(i)) + diff(M(k,i), q_vec(j)) - diff(M(i,j), q_vec(k)));
            temp = temp + c_ijk * dq_vec(i);
        end
        C(k,j) = temp;
    end
end
C = simplify(C);

%% 6. 代入数值并生成函数文件
fprintf('正在生成 MATLAB 函数文件...\n');

% 定义数值参数
param_values = struct();
param_values.m1 = 2.0; param_values.m2 = 1.5; param_values.m3 = 1.0;
param_values.a2 = 0.4; param_values.a3 = 0.3;
param_values.d1 = 0.1; % 假设值，不影响动力学主项
param_values.g = 9.81;
% 估算转动惯量 (假设为细杆 I = 1/12 * m * L^2)
param_values.Izz1 = 0.1; % 假设值
param_values.Izz2 = 1/12 * 1.5 * 0.4^2;
param_values.Izz3 = 1/12 * 1.0 * 0.3^2;

% 替换符号变量为数值
M_num = subs(M, {m1,m2,m3,a2,a3,d1,g,Izz1,Izz2,Izz3}, ...
             {param_values.m1, param_values.m2, param_values.m3, ...
              param_values.a2, param_values.a3, param_values.d1, ...
              param_values.g, param_values.Izz1, param_values.Izz2, param_values.Izz3});
G_num = subs(G, {m1,m2,m3,a2,a3,d1,g,Izz1,Izz2,Izz3}, ...
             {param_values.m1, param_values.m2, param_values.m3, ...
              param_values.a2, param_values.a3, param_values.d1, ...
              param_values.g, param_values.Izz1, param_values.Izz2, param_values.Izz3});
C_num = subs(C, {m1,m2,m3,a2,a3,d1,g,Izz1,Izz2,Izz3}, ...
             {param_values.m1, param_values.m2, param_values.m3, ...
              param_values.a2, param_values.a3, param_values.d1, ...
              param_values.g, param_values.Izz1, param_values.Izz2, param_values.Izz3});

% 生成 .m 文件供 Simulink 调用
matlabFunction(M_num, 'File', 'get_M_matrix', 'Vars', {q1, q2, q3});
matlabFunction(C_num, 'File', 'get_C_matrix', 'Vars', {q1, q2, q3, dq1, dq2, dq3});
matlabFunction(G_num, 'File', 'get_G_vector', 'Vars', {q1, q2, q3});

fprintf('完成！已生成 get_M_matrix.m, get_C_matrix.m, get_G_vector.m\n');