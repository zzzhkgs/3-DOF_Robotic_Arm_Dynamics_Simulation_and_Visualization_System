function [sys,x0,str,ts,simStateCompliance] = robot_plant(t,x,u,flag)
% robot_plant.m (50-200Nm)

switch flag
  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
  case 1
    sys=mdlDerivatives(t,x,u);
  case 3
    sys=mdlOutputs(t,x,u);
  case {2,4,9}
    sys=[];
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
% =========================================================================
% 子函数 1: 初始化
% =========================================================================
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 6;  
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;  
sizes.NumInputs      = 3;  
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% --- 【关键修改开始】自动同步初始状态 ---
try
    % 1. 尝试从 MATLAB 主工作区读取 x0_robot 变量
    % (这个变量应该由 generate_trajectory.m 生成)
    initial_state = evalin('base', 'x0_robot');
    
    % 2. 安全检查：确保它是一个 6维向量 (3个角度 + 3个速度)
    if length(initial_state) == 6
        x0 = initial_state;
        % disp('robot_plant: 已成功同步初始状态！'); 
    else
        warning('工作区变量 x0_robot 维度错误，使用默认值 [0.1; 0.1; 0.1; 0; 0; 0]。');
        x0 = [0.1; 0.1; 0.1; 0; 0; 0]; 
    end
catch
    % 3. 如果没找到变量 (比如你忘了运行脚本)，为了不报错，使用默认值
    warning('未找到工作区变量 x0_robot，使用默认初始状态。建议先运行 generate_trajectory.m！');
    x0 = [0.1; 0.1; 0.1; 0; 0; 0]; 
end
% --- 【关键修改结束】 -------------------

str = [];
ts  = [0 0];
simStateCompliance = 'UnknownSimState';

% =========================================================================
% 子函数 2: 计算导数 
% =========================================================================
function sys=mdlDerivatives(t,x,u)

% 1. 提取状态
q  = x(1:3);   
dq = x(4:6);   

% 2. 提取输入力矩 
tau = u(1:3);
tau = max(min(tau, 1000), -1000); 

% =========================================================================
% 
% =========================================================================
tau_load = [0; 0; 0];

if 0
    % 预热期: 空载
    tau_load = [0; 0; 0];

elseif 0
    % 阶段一: 突加 50Nm 负载 
    % 轴2受力 50Nm, 轴3受力 30Nm
    tau_load = [0; 50.0; 30.0]; 

elseif 0
    % 阶段二: 负载线性增加 50Nm -> 200Nm
    % 斜率计算: (200 - 50) / 5s = 30 Nm/s
    slope = (200.0 - 50.0) / (15.0 - 10.0); 
    load_val = 50.0 + (t - 10.0) * slope;
    % 轴3按 0.6 的比例相应增加
    tau_load = [0; load_val; load_val * 0.6]; 

elseif 0 
    % 阶段三: 150Nm 负载 + 50Nm 震荡
    % 峰值负载 = 150 + 50 = 200Nm
    % 频率 5Hz (高频震荡)
    vibration = 50.0 * sin(2 * pi * 5 * (t - 15.0)); 
    base_load = 150.0;
    
    current_load_2 = base_load + vibration;
    current_load_3 = current_load_2 * 0.6;
    
    tau_load = [0; current_load_2; current_load_3];

else
    % 阶段四: 恒定100Nm 负载
    tau_load = [0; 100.0; 60.0];
end
% ----------------------------------------------------

% 3. 动力学参数
M = get_M_matrix(q(1), q(2), q(3));
C = get_C_matrix(q(1), q(2), q(3), dq(1), dq(2), dq(3));
G = get_G_vector(q(1), q(2), q(3));

% 4. 摩擦力 (使用 tanh 保持稳定)
Fv = diag([0.5, 0.5, 0.5]); 
Fc = diag([0.1, 0.1, 0.1]); 
F_friction = Fv * dq + Fc * tanh(2 * dq);

% 5. 求解加速度
% 动力学方程: M*ddq = tau - C*dq - G - Friction - Load
total_torque = tau - C*dq - G - F_friction - tau_load;

ddq = pinv(M) * total_torque;

if any(isnan(ddq)) || any(isinf(ddq))
    ddq = zeros(3,1);
end

sys = [dq; ddq];

% =========================================================================
% 子函数 3: 输出
% =========================================================================
function sys=mdlOutputs(t,x,u)
sys = x;