function q = inverse_kinematics(x, y, z)
    % 机械臂几何参数 (来自你的 derive_robot_dynamics.m)
    d1 = 0.1; 
    a2 = 0.4; 
    a3 = 0.3;

    % 1. 求解关节 1 (基座旋转)
    q1 = atan2(y, x);

    % 2. 计算平面参数
    r = sqrt(x^2 + y^2);    % 水平投影距离
    z_eff = z - d1;         % 相对于肩关节的高度
    D_sq = r^2 + z_eff^2;   % 肩关节到末端的距离平方
    
    % 3. 求解关节 3 (肘部) - 使用余弦定理
    % D^2 = a2^2 + a3^2 - 2*a2*a3*cos(pi - q3)
    % cos_q3 = (r^2 + z_eff^2 - a2^2 - a3^2) / (2 * a2 * a3)
    cos_q3 = (D_sq - a2^2 - a3^2) / (2 * a2 * a3);
    
    % 安全检查：目标点是否超出工作空间
    if abs(cos_q3) > 1
        error('目标点超出机械臂工作空间！');
    end
    
    % 这里通常有两个解（肘部向上/向下），我们选肘部向上的解
    q3 = -acos(cos_q3); % 注意：根据具体DH定义方向可能需要调整符号

    % 4. 求解关节 2 (肩部)
    % alpha 是目标点相对于水平面的角度
    % beta 是三角形内角
    alpha = atan2(z_eff, r);
    beta  = atan2(a3 * sin(-q3), a2 + a3 * cos(-q3));
    q2 = alpha - beta;

    q = [q1; q2; q3];
end