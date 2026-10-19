function [ITAE, sim_out] = quadrotor_system(params, ctrl_type, scenario)
    % 物理参数
    m = 1.2; g = 9.81; l = 0.25; Ix = 0.011; Iy = 0.011; Iz = 0.022;
    dt = 0.01; T = 20; t = 0:dt:T; N = length(t);
    X = zeros(12, N); X(3,1) = 1; 
    
    % 三维螺旋轨迹及前馈导数
    xd = cos(0.5*t); yd = sin(0.5*t); zd = 1 + 0.1*t;
    int_e = [0;0;0]; err_buffer = zeros(3, 10); ITAE = 0;
    max_angle = 35 * pi / 180; % 飞控物理饱和限制 (最大倾角35度)
    
    for k = 1:N-1
        s = X(:, k); 
        ref = [xd(k); yd(k); zd(k)];
        d_ref = [-0.5*sin(0.5*t(k)); 0.5*cos(0.5*t(k)); 0.1];
        dd_ref = [-0.25*cos(0.5*t(k)); -0.25*sin(0.5*t(k)); 0]; % 加速度前馈
        
        e_pos = s(1:3) - ref; 
        de_pos = s(7:9) - d_ref;
        
        % 积分抗饱和处理 (Anti-windup)，防止风扰期间积分器暴涨导致风停后超调
        int_e = max(min(int_e + e_pos * dt, 2), -2); 
        
        if strcmp(ctrl_type, 'FOSMC') || strcmp(ctrl_type, 'SMC')
            lambda = params(1:3); alpha = params(4);
            err_buffer = [e_pos, err_buffer(:, 1:9)];
            D_alpha = (err_buffer(:,1) - alpha*err_buffer(:,2)) / (dt^alpha);
            
            % 带有抗饱和积分和分数阶导数的复合滑模面
            S = lambda(:).*e_pos + 1.5*int_e + de_pos + 0.1*D_alpha; 
            
            % 控制律：前馈 + 连续化鲁棒趋近律
            U_virt = dd_ref - 5*S - 3*(S ./ (abs(S) + 0.05)); 
            ux = U_virt(1); uy = U_virt(2); uz = U_virt(3);
        elseif strcmp(ctrl_type, 'PID')
            Kp = params(1); Ki = params(2); Kd = params(3);
            U_virt = dd_ref - Kp*e_pos - Ki*int_e - Kd*de_pos; 
            ux = U_virt(1); uy = U_virt(2); uz = U_virt(3);
        end
        
        U1 = m * (g + uz); 
        
        % 姿态期望值提取与限幅保护
        phi_d = max(min(-uy/g, max_angle), -max_angle); 
        theta_d = max(min(ux/g, max_angle), -max_angle);
        
        % 姿态内环 P-D 控制 
        U2 = 10*(phi_d - s(4)) - 3*s(10); 
        U3 = 10*(theta_d - s(5)) - 3*s(11); 
        U4 = -3*s(6) - 1*s(12);
        
        % 干扰模型
        dist = [0;0;0];
        if strcmp(scenario, 'Normal')
            dist = 0.5 * sin(t(k)) * [1;1;0];
        end
        % 10s 到 11s 注入强风脉冲
        if strcmp(scenario, 'Disturbed') && t(k) >= 10 && t(k) <= 11
            dist = [3.0; 0; 0]; 
        end
        
        dX = zeros(12,1); dX(1:3) = s(7:9);
        phi = s(4); theta = s(5); psi = s(6);
        
        % ==========================================================
        % 【史诗级修复】：严格匹配论文公式 (1)，将推力映射到世界坐标系
        % ==========================================================
        ddx = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * U1/m + dist(1)/m;
        ddy = (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * U1/m + dist(2)/m;
        ddz = (cos(phi)*cos(theta)) * U1/m - g + dist(3)/m;
        
        dX(7:9) = [ddx; ddy; ddz];
        
        dX(4:6) = s(10:12);
        dX(10) = (s(11)*s(12)*(Iy-Iz) + l*U2)/Ix; 
        dX(11) = (s(10)*s(12)*(Iz-Ix) + l*U3)/Iy; 
        dX(12) = (s(10)*s(11)*(Ix-Iy) + U4)/Iz;
        
        X(:, k+1) = s + dX*dt;
        ITAE = ITAE + t(k)*(abs(e_pos(1)) + abs(e_pos(2)) + abs(e_pos(3)))*dt;
    end
    sim_out.t = t; sim_out.x = X(1,:); sim_out.y = X(2,:); sim_out.z = X(3,:);
    sim_out.xd = xd; sim_out.yd = yd; sim_out.zd = zd;
end