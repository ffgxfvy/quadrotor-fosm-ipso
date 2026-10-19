% =========================================================================
% 四旋翼轨迹跟踪控制对比仿真 (终极图表数据全息导出)
% =========================================================================
clc; clear; close all;

%% --- 1. IPSO 参数寻优 ---
disp('正在进行 IPSO 参数寻优...');
swarm_size = 30; max_iter = 50; dim = 4;
lb = [1, 1, 1, 0.1]; ub = [15, 15, 15, 0.9];
pos = repmat(lb, swarm_size, 1) + rand(swarm_size, dim) .* (ub - lb);
vel = zeros(swarm_size, dim);
pBest = pos; pBest_fit = inf(swarm_size, 1);
gBest = zeros(1, dim); gBest_fit = inf;
fit_hist = zeros(max_iter, 1);

for t_iter = 1:max_iter
    w = 0.9 - 0.5 * (t_iter / max_iter)^2; 
    for i = 1:swarm_size
        cost = quadrotor_system(pos(i,:), 'FOSMC', 'Normal'); 
        if cost < pBest_fit(i), pBest_fit(i) = cost; pBest(i,:) = pos(i,:); end
        if cost < gBest_fit, gBest_fit = cost; gBest = pos(i,:); end
    end
    r1 = rand(swarm_size, dim); r2 = rand(swarm_size, dim);
    vel = w*vel + 1.5*r1.*(pBest - pos) + 1.5*r2.*(repmat(gBest, swarm_size, 1) - pos);
    pos = pos + vel; pos = max(pos, lb); pos = min(pos, ub);
    fit_hist(t_iter) = gBest_fit;
end

% 导出图3：收敛曲线
fig3 = figure(3); set(gcf, 'Color', 'w');
plot(1:max_iter, fit_hist, 'k-o', 'LineWidth', 1.5, 'MarkerSize', 5, 'MarkerFaceColor', 'w');
grid on; xlabel('Iteration (k)', 'FontName', 'Times New Roman'); ylabel('Fitness Value (ITAE)', 'FontName', 'Times New Roman');
set(gca, 'YScale', 'log'); set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
print(fig3, 'Figure_3_Convergence.png', '-dpng', '-r300');

%% --- 2. 开展对比实验与鲁棒性测试 ---
disp('正在运行对比实验与极强脉冲风鲁棒测试...');

[itae_fosmc, data_fosmc] = quadrotor_system(gBest, 'FOSMC', 'Normal');
pid_params = [2.0, 0.5, 1.5, 0]; 
[itae_pid, data_pid] = quadrotor_system(pid_params, 'PID', 'Normal');
smc_params = [8.0, 8.0, 8.0, 1.0]; 
[itae_smc, data_smc] = quadrotor_system(smc_params, 'SMC', 'Normal');
[~, data_robust] = quadrotor_system(gBest, 'FOSMC', 'Disturbed');

% 导出图4 (3D)
fig4 = figure(4); set(gcf, 'Color', 'w');
plot3(data_fosmc.xd, data_fosmc.yd, data_fosmc.zd, 'k:', 'LineWidth', 2); hold on;
plot3(data_pid.x, data_pid.y, data_pid.z, '--', 'LineWidth', 1.5, 'Color', [0.7 0.7 0.7]);
plot3(data_smc.x, data_smc.y, data_smc.z, '-.', 'LineWidth', 1.5, 'Color', [0.4 0.4 0.4]);
plot3(data_fosmc.x, data_fosmc.y, data_fosmc.z, 'k-', 'LineWidth', 2);
grid on; legend('Reference', 'Traditional PID', 'Conventional SMC', 'IPSO-FOSMC', 'Location', 'best');
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); view(-45, 30);
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
print(fig4, 'Figure_4_3D_Tracking.png', '-dpng', '-r300');

% 导出图5 (误差)
fig5 = figure(5); set(gcf, 'Color', 'w');
subplot(3,1,1); plot(data_pid.t, data_pid.x-data_pid.xd, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5); hold on; 
plot(data_smc.t, data_smc.x-data_smc.xd, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
plot(data_fosmc.t, data_fosmc.x-data_fosmc.xd, 'k-', 'LineWidth', 1.5); ylabel('e_x (m)'); grid on; legend('PID', 'Conv-SMC', 'IPSO-FOSMC', 'Location', 'northeast');
subplot(3,1,2); plot(data_pid.t, data_pid.y-data_pid.yd, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5); hold on; 
plot(data_smc.t, data_smc.y-data_smc.yd, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
plot(data_fosmc.t, data_fosmc.y-data_fosmc.yd, 'k-', 'LineWidth', 1.5); ylabel('e_y (m)'); grid on;
subplot(3,1,3); plot(data_pid.t, data_pid.z-data_pid.zd, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5); hold on; 
plot(data_smc.t, data_smc.z-data_smc.zd, '-.', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.5);
plot(data_fosmc.t, data_fosmc.z-data_fosmc.zd, 'k-', 'LineWidth', 1.5); ylabel('e_z (m)'); xlabel('Time (s)'); grid on;
set(findall(gcf,'type','axes'), 'FontName', 'Times New Roman', 'FontSize', 12);
print(fig5, 'Figure_5_Errors.png', '-dpng', '-r300');

% 导出图6 (鲁棒性)
fig6 = figure(6); set(gcf, 'Color', 'w');
fill([10 11 11 10], [-5 -5 5 5], [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5); hold on;
plot(data_robust.t, data_robust.x - data_robust.xd, 'k-', 'LineWidth', 2); yline(0, 'k--', 'LineWidth', 1); ylim([-1.5 1.5]); 
grid on; xlabel('Time (s)'); ylabel('Position Error e_x (m)'); text(10.5, 1.2, 'Wind Gust Pulse', 'HorizontalAlignment', 'center', 'FontName', 'Times New Roman', 'FontSize', 11);
legend('Wind Pulse (3N)', 'Tracking Error', 'Location', 'northeast');
set(gca, 'FontName', 'Times New Roman', 'FontSize', 12);
print(fig6, 'Figure_6_Robustness.png', '-dpng', '-r300');

%% --- 3. 提取所有图像的底层数据并输出报告 ---
fprintf('\n=================================================================\n');
fprintf('  【AI 绘图脑补专用：全息数据采集报告】 请全部复制\n');
fprintf('=================================================================\n');

fprintf('\n[图3: IPSO 适应度收敛全记录 (1-50代)]\n');
for i = 1:50
    fprintf('Iter %2d: %.6f | ', i, fit_hist(i));
    if mod(i, 5) == 0, fprintf('\n'); end
end

fprintf('\n[图5: 各控制器时域误差切片 (ex, ey, ez 单位:米)]\n');
t_samples = [2.0, 5.0, 10.0, 15.0, 20.0];
for ts = t_samples
    idx = round(ts / 0.01) + 1;
    if idx > length(data_pid.t), idx = length(data_pid.t); end
    fprintf('--- 时间 t = %.1f s ---\n', ts);
    fprintf('  PID       : ex=%7.4f, ey=%7.4f, ez=%7.4f\n', data_pid.x(idx)-data_pid.xd(idx), data_pid.y(idx)-data_pid.yd(idx), data_pid.z(idx)-data_pid.zd(idx));
    fprintf('  Conv-SMC  : ex=%7.4f, ey=%7.4f, ez=%7.4f\n', data_smc.x(idx)-data_smc.xd(idx), data_smc.y(idx)-data_smc.yd(idx), data_smc.z(idx)-data_smc.zd(idx));
    fprintf('  IPSO-FOSMC: ex=%7.4f, ey=%7.4f, ez=%7.4f\n', data_fosmc.x(idx)-data_fosmc.xd(idx), data_fosmc.y(idx)-data_fosmc.yd(idx), data_fosmc.z(idx)-data_fosmc.zd(idx));
end

fprintf('\n[图6: 阵风脉冲核心区域高密度采样 (展示吹偏与回零全貌)]\n');
t_robust_samples = [9.5, 9.9, 10.0, 10.1, 10.5, 10.9, 11.0, 11.1, 11.5, 12.0, 13.0, 15.0, 20.0];
fprintf('  时间(s) | X轴位置误差 (m) | 状态解读\n');
for ts = t_robust_samples
    idx = round(ts / 0.01) + 1;
    if idx > length(data_robust.t), idx = length(data_robust.t); end
    err_x = data_robust.x(idx) - data_robust.xd(idx);
    
    state_str = '正常跟踪';
    if ts >= 10.0 && ts <= 11.0, state_str = '>> 受到 3N 强风冲击中'; end
    if ts > 11.0 && ts <= 13.0, state_str = '<< 风停，控制器发力回零中'; end
    if ts > 13.0, state_str = '稳态保持'; end
    
    fprintf('  t=%4.1f | ex = %9.6f | %s\n', ts, err_x, state_str);
end

fprintf('\n=================================================================\n');
disp('>>> 采集完毕！请将以上框内数据发给我。 <<<');