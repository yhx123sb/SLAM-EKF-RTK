data = load('aligned_data.mat'); % 假设包含时间戳、RTK 数据和 IMU 数据
aligned_data.time = data.aligned_data.time;
aligned_data.rtk.lat = data.aligned_data.rtk.lat;
aligned_data.rtk.lon = data.aligned_data.rtk.lon;

% 可选 IMU 数据
aligned_data.imu.accel = data.aligned_data.imu.accel; % Nx3 加速度
aligned_data.imu.gyro = data.aligned_data.imu.gyro;   % Nx3 角速度

% 运行 EKF 轨迹融合
traj = run_ekf_2d(aligned_data);

% 输出结果
disp('[✓] 轨迹融合完成');
disp('部分状态结果：');
disp(traj(1:10, :)); % 打印前 10 行

% 提取轨迹中的位置
ekf_x = traj(:, 1); % EKF 融合后的位置 x
ekf_y = traj(:, 2); % EKF 融合后的位置 y

% 绘制轨迹对比
figure;
plot(ekf_x, ekf_y, 'b', 'LineWidth', 1.5); 
hold on;
plot(aligned_data.rtk.lat, aligned_data.rtk.lon, 'r--', 'LineWidth', 1.5);
legend('EKF Trajectory', 'RTK Raw');
xlabel('Latitude (x)');
ylabel('Longitude (y)');
title('EKF Trajectory vs RTK');
grid on;
axis equal;

% 保存 EKF 输出轨迹为文件
output_data = table(traj(:, 1), traj(:, 2), traj(:, 3), ...
                    traj(:, 4), traj(:, 5), ...
                    'VariableNames', {'x', 'y', 'theta', 'v', 'omega'});
writetable(output_data, 'ekf_trajectory.csv');
disp('[✓] 融合轨迹已保存至 ekf_trajectory.csv');
