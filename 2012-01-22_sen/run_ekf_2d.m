function traj = run_ekf_2d(aligned_data)
    N = length(aligned_data.time);
    dt = diff(aligned_data.time);
    
    % 初始状态
    x = zeros(5, 1);              % [x; y; theta; v; omega]
    P = eye(5) * 1e-2;            % 初始协方差

    Q = diag([0.1 0.1 0.1 0.1 0.1]);   % 过程噪声协方差
    R = diag([1.0 1.0]);                 % 观测噪声（RTK）

    traj = zeros(N, 5);          % 保存所有状态
    traj(1,:) = x';

    for k = 2:N
        % 时间差
        dt_k = dt(k-1);

        % --- 状态预测 ---
        theta = x(3);
        v     = x(4);
        omega = x(5);

        x_pred = [x(1) + v * cos(theta) * dt_k;
                  x(2) + v * sin(theta) * dt_k;
                  theta + omega * dt_k;
                  v;
                  omega];

        % 雅可比矩阵 F
        F = [1 0 -v*sin(theta)*dt_k cos(theta)*dt_k 0;
             0 1  v*cos(theta)*dt_k sin(theta)*dt_k 0;
             0 0 1 dt_k 0;
             0 0 0 1 0;
             0 0 0 0 1];

        P = F * P * F' + Q;

        % --- 观测更新（RTK）---
        z = [aligned_data.rtk.lat(k);
             aligned_data.rtk.lon(k)];

        H = [1 0 0 0 0;
             0 1 0 0 0];

        y = z - H * x_pred;
        S = H * P * H' + R;
        K = P * H' / S;

        x = x_pred + K * y;
        P = (eye(5) - K * H) * P;

        traj(k, :) = x';
    end

    disp('[✓] EKF 2D 轨迹融合完成');
end
