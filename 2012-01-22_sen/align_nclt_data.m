function aligned_data = align_nclt_data(sensor_data)
    % 使用 GPS 时间戳（Var1）作为主时间轴
    ref_time_us = sensor_data.gps.Var1;     % 微秒时间戳
    ref_time = double(ref_time_us) / 1e6;    % 转换为秒
    aligned_data.time = ref_time;            % 保存统一时间轴
    
    %% 对 RTK GPS 进行插值（lat/lon/alt）
    rtk_time = double(sensor_data.gps_rtk.Var1) / 1e6;
    aligned_data.rtk.lat  = interp1(rtk_time, sensor_data.gps_rtk.Var4, ref_time, 'linear', 'extrap');
    aligned_data.rtk.lon  = interp1(rtk_time, sensor_data.gps_rtk.Var5, ref_time, 'linear', 'extrap');
    aligned_data.rtk.alt  = interp1(rtk_time, sensor_data.gps_rtk.Var6, ref_time, 'linear', 'extrap');

    %% 对 IMU（ms25）插值：加速度 + 角速度
    imu_time = double(sensor_data.ms25.Var1) / 1e6;
    aligned_data.imu.accel = interp1(imu_time, sensor_data.ms25{:, 5:7}, ref_time, 'linear', 'extrap');
    aligned_data.imu.gyro  = interp1(imu_time, sensor_data.ms25{:, 8:10}, ref_time, 'linear', 'extrap');

    %% 对 odometry_mu_100hz 插值（x, y, z, roll, pitch, yaw）
    odo_time = double(sensor_data.odometry_mu_100hz.Var1) / 1e6;
    aligned_data.odom.pose = interp1(odo_time, sensor_data.odometry_mu_100hz{:, 2:7}, ref_time, 'linear', 'extrap');

    %% 可选：插值轮速、FOG、GPS fix 模式等更多数据
    % 同样用 interp1(...) 方法处理

    disp('[✓] 传感器数据插值完成，aligned_data 已生成。');
end
