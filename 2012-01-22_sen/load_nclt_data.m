function sensor_data = load_nclt_data(data_dir)
    % data_dir: 存放xls文件的文件夹路径

    % 定义所有文件名与字段名的对应
    files = {
        'gps', 'gps';
        'gps_rtk', 'gps_rtk';
        'gps_rtk_err', 'gps_rtk_err';
        'kvh', 'kvh';
        'ms25', 'ms25';
        'ms25_euler', 'ms25_euler';
        'odometry_cov', 'odometry_cov';
        'odometry_cov_100hz', 'odometry_cov_100hz';
        'odometry_mu', 'odometry_mu';
        'odometry_mu_100hz', 'odometry_mu_100hz';
        'wheels', 'wheels';
    };

    % 初始化结构体
    sensor_data = struct();

    % 遍历每个文件并读取
    for i = 1:size(files, 1)
        file_name = files{i, 1};
        field_name = files{i, 2};
        full_path = fullfile(data_dir, [file_name, '.csv']);

        if isfile(full_path)
            disp(['Reading ', full_path, ' ...']);
            sensor_data.(field_name) = readtable(full_path);
        else
            warning(['File not found: ', full_path]);
        end
    end
end
