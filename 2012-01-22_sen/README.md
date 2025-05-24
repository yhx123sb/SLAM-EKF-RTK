# NCLT 数据集轨迹融合项目

本项目实现了基于 NCLT (North Campus Long-Term) 数据集的传感器数据融合和轨迹估计。使用扩展卡尔曼滤波器 (EKF) 对多种传感器数据进行融合，实现高精度的轨迹估计。

## 项目结构

```
.
├── 2012-01-22/          # 原始数据目录
├── aligin_nclt_data_main.m    # 数据对齐主程序
├── align_nclt_data.m    # 数据对齐实现
├── ekf_main.m           # EKF 主程序
├── load_nclt_data.m     # 数据加载函数
├── read_nclt_data.m     # 数据读取函数
├── run_ekf_2d.m         # 2D EKF 实现
├── timestamp.m          # 时间戳处理
├── aligned_data.mat     # 对齐后的数据
├── ekf_trajectory.csv   # EKF 轨迹输出
├── output.jpg          # 可视化结果
└── sensor_data.mat     # 传感器数据
```

## 功能特点

- 支持多种传感器数据读取和处理：
  - GPS/RTK 数据
  - IMU 数据 (KVH, MS25)
  - 里程计数据
  - 轮速数据
- 实现传感器数据的时间对齐
- 使用扩展卡尔曼滤波器进行轨迹估计
- 支持 2D 轨迹估计
- 提供可视化结果输出

## 使用方法

1. 数据准备
   ```matlab
   % 加载原始数据
   sensor_data = load_nclt_data('2012-01-22');
   ```

2. 数据对齐
   ```matlab
   % 运行数据对齐
   aligned_data = align_nclt_data(sensor_data);
   ```

3. 运行 EKF
   ```matlab
   % 执行轨迹估计
   trajectory = run_ekf_2d(aligned_data);
   ```

## 依赖要求

- MATLAB R2019b 或更高版本
- 需要安装的 MATLAB 工具箱：
  - Statistics and Machine Learning Toolbox
  - Signal Processing Toolbox

## 输出说明

- `aligned_data.mat`: 时间对齐后的传感器数据
- `ekf_trajectory.csv`: EKF 估计的轨迹数据
- `output.jpg`: 轨迹可视化结果

## 注意事项

1. 确保数据文件路径正确
2. 数据对齐过程可能需要较长时间
3. 建议使用 SSD 存储数据以提高读取速度

## 许可证

本项目采用 MIT 许可证 