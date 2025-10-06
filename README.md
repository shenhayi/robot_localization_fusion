# Robot Localization Fusion Package

这个ROS2包使用robot_localization来融合Odometry和IMU数据，输出30Hz的融合odometry。

## 功能特性

- 融合轮式里程计（/Odometry）和IMU数据（/livox/imu）
- 输出30Hz的融合odometry（/odom）
- 基于EKF（扩展卡尔曼滤波）的传感器融合
- 支持3D定位和姿态估计

## 依赖项

- ROS2 Foxy
- robot_localization
- nav_msgs
- sensor_msgs
- geometry_msgs
- tf2

## 安装

1. 将此包放在你的ROS2工作空间的src目录下
2. 安装依赖项：
   ```bash
   sudo apt install ros-foxy-robot-localization
   ```
3. 编译包：
   ```bash
   cd /path/to/your/workspace
   colcon build --packages-select robot_localization_fusion
   source install/setup.bash
   ```

## 使用方法

### 启动融合节点

```bash
# 使用默认配置启动
ros2 launch robot_localization_fusion odom_imu_fusion.launch.py

# 使用自定义配置文件启动
ros2 launch robot_localization_fusion odom_imu_fusion.launch.py config_file:=/path/to/your/config.yaml
```

### 输入话题

- `/Odometry` (nav_msgs/Odometry): 轮式里程计数据
- `/livox/imu` (sensor_msgs/Imu): IMU数据

### 输出话题

- `/odom` (nav_msgs/Odometry): 融合后的30Hz里程计数据
- `/pose` (geometry_msgs/PoseWithCovarianceStamped): 融合后的位姿数据

## 配置说明

配置文件位于 `config/ekf_fusion.yaml`，主要参数包括：

- `frequency`: 滤波器频率（30Hz）
- `odom0`: 里程计输入话题
- `imu0`: IMU输入话题
- `process_noise_covariance`: 过程噪声协方差矩阵
- `initial_state_covariance`: 初始状态协方差矩阵

## 坐标系配置

- `map_frame`: "map"
- `odom_frame`: "odom" 
- `base_link_frame`: "base_link"
- `world_frame`: "odom"

## 调试

启用调试模式：
```bash
ros2 launch robot_localization_fusion odom_imu_fusion.launch.py config_file:=/path/to/config.yaml
```

查看融合状态：
```bash
ros2 topic echo /odom
ros2 topic echo /pose
```

## 注意事项

1. 确保输入话题的坐标系配置正确
2. 根据实际传感器特性调整噪声参数
3. 检查IMU和里程计的时间同步
4. 确保tf树配置正确

## 故障排除

如果遇到问题，请检查：
1. 输入话题是否正常发布
2. 坐标系变换是否正确
3. 配置文件参数是否合理
4. 查看节点日志输出
