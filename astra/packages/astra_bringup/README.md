# Astra bringup

这个包是仿真和实物的统一启动入口。它只负责组合已有模块，不实现定位、建图或规划算法。

```text
sim.launch.py
    XTDrone2 Gazebo/PX4
    + FAST-LIVO2 + 局部地图 + EGO-Planner + 预览桥

hardware.launch.py
    真实传感器驱动（由 sensor_launch_file 提供）
    + FAST-LIVO2 + 局部地图 + EGO-Planner + 预览桥
```

## 仿真

```bash
ros2 launch astra_bringup sim.launch.py \
  px4_dir:=~/dev/px4/PX4-Autopilot-1.17 \
  enable_px4_output:=false
```

该入口默认不会向 PX4 输出轨迹，也不会解锁；`enable_px4_output` 只用于隔离 SITL 预览/调试，
正式飞行仍由项目监督器负责健康检查、解锁和任务交接。原有 `astra/run_117.bash` 运行器继续
作为回归测试入口，不受这个包替代。

## 实物准备

```bash
ros2 launch astra_bringup hardware.launch.py \
  adapter_config:=/path/to/adapter_topics.yaml \
  estimator_config:=/path/to/fast_livo2_hardware.yaml \
  sensor_launch_file:=/path/to/real_sensor.launch.py
```

硬件入口默认 `use_sim_time=false`、不启动 Gazebo 时钟、不打开 PX4 输出。真实传感器驱动必须
先发布配置中声明的 LiDAR、图像、CameraInfo 和 IMU 话题，并完成实际外参和时间偏移标定。
`sensor_launch_file` 是可选的 Python launch 文件路径；不提供时，入口只启动算法层，方便先检查
配置和话题连接。

当前 `astra_px4_offboard` 的输出仍限制在隔离 SITL。真实飞控接入必须完成硬件标定、串口/DDS
配置、External Vision 和无桨门控后，再单独实现经过审查的硬件执行后端。
