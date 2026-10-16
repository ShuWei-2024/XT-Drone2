# Gazebo 等效传感器

[文档索引](../../../docs/README.md) · [架构与接口](../../../docs/ARCHITECTURE.md)

## 模型与数据

本包提供 Gazebo Harmonic 传感器模型与桥接。飞行链使用 PX4 控制的 x500 挂载 Mid-360 等效 `gpu_lidar`、D435i 等效 RGB/depth 相机与 IMU；传感器独立验证入口也保留。

| ROS 接口 | 类型 | 标称频率 |
| --- | --- | --- |
| `/sensors/lidar/points` | PointCloud2 | 10 Hz |
| `/sensors/camera/color/image_raw` | Image，640 × 480 | 30 Hz |
| `/sensors/camera/depth/image_raw` | Image，640 × 480 | 30 Hz |
| `/sensors/camera/color/camera_info` | CameraInfo | 30 Hz |
| `/sensors/imu/data` | Imu | 100 Hz |

LiDAR 为规则 16 × 720 等效扫描，**不模拟 Livox 非重复扫描与真实发射时间**。银色外壳、黑盖和红标只是显示外观，不是厂商 CAD；不改变传感器原点/安装和碰撞几何。实际输入时间/坐标约定见 [适配器说明](../astra_fast_livo2/README.md)。

## 运行

完整 PX4 场景使用 [快速启动](../../../docs/QUICKSTART.md) 的隔离运行器；它配置资源、传感器桥和机体参考，不要同时另开一套传感器仿真。

仅检查传感器的静态入口，从仓库根目录执行：

```bash
source astra/env_117.bash
export GZ_SIM_RESOURCE_PATH="$HOME/dev/px4/PX4-Autopilot-1.17/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
ros2 launch astra_gz_sensors gz_sensors.launch.py rviz:=false
```

该独立入口不启动 PX4、Agent、FAST-LIVO2 或规划器。飞行模型/世界位于 `models/` 和 `worlds/`，标称 mount TF 在 `config/static_sensor_tf.yaml`，实际飞行参考检查由 `verify_sensor_geometry_117.py` 完成。

静态传感器验证中的 `map → odom → base_link` 固定关系不能代替飞行时的估计位姿树。Gazebo 的图形负载和 ROS 传输可使实际频率偏离标称；用运行日志检查，不只看配置数字。
