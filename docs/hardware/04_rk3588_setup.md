# RK3588 伴随计算机环境

[返回实物索引](README.md)

## 环境目标

```text
Ubuntu 24.04 或项目确认的兼容系统
ROS 2 Jazzy
FAST-LIVO2 依赖
Mid-360 / D435i 驱动
Micro XRCE-DDS Agent
项目 ROS 2 工作区
```

实物环境应使用独立的 ROS 2 workspace 和环境脚本，不与仿真工作区混用未记录的 overlay。安装完成后记录：

- `uname -a`、CPU、内存和温度；
- ROS 2、Gazebo/驱动和 `px4_msgs` 版本；
- OpenCV、PCL、Eigen 等依赖版本；
- `/dev` 设备名、网卡名和 IP；
- Agent 启动命令和 systemd/脚本配置。

## 性能基线

在接入飞控前单独运行 FAST-LIVO2，记录 5 分钟：

- LiDAR、相机和 IMU 实际频率；
- 输入回调间隔和 odom 测量年龄；
- LIO/VIO 处理耗时的 p50/p95/max；
- CPU、GPU、内存、温度和降频；
- 消息缺口和长期内存变化。

真实设备上不能只看平均频率；偶发消息积压和温度降频必须进入飞行前门控。
