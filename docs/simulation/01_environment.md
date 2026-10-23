# 仿真环境

[返回仿真索引](README.md)

仿真环境的完整安装命令见[仿真快速启动](../QUICKSTART.md)第 1 至 4 节。固定基线为：

| 组件 | 基线 |
| --- | --- |
| 操作系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 仿真器 | Gazebo Harmonic |
| 飞控 | PX4 v1.17.0 SITL |
| 通信 | Micro XRCE-DDS Agent + `px4_msgs` |
| 运行分支 | `astra/px4-1.17` |

大型地图、运行日志和构建产物在本机生成，不随 Git 下载。仿真环境和实物环境不要共用未记录的 overlay；每次运行前加载 `astra/env_117.bash`。
