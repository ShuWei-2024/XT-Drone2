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

## 获取项目代码

实物电脑或 RK3588 上拉取项目的分支是 `astra/px4-1.17`。当前仓库没有单独的
`hardware` 分支；仿真和实物迁移文档共同维护在这个项目分支中。

首次获取：

```bash
mkdir -p ~/dev
cd ~/dev
git clone --branch astra/px4-1.17 --single-branch \
  https://github.com/ShuWei-2024/XT-Drone2.git XT-Drone2-astra
cd XT-Drone2-astra
git submodule update --init xtd2_third_party_pkgs/px4_msgs
```

已有工作区更新：

```bash
cd ~/dev/XT-Drone2-astra
git fetch origin
git switch astra/px4-1.17
git pull --ff-only
git submodule update --init xtd2_third_party_pkgs/px4_msgs
```

确认分支和消息子模块：

```bash
git branch --show-current
git log -1 --oneline
git submodule status
```

在 Windows + WSL 中，建议把工作区放在 WSL 的 Linux 文件系统（例如 `~/dev`），不要放在
`/mnt/c` 下构建。后者适合临时编辑和复制文件，频繁编译 ROS 2、FAST-LIVO2 时通常会有较大的
文件系统 I/O 开销。

## PX4 源码是独立依赖

项目仓库不包含完整 PX4 源码。需要编译固件的开发机可以单独获取官方 PX4 1.17：

```bash
mkdir -p ~/dev/px4
git clone --branch v1.17.0 --single-branch \
  https://github.com/PX4/PX4-Autopilot.git \
  ~/dev/px4/PX4-Autopilot-1.17
```

这里的 PX4 源码目录和 `XT-Drone2-astra` 是两个不同的目录：

```text
~/dev/XT-Drone2-astra                                  项目代码和文档
~/dev/XT-Drone2-astra/xtd2_third_party_pkgs/px4_msgs     ROS 2 消息定义子模块
~/dev/px4/PX4-Autopilot-1.17                            PX4 固件源码
```

仿真机上的 `astra/build_px4_117.bash` 会应用仿真兼容补丁并构建 SITL，不能直接当作真实
飞控 target 的构建和烧录流程。实物第一阶段以与飞控型号匹配的官方 PX4 1.17 固件为基线，
具体 target、参数和烧录步骤见[PX4 固件与烧录](03_px4_firmware.md)。

## 开发机、RK3588 与飞控的分工

不要求所有程序运行在同一台电脑上。建议采用一套 Git 代码、多个明确分工的环境：

| 设备 | 主要工作 |
| --- | --- |
| 仿真 Ubuntu 电脑 | Gazebo、PX4 SITL、ROS 2 回归测试 |
| Windows + WSL 或另一台 Ubuntu 电脑 | 修改代码、Git、编译 PX4 固件、分析日志 |
| RK3588 | 真实传感器驱动、FAST-LIVO2、局部地图、EGO-Planner、执行桥和 ROS 2/DDS |
| PX4 飞控 | 运行已烧录固件，执行姿态/位置控制和 failsafe |

PX4 固件可以在开发机编译后，通过 QGroundControl、USB/DFU 或厂商工具烧录到飞控；
编译固件的电脑不需要在飞行时继续连接飞控。真实飞行时，RK3588 才是伴随计算机，必须在
RK3588 的 ARM64 原生环境中重新构建并测量 FAST-LIVO2 的频率、延迟、温度和内存。

WSL 可以作为代码和固件开发环境，但不建议作为最终飞行运行环境。USB 串口透传、DDS 网络、
GPU、实时调度和真实传感器驱动在 WSL 中需要额外配置；不要用 x86 WSL 的性能结果替代 RK3588
实测结果。

实物 ROS 2 工作区的基本关系是：

```text
RK3588: 传感器驱动 -> FAST-LIVO2 -> 局部地图 -> EGO-Planner -> Offboard 执行桥
                                                        |
                                                        +-> DDS/串口 -> PX4 飞控
```

## 性能基线

在接入飞控前单独运行 FAST-LIVO2，记录 5 分钟：

- LiDAR、相机和 IMU 实际频率；
- 输入回调间隔和 odom 测量年龄；
- LIO/VIO 处理耗时的 p50/p95/max；
- CPU、GPU、内存、温度和降频；
- 消息缺口和长期内存变化。

真实设备上不能只看平均频率；偶发消息积压和温度降频必须进入飞行前门控。
