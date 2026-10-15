# 安装与快速启动

[返回文档索引](README.md)

本教程面向 Ubuntu 24.04。所有项目命令在仓库根目录执行；每次仿真单独运行，名称不能重复。`static` 默认不解锁；`small`、`obstacle` 是会解锁并飞行的本机隔离 SITL 测试。

## 1. 系统依赖

先按官方教程配置 [ROS 2 Jazzy 软件源](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 和 [Gazebo Harmonic 软件源](https://gazebosim.org/docs/harmonic/install_ubuntu/)，再安装：

```bash
sudo apt update
sudo apt install git curl build-essential cmake ninja-build ccache pkg-config \
  libssl-dev libxml2-dev libeigen3-dev libpcl-dev libopencv-dev libfmt-dev \
  libboost-thread-dev libmimalloc-dev python3-dev python3-venv python3-pip \
  python3-colcon-common-extensions python3-pytest python3-numpy python3-scipy \
  python3-matplotlib python3-tk ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-tf-transformations ros-jazzy-ament-cmake-gtest \
  gz-harmonic python3-gz-msgs10 python3-gz-transport13
```

使用支持 Ogre2 渲染的显卡驱动。无 GUI 运行仍需要渲染 LiDAR/RGB；关闭窗口不等于无需图形驱动。ROS Python 环境与下文离线地图转换环境分开。

## 2. 克隆当前分支

```bash
git clone --branch astra/px4-1.17 https://github.com/ShuWei-2024/XT-Drone2.git XT-Drone2-astra
cd XT-Drone2-astra
git submodule update --init xtd2_third_party_pkgs/px4_msgs
```

仅初始化所需消息子模块；`docs/wiki` 不参与构建。准确版本见 [依赖锁](../astra/dependencies.lock.json)。

## 3. PX4 与 Python 构建环境

```bash
mkdir -p ~/dev/px4
git clone --branch v1.17.0 --single-branch \
  https://github.com/PX4/PX4-Autopilot.git ~/dev/px4/PX4-Autopilot-1.17
python3 -m venv ~/dev/px4/venv
~/dev/px4/venv/bin/python -m pip install \
  -r ~/dev/px4/PX4-Autopilot-1.17/Tools/setup/requirements.txt
bash astra/build_px4_117.bash
```

脚本核对固定提交、应用工程记录的 HRT/磁力计补丁并以两个编译任务构建。构建按 PX4 规则拉取所需子模块，网络失败先恢复下载再重跑。

`ASTRA_PX4_DIR` 可指定其他 PX4 路径；`ASTRA_PX4_PYTHON_ENV` 可改变**构建**环境。但运行器的 GCS 心跳当前固定使用 `~/dev/px4/venv/bin/python`，首次部署应保留上述环境，不能仅设置构建变量后假定运行路径也改变。

## 4. Micro XRCE-DDS Agent

运行器当前要求下面的用户目录与二进制路径：

```bash
mkdir -p ~/dev/third_party
git clone --branch v3.0.2 --single-branch \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  ~/dev/third_party/Micro-XRCE-DDS-Agent
cmake -S ~/dev/third_party/Micro-XRCE-DDS-Agent \
  -B ~/dev/third_party/Micro-XRCE-DDS-Agent/build
cmake --build ~/dev/third_party/Micro-XRCE-DDS-Agent/build -j2
```

## 5. ROS 构建与静止检查

在干净终端执行，避免继承其他工程 overlay：

```bash
bash astra/build_117.bash
source astra/env_117.bash
python3 -c 'from px4_msgs.msg import VehicleLocalPosition, TimesyncStatus; print(VehicleLocalPosition.MESSAGE_VERSION)'
bash astra/run_117.bash static static_check_01 --seconds 60
```

构建脚本使用工程内 Sophus、顺序构建和两个编译任务，输出到 `build_117/`、`install_117/`、`log_117/`。静止检查输出保存在 `astra/runtime/static_check_01/`。完整回归入口见 [开发指南](DEVELOPMENT.md)。

## 6. 准备室外地图

Baylands 是公共制作的 Gazebo 公园模型，并非真实扫描；准备程序下载固定版本资源、核对 SHA-256、保留许可并生成碰撞/显示资产和测试边界。

```bash
mkdir -p astra/runtime
python3 -m venv astra/runtime/map_tools_venv
astra/runtime/map_tools_venv/bin/python -m pip install \
  -r astra/config/map_tools_requirements.txt \
  numpy scipy Pillow pycollada==0.9.3 shapely==2.2.0
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands.py
```

下载包约 150.7 MB，转换另需内存和磁盘空间。程序拒绝覆盖已有输出。新生成场景包含彩色纹理和灯光；旧生成目录不会随代码更新自动改变外观。下载中断可重用缓存，转换失败则先检查输出并保留诊断，再选择新输出目录。

启动三维交互任务：

```bash
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --external-vision
```

命令自动启动完整飞行链、RViz2 和中文航点窗口；等待 READY、添加目标并点击开始后才起飞。
Gazebo 场景窗口另开终端执行 `GZ_PARTITION=astra_px4_117 gz sim -g`，详见第 8 节。
默认开放起飞点 10 m 内满足下降净空的区域，不代表整张地图已验证。EV 是可选 XY/Z 位置融合；不加该参数时关闭。

本机已准备独立彩色目录时，用以下命令替代上面的启动命令：

```bash
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --scan-dir astra/runtime/maps/baylands_park_coloured \
  --external-vision
```

该目录不随 Git 下载；新电脑使用本节刚生成的默认地图即可。操作见 [操作指南](OPERATIONS.md)。

## 7. 可选：真实室内扫描场景

Replica office4 为真实扫描的裁剪衍生数据，含缺失表面。使用前阅读 [Replica Research Terms](https://github.com/facebookresearch/Replica-Dataset/blob/main/LICENSE)，保留数据许可；项目代码许可不覆盖地图数据。

```bash
mkdir -p astra/runtime/map_candidates/replica_office4
curl -fL --retry 3 \
  'https://huggingface.co/datasets/voviktyl/Replica-SLAM/resolve/7d7ae90b48035aa2e9baa58018f5e090bd6d5dc5/cull_replica_mesh/office4.ply' \
  -o astra/runtime/map_candidates/replica_office4/office4.ply
curl -fL --retry 3 \
  'https://raw.githubusercontent.com/facebookresearch/Replica-Dataset/main/LICENSE' \
  -o astra/runtime/map_candidates/replica_office4/REPLICA_LICENSE.txt
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_replica_office0.py \
  --scene office4 --source astra/runtime/map_candidates/replica_office4/office4.ply \
  --output astra/runtime/maps/replica_office4
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_scan_avoidance.py \
  --scene-dir astra/runtime/maps/replica_office4 --goal-xy 2.4 -1.1 \
  --xy-bounds -0.60 -2.20 2.90 0.50 --takeoff-height-m 0.9 --alignment-height-m 1.60
bash astra/rviz_flight_117.bash indoor_demo_01
```

转换器验证 office4 源文件摘要；脚本名称中的 office0 不限制 `--scene office4`。背景来自导入扫描，实时定位和规划仍使用仿真传感器。具体几何与验证范围见 [室内扫描绕障报告](../astra/validation/REAL_SCAN_AVOIDANCE_117.md)。

## 8. 查看 Gazebo

另开终端加载 Gazebo 环境并连接同一分区：

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

RViz2 用于三维地图与任务编辑；Gazebo 用于查看仿真飞机和场景。仿真完成、进程清理后再运行离线审计。
