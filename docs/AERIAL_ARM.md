# X500 空中机械臂模型与测试

本模型面向当前 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 和 PX4 v1.17.0 环境。

## 模型组成

- 模型名：`x500_arm`；默认仿真实体及 ROS 命名空间：`x500_arm_0`。
- 基于现有 X500，保留飞控传感器和四电机配置。
- 三个转动关节：底座 yaw、肩关节、肘关节；两个直线关节组成平行夹爪。
- 加高的四条支腿和两条滑橇为机械臂提供离地间隙。
- 机械臂与新增起落架质量合计 0.457 kg，带质量、惯量、碰撞体和关节限位。
- 专用 PX4 airframe：`22001_gz_x500_arm`，继承 X500 控制分配，悬停推力初值设为 `0.70`。
- `aerial_arm.sdf` 世界包含工作台和一个有物理碰撞的方块。

关节使用 Gazebo Harmonic 原生 JointPositionController，通过 ROS/Gazebo 桥接控制；JointStatePublisher 提供真实关节状态。没有增加 MoveIt 或 ROS 2 control 安装要求。

这是参考几何和动力学模型，未按某一款实机标定。本次验证覆盖关节运动、夹爪空载开合以及飞行；方块的稳定抓取、搬运和负载飞行尚未验证。

## 1. 启动模型：终端一

先结束其他占用当前 PX4 实例、Gazebo 世界或 Agent 8888 端口的仿真。

```bash
source ~/PX4-Autopilot/Tools/setup/xtdrone2_env.sh
source ~/ws_xtd2/install/setup.bash
ros2 launch xtd2_launch aerial_arm_launch.py
```

会打开 Gazebo，显示 X500、机械臂、夹爪和加高起落架。默认启动世界 `aerial_arm`，在同一终端按 Ctrl+C 停止仿真。

无界面启动可使用：

```bash
ros2 launch xtd2_launch aerial_arm_launch.py gui:=false
```

## 2. 手动测试机械臂：终端二

```bash
source ~/PX4-Autopilot/Tools/setup/xtdrone2_env.sh
source ~/ws_xtd2/install/setup.bash

# 水平初始姿态，打开夹爪
ros2 run xtd2_control aerial_arm_control --pose stow --gripper 0.04

# 前伸姿态，闭合夹爪
ros2 run xtd2_control aerial_arm_control --pose reach --gripper 0.00

# 向下伸展，打开夹爪
ros2 run xtd2_control aerial_arm_control --pose down --gripper 0.04

# 回到初始姿态
ros2 run xtd2_control aerial_arm_control --pose stow --gripper 0.04
```

每条命令平滑过渡约三秒，随后检查关节反馈；成功打印 `PASS: arm reached target`，失败返回非零退出码。上述命令不会解锁或起飞。

自定义关节角度（弧度）：

```bash
ros2 run xtd2_control aerial_arm_control --yaw 0.4 --shoulder 0.6 --elbow -0.5 --gripper 0.03
```

范围：yaw `[-1.57, 1.57]`、shoulder `[-0.35, 1.45]`、elbow `[-1.6, 1.6]`。`--gripper` 为两个手指合计的移动行程，范围 `[0, 0.05]` 米；内侧间隙约为 `0.01 + 行程` 米，因此闭合时仍保留约 10 mm 间隙。

`stow` 在当前参考模型中表示三个关节角为零的水平初始姿态。

查看状态：

```bash
ros2 topic echo /x500_arm_0/arm/joint_states --once
```

## 3. 自动测试

保持终端一的仿真运行。测试过程中只运行一个控制程序。

仅验证关节和夹爪，不起飞：

```bash
python -m xtd2_control.aerial_arm_test
```

完整飞行测试会自动解锁、起飞到相对地面约 2 米、悬停时移动机械臂及开合夹爪，最后降落并等待自动上锁：

```bash
python -m xtd2_control.aerial_arm_test --fly --gcs-heartbeat
```

使用上面环境脚本激活的虚拟环境 Python；`pymavlink` 已安装在该虚拟环境中。`--gcs-heartbeat` 提供本地仿真地面站心跳，不关闭 PX4 解锁检查。若使用已连接的 QGroundControl，可省略该选项：

```bash
python -m xtd2_control.aerial_arm_test --fly
```

完整通过时输出 JSON 中的 `"passed": true`。飞行测试依据 `VehicleStatus` 确认模式和解锁状态，而不是只判断命令已经发送。

## 当前验证结果

2026-10-10 在现有虚拟机上完成：

| 检查 | 结果 |
| --- | --- |
| PX4 构建及新 airframe 打包 | 通过 |
| 三个 ROS 包构建 | 通过 |
| SDF 解析及合并基础模型 | Valid |
| 五个关节反馈和命令桥接 | 通过 |
| 初始姿态、前伸、向下及夹爪开合 | 通过 |
| 用户手动控制命令及位置反馈确认 | 四条命令均通过 |
| PX4 水平/高度估计有效 | 通过 |
| Offboard、解锁、2 米起飞 | 通过 |
| 悬停时机械臂运动和夹爪闭合 | 通过 |
| 降落与自动上锁 | 通过 |

这次完整飞行测试记录的最大水平偏移约 0.290 米；这是单次仿真结果，不代表已完成稳定性或实机性能评估。

运行日志、结果和改动前文件保存在 `~/aerial-arm-review/20261010/`。

## 修改后重新构建

当前工程已经构建，可以直接测试。后续修改代码时：

```bash
source ~/PX4-Autopilot/Tools/setup/xtdrone2_env.sh
cd ~/PX4-Autopilot
make px4_sitl -j2
cd ~/ws_xtd2
colcon build --packages-select xtd2_gz_sim xtd2_launch xtd2_control --executor sequential
source ~/ws_xtd2/install/setup.bash
```

新增 airframe 同时注册在 PX4 的 airframes/CMakeLists.txt；仅复制 XTDrone2 仓库不会带上 PX4 目录中的这两项修改。本模型配套配置的副本保存在 `xtd2_gz_sim/px4_airframes/22001_gz_x500_arm`。

参考：[Gazebo Harmonic 关节控制器](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointPositionController.html)、[PX4 1.17 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard)。
