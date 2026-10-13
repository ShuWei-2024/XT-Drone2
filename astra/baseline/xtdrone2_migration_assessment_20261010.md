# 迁移到 XTDrone2 的可行性评估

评估日期：2026-10-10。目标按“将当前 Astra 定位、建图、规划与 PX4 SITL 闭环链路放到 XTDrone2 框架下运行”理解。

## 结论

**迁移可行，推荐分阶段接入 XTDrone2 的仿真启动、模型和场景管理，保留已验证的 FAST-LIVO2、局部地图、Jazzy EGO-Planner 与保护执行桥。** 软件栈基本一致，主要工作是启动组织、传感器桥接、话题命名空间和控制权适配，算法主体可以复用。

本次完成固定版本的源码审查与离线接口探测。未安装替换主工作空间，也未启动 XTDrone2 飞行，因此这是代码级可行性判断，尚不是迁移完成或运行认证。

审查版本：[XTDrone2 main，15b2ef6](https://github.com/andy-zhuo-02/XTDrone2/tree/15b2ef6eb9fd8c2631f931a27064f323d26a9ed3)。该提交记录时间为 2025-08-02。下载了 21 个关键文本文件，并逐个核对 Git blob 哈希与固定提交的仓库树；没有递归下载模型网格、贴图或子模块。

## 兼容性证据

| 项目 | 代码/文档证据 | 迁移判断 |
| --- | --- | --- |
| 系统版本 | `docs/InstallationTutorial.md` 指定 Ubuntu 24.04.1、ROS 2 Jazzy、Gazebo Harmonic（原文 Hermonic） | 与当前工程一致，无需为了框架更换 ROS 发行版 |
| PX4 通信 | `multirotor_communication.py:67` 起使用 `px4_msgs` 和 `/<namespace>/fmu/in,out/...` | 可以保留现有 DDS 方案，通过话题 remap 接入 |
| x500 动力模型 | XTDrone2 的 `xtd2_gz_sim/models/x500/model.sdf` 与本机 PX4 v1.15.3 对应文件字节一致 | 电机与机体集成具备直接复用基础；尚未核查完整 x500_base 及所有资源 |
| 消息契约 | XTDrone2 锁定 px4_msgs `18cbcced...`；离线比较 `TrajectorySetpoint`、`VehicleLocalPosition` 后字段一致 | 两条核心接口可兼容；其余消息仍需统一到本机已验证的 v1.15.3 消息包 |
| 现有传感器 | 当前模型通过 merge include 复用 `model://x500`，附加 LiDAR/RGB/IMU link | 可把已有传感器模型作为 XTDrone2 自定义 x500 变体注册 |
| 场景资源 | `xtd2_gz_sim/setup.py` 递归安装 models/worlds；启动器设置 `GZ_SIM_RESOURCE_PATH` | 可纳入现有纹理墙/柱体场景；资源路径需要合并，避免同名模型被错误覆盖 |

现有 Mid-360/D435i 为等效仿真模型。迁移框架不会自动提升传感器物理真实性，也不能直接证明 RK3588 或真机性能。

## 必须适配的部分

### 1. 自定义模型与启动器

[`px4_launch.py:36`](https://github.com/andy-zhuo-02/XTDrone2/blob/15b2ef6eb9fd8c2631f931a27064f323d26a9ed3/xtd2_launch/xtd2_launch/utils/px4_launch.py#L36) 只登记 `gz_x500:4001` 和 `gz_x500_depth:4002`。我们的带传感器模型需要增加映射，检查 PX4 实际产生的模型名和 motor topic。

同文件默认 PX4 目录是 `~/PX4-Autopilot`，工作目录固定为它的 `build/px4_sitl_default`。迁移应显式传入当前 PX4 路径，并增加独立 rootfs/参数目录和退出清理，保留已有可恢复测试方式。

[`ros2_single_vehicle_demo_launch.py:70`](https://github.com/andy-zhuo-02/XTDrone2/blob/15b2ef6eb9fd8c2631f931a27064f323d26a9ed3/xtd2_launch/launch/ros2_single_vehicle_demo_launch.py#L70) 在 spawn 调用中固定传入 `gz_x500`、`0`、`x500_0`，没有把已声明的 model/id/namespace 参数传下去。自定义模型和命名空间需修改这处接线或使用新的组合 launch。

默认 YAML 的 `model/id/pose` 是可直接读取的同层键，离线解析通过；这处没有确认到阻塞性配置错误。

### 2. 传感器桥接

XTDrone2 随附 `ros_gz_bridge.yaml` 只桥接模型 pose 和 odometry；这不能直接给 FAST-LIVO2 提供 RGB、点云、CameraInfo 和 IMU。

需要复用我们的传感器桥接 YAML、静态 TF、相机内参和输入适配器。在保持传感器安装位置及旋转一致的情况下复用现有外参，再检查实际 CameraInfo、TF 和时间戳。多机阶段再给每个传感器 topic/frame 加前缀。

模型 pose/odometry 可以用于审计，不能将 Gazebo 真值改名后送给 FAST-LIVO2 或规划器作为定位结果。

### 3. PX4 话题和时间模型

当前桥使用 `/fmu/in/...`、`/fmu/out/...`；XTDrone2 启动器设置 `PX4_UXRCE_DDS_NS=x500_0` 后，接口变为 `/x500_0/fmu/...`。需同时 remap 执行桥、预览、飞行监督器和审计节点。

初期保留当前 `UXRCE_DDS_SYNCT=0` + 仿真时间方案，给相关节点设置 `use_sim_time=true`，继续按 `timestamp_sample`/LiDAR 测量时间比较位姿。XTDrone2 通信节点使用自己的 ROS clock，不能直接假定它继承了我们已经验证的时间配置。

固定 ENU/NED 轴转换只适用于当前零航向出生位置。换出生位置、航向或发生 EKF reset 后，需要重新建立并审核初始变换。

### 4. 执行保护与控制权

离线运行原始方法后确认：

- [`multirotor_communication.py:83`](https://github.com/andy-zhuo-02/XTDrone2/blob/15b2ef6eb9fd8c2631f931a27064f323d26a9ed3/xtd2_communication/xtd2_communication/multirotor_communication.py#L83) 在启用 Offboard 后每 50 ms 重发最后一个 `self.cmd`，重写发布时间；这条路径没有我们所需的 odom、地图 readiness、规划 heartbeat 或命令年龄保护。
- [`hover():307`](https://github.com/andy-zhuo-02/XTDrone2/blob/15b2ef6eb9fd8c2631f931a27064f323d26a9ed3/xtd2_communication/xtd2_communication/multirotor_communication.py#L307) 只输出日志。离线探测确认状态和旧位置目标均未改变，也没有发布悬停指令。
- `cmd_pose_local_flu_callback` 使用当前 PX4 heading 做 FLU 转换，不等于规划器的 `camera_init` → PX4 local NED 固定坐标变换。规划轨迹不能直接送入该接口代替已有变换。
- 命令服务在发布指令后直接返回 success，没有把 PX4 ack/实际状态作为成功条件；已有监督器仍需检查真实 arming/nav_state。

推荐新增一个组合 launch：让 XTDrone2 提供仿真/车辆启动，运行我们的保护桥作为唯一轨迹输出者，并将默认 `multirotor_communication` 设为可选关闭。若老师要求统一使用 XTDrone2 的通信接口，则需明确控制权切换，并把故障锁存、过期检测和悬停/降落语义迁入该接口；工作量更大。

### 5. 子模块与依赖

Git 树存在 EGO-Planner gitlink：`xtd2_third_party_pkgs/motion_planning/ego-planner-swarm`，指向 `413a3f73...`；但 `.gitmodules` 未登记该路径的 URL，GitHub 内容 API 也返回 `submodule_git_url=null`。不能假定递归拉取后就得到可用规划器。当前 launch 中没有发现已连接 FAST-LIVO2/局部地图/B-spline 的完整链路。

迁移先使用我们已验证的 `JIMT111/ego-planner-swarm-jazzy-fixed` 及本地保护补丁；如要替换版本，需重新验证尾段碰撞、速度/加速度约束和 readiness 撤销。

本机 `rclpy`、`launch`、`launch_ros`、`ros_gz_bridge.actions` 可导入；`tf_transformations` 未安装，而 XTDrone2 通信节点依赖它。后续应补齐依赖或采用已有四元数转换实现。本次没有安装系统软件。

多机另外存在 `VehicleCommand.target_system=1` 硬编码，以及当前全局传感器 topic/frame 的复用问题。单机 instance 0 可先验证；多机需要单独适配与审计，不能直接承诺可用。

## 推荐迁移路线与验收

1. **独立工作空间与固定版本。** 保留原工程作为可回退对照；锁定 XTDrone2 commit、PX4 v1.15.3、匹配 px4_msgs 和现有算法版本。启动一个世界、一个 DDS Agent、一架 instance 0 飞机，避免重复实例及端口冲突。
2. **先运行传感器和定位。** 注册带传感器 x500，在原增强场景下核对频率、测量时间、相机图像和 TF；先验收静止定位与地图。
3. **规划预览与控制权检查。** 规划器只生成预览，确认命名空间、ENU/NED 变换、地图 readiness 和故障撤销正确，确保同一架 PX4 只有一个 setpoint 输出者。
4. **隔离 SITL 飞行回归。** 起飞、短距离前进/侧移、单目标绕障、降落；复测 odom 过期、地图节点退出、估计器 reset、轨迹失效后的保持与降落。参数和模型恢复后再切换主入口。

沿用原有 0.30 m XY 分歧门限、250 ms odom 新鲜度、0.45 m 点云盲区和低速规划配置。误差按测量时间分阶段审计，距离覆盖场景全部障碍物。

基准是增强场景 Run 31/32：FAST-LIVO2/Gazebo XY p95 为 0.0051/0.0061 m；最大 PX4 分歧为 0.116/0.169 m；视觉特征零帧数均为 0。实际速度峰值 0.235/0.219 m/s 和 PX4 本身的估计误差仍需保留记录。迁移不应以放宽门限来获得通过。

## 工作量预估

以下是单人熟悉当前工程、同一电脑和同一软件版本条件下的工程估算，包含验证，不是进度承诺。

| 范围 | 预估投入 |
| --- | --- |
| XTDrone2 启动自定义传感器模型，保留现有算法与保护桥，完成首次单机闭环 | 约 2～5 个工作日 |
| 完成异常回归、版本锁定、参数恢复、复现文档并作为常用入口 | 累计约 5～10 个工作日 |
| 全面改用 XTDrone2 通信接口，或同时扩展多机与全新 PX4/算法版本 | 约 2 周以上，需另定范围 |

值得迁移的收益是统一团队的仿真入口、场景和模型管理，以及为后续多机实验建立框架。单机定位精度和避障性能仍取决于我们的感知、规划与保护链路，不会因为换框架自动改善。

## 本次验证材料

固定源码快照：`work/xtdrone2_migration_review_20261010/source/`。

- `upstream_commit.json` / `upstream_tree.json`：版本与完整仓库树。
- `source_inventory.json`：21 个下载文件的哈希与版本匹配结果。
- `offline_probe_results.json`：模型映射、HOVER 行为、旧指令重发和两条消息 schema 比较。
- `local_import_checks.json`：本机依赖导入结果。
- `probe_interfaces.py`：离线探测脚本；不创建 ROS publisher、不启动 PX4。

```bash
cd /home/cyf/Documents/Codex/2026-10-06/home-cyf-dev-astra-ros2-ws
source /opt/ros/jazzy/setup.bash
source /home/cyf/dev/astra_ros2_ws/install_timesync_msg/local_setup.bash
python3 work/xtdrone2_migration_review_20261010/probe_interfaces.py
```
