# Astra：基于 XT-Drone2 的 ROS 2 / PX4 1.17 工程

本分支以 [ShuWei-2024/XT-Drone2](https://github.com/ShuWei-2024/XT-Drone2) 为基础，接入我们已验证的 FAST-LIVO2、局部地图、EGO-Planner 和保护执行桥。项目原始介绍保存在 [docs/README.upstream.md](docs/README.upstream.md)，原 MIT 协议和各模块原有版权说明保留。

**PX4 1.17 已完成完整算法构建、小范围飞行、两次约 33 m 室内自主绕障和一次实际 8 m 横向穿越，并实测地图失联后的保持与降落保护。** 更长七段路线仍出现高度差和偶发 odom 超时保护，长期稳定性尚未通过。结果限于低速静态障碍 SITL，尚未认证无限运行或真实硬件。详见 [运行验收报告](astra/validation/RUNTIME_117.md) 和 [大范围测试报告](astra/validation/LARGE_INDOOR_117.md)。

## 1. 当前版本和目录

| 项目 | 固定版本 / 位置 |
| --- | --- |
| 基础框架 | XT-Drone2 `15b2ef6eb9fd8c2631f931a27064f323d26a9ed3` |
| 开发分支 | `astra/px4-1.17` |
| ROS / Gazebo | ROS 2 Jazzy / Gazebo Harmonic |
| PX4 | `v1.17.0`，提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| PX4 消息 | `release/1.17` 的固定提交，详见 `astra/dependencies.lock.json` |
| Astra ROS 包 | `astra/packages/` |
| ROS 构建目录 | `build_117/`、`install_117/`、`log_117/` |

本机工程目录为 `/home/cyf/dev/XT-Drone2-astra`，PX4 目录为 `/home/cyf/dev/px4/PX4-Autopilot-1.17`。

版本信息和运行时依赖锁定在 `astra/dependencies.lock.json`。本地 SITL 定时器回归结果见 `astra/validation/hrt_regression_117.json`，构建脚本会自动检查相关补丁和回归条件。

## 2. 架构与控制权

```text
XT-Drone2：仿真启动、车辆模型、场景、命名空间
                         ↓
等效 Mid-360 + RGB + IMU → FAST-LIVO2
                         ↓
保护局部地图 → EGO-Planner → B-spline 预览 → 保护执行桥
                                                   ↓
                                               PX4 Offboard
```

External Vision 融合继续关闭。Gazebo 真值只用于审计和独立停测，不作为定位或规划输入。传感器模型是等效仿真，不等于完整 Livox 扫描物理模型。

使用 Astra 执行桥时，关闭 XT-Drone2 默认通信控制节点：`enable_communication:=false`。同一飞机只允许一个 setpoint 输出者。桥默认 `enable_px4_output:=false`，默认不发飞控命令、不解锁。

## 3. 拉取工程

团队成员从团队仓库克隆当前开发分支：

```bash
git clone --branch astra/px4-1.17 https://github.com/ShuWei-2024/XT-Drone2.git XT-Drone2-astra
cd XT-Drone2-astra
git switch astra/px4-1.17
git submodule update --init xtd2_third_party_pkgs/px4_msgs
```

消息子模块指向一个固定提交；更新它后必须把父仓库的指针变化一起提交。`docs/wiki` 是可选文档子模块，构建不需要。原框架缺少地址的 EGO-Planner gitlink 已移除，使用 `astra/packages/ego_planner_jazzy`。

## 4. 构建 PX4 1.17

克隆固定版本的 PX4：

```bash
git clone --branch v1.17.0 --single-branch https://github.com/PX4/PX4-Autopilot.git ~/dev/px4/PX4-Autopilot-1.17
```

本机复用已有 Python 构建环境，只读取它，不修改依赖：

```bash
cd /home/cyf/dev/XT-Drone2-astra
bash astra/build_px4_117.bash
```

可以用 `ASTRA_PX4_DIR`、`ASTRA_PX4_PYTHON_ENV` 指定其他位置。脚本核对固定提交并使用两个编译任务。缺少依赖时应建立独立 Python 环境。PX4 构建会获取需要的子模块；如网络需代理，只为本次命令设置代理，不改全局 Git 配置。

## 5. 构建 ROS 接口并检查

使用干净终端：

```bash
cd /home/cyf/dev/XT-Drone2-astra
bash astra/build_117.bash
source astra/env_117.bash
python3 -c 'from px4_msgs.msg import VehicleLocalPosition, TimesyncStatus; print(VehicleLocalPosition.MESSAGE_VERSION)'
python3 -m pytest -q astra/packages/astra_px4_offboard/test
```

此脚本构建完整 19 包链路，包括 FAST-LIVO2、vikit、EGO-Planner、消息、执行桥和 XT-Drone2。Sophus 依赖保存在 `astra/vendor/Sophus-legacy`，构建和运行使用工程内的依赖。

1.17 的位置和状态消息版本为 1，话题带 `_v1`；版本为 0 的轨迹输入不带后缀，例如：

```text
/x500_0/fmu/out/vehicle_local_position_v1
/x500_0/fmu/out/vehicle_status_v1
/x500_0/fmu/in/trajectory_setpoint
```

桥、预览和审计器按已安装消息的 `MESSAGE_VERSION` 生成话题；不要混用旧消息 overlay。PX4 与 Agent 实际握手、传感器接收、消息版本和地图 readiness 已通过 60 秒静止测试，见 `astra/validation/static_stack_117.json`。最终传感器接口的运动误差另存 `astra/validation/obstacle_reliability_117.json`，采用冻结初始变换，不在飞行数据上重新拟合。

## 6. 仿真入口与验收顺序

完整 Astra 组合入口通过 XT-Drone2 启动增强世界、Gazebo-ROS 传感器桥和 PX4；每次运行使用独立 rootfs/参数目录。

```bash
cd /home/cyf/dev/XT-Drone2-astra
# 在干净终端执行；名称必须从未使用过。
bash astra/run_117.bash static static_check_01 --seconds 60
bash astra/run_117.bash small small_check_01 --calibrate-ground-height
bash astra/run_117.bash obstacle obstacle_check_01 --calibrate-ground-height
# 正、负两个方向，各绕障往返两次。
bash astra/run_117.bash obstacle repeat_check_01 --calibrate-ground-height --route-json astra/config/route_repeat_north_south.json
# 规划控制权交接 3 秒后停止本轮地图进程，验证保持和降落。
bash astra/run_117.bash obstacle map_loss_check_01 --calibrate-ground-height --stop-map-after-handoff-s 3
```

`static` 不解锁、不发送飞控输入。`small` 和 `obstacle` **会解锁本机 PX4 SITL 并执行飞行**，只在隔离仿真中使用。运行器拒绝已有 PX4/Agent 进程，使用 ROS domain 94、独立 Gazebo partition 和 `x500_0` namespace，默认不启动 XT-Drone2 通信控制节点。小范围飞行通过原生 NED setpoint 检查起飞、0.4 m 前进/侧移和返回；绕障模式才启用受保护的 EGO B-spline 执行桥。External Vision 关闭。

运行证据在 `astra/runtime/<名称>/`：`runtime.json` 记录临时参数；`preflight_health.json` 记录起飞前门控；`frame_samples.csv` 保存测量时间与发布时间；`motion_events.json` 保存实际运动、停测原因和降落状态；`log/` 保存 PX4 ULog。运行结束会停止其创建的进程并校验参数/源码基线与执行源码哈希。

起飞前要求传感器、地图和位姿有效且新鲜，测量时间与真值配对在 20 ms 内。收到消息时的 `/clock - timestamp_sample` 是处理/传输年龄，不等于时钟偏移；日志分别记录测量到发布与发布到接收的差值，后者也受回调顺序影响。配对覆盖率检查不等于独立时钟标定。

`--calibrate-ground-height` 在地面采集 4 秒测量时间配对的 FAST-LIVO2/PX4 高度，冻结零点差，再用独立 4 秒数据验证 0.15 m 残差门限。Gazebo 不参与该偏置估计；运行中持续检查相对冻结偏置的高度漂移，不通过重拟合消除误差。未加该标志时保留原零点门控。地图停止试验的预期故障退出单独记录在 `fault_regression.json`，不能当作正常路线完成；本次停图到冻结保持约 294 ms，保持 1 秒后请求降落。

隔离 SITL 的 `EKF2_BARO_NOISE` 默认 0.30 m，与官方模型 3 Pa（近海平面约 0.25 m）的气压噪声相匹配；传感器实际噪声没有降低。可通过 `--baro-noise-m 0.1` 复现此前对照。此参数仅进入每轮独立 rootfs，不修改其他 PX4 配置，也不是实物飞控推荐参数。FAST-LIVO2 的秒到时间戳转换已修复纳秒截断，保持 20 ms 时间配对门限。运行器拒绝重叠 Gazebo（包括改写后的进程名），结束时清理其创建的整个进程组。

磁力计接口为显式 ENU/tesla：世界设置 `use_earth_frame_ned=false`、`use_units_gauss=false`，新 PX4 本地补丁仅在 `ASTRA_GZ_MAG_ENU_TESLA=1` 时将传感器 FLU 转为 FRD、tesla 转为 gauss；运行器自动配套设置。运行器还复制官方 `x500_base` 到每次运行目录，把原 gauss 噪声数值换算成 tesla（乘 1e-4）；物理噪声幅度保持相同。不要单独改其中一侧。补丁保存于 `astra/config/px4_117_gz_magnetometer.patch`，构建脚本会检查或应用。仿真地面站仅发送 localhost MAVLink 心跳，不发送目标或解锁命令；它满足 PX4 的 GCS 健康检查，不关闭该检查。

小范围当前采用原有开阔运动审计模式：到位后观察 4 秒，**不宣称连续 4 秒严格稳定悬停认证**。保留位置、真实位移、数据 250 ms 新鲜度、航向、重置和高度差保护。规划速度 0.20 m/s、加速度 0.25 m/s²；初始航向差必须连续 2 秒不超过 5°。测试保持原保护门槛。

运行和审计工具位于 `astra/tools/`。

## 7. Git 日常怎么操作

- **仓库**：工程和它的修改历史。
- **分支**：同一工程的开发方向，`astra/px4-1.17` 记录我们的开发工作。
- **提交（commit）**：保存一次有说明的改动。
- **推送（push）**：把本地提交上传 GitHub，让老师和同学看到。
- **子模块**：父仓库记录另一个 Git 仓库的准确版本；这里用于 `px4_msgs`。

`origin` 指向老师的团队仓库 `ShuWei-2024/XT-Drone2`，当前工作分支为 `astra/px4-1.17`：

```bash
git push -u origin astra/px4-1.17
```

该命令只更新开发分支。合入团队 `main` 需另行创建 Pull Request 并审核。

日常改代码后先检查，再提交指定文件：

```bash
git status
git diff
git add README.md astra/<本次改动的文件>
git commit -m '说明本次改动和原因'
git push
```

如需设置 Git 作者，可只在这个仓库设置自己的姓名和 GitHub 邮箱：`git config user.name ...`、`git config user.email ...`。

## 8. 当前证据

当前构建与离线检查记录保存在 `astra/validation/`。此目录不保存大型 ULog、构建缓存或录制包；这些运行产物不上传 Git。
