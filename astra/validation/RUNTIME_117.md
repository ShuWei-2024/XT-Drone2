# PX4 1.17 / XT-Drone2 运行验收：2026-10-10

完整感知与规划源码已在新框架构建并运行。完成了一次小范围原生 NED 飞行；最终磁力计接口修正仅通过静止检查，绕障执行在起飞前被高度门控拦住。不能把旧 1.15 飞行或本次全程拟合报告当成最终 1.17 闭环认证。

## 已完成的移植

- 19 个 ROS 包和 PX4 v1.17.0 编译通过；使用固定 release/1.17 消息及版本话题后缀。
- XT-Drone2 负责增强世界、模型和 PX4 启动；完整 FAST-LIVO2、局部地图、EGO-Planner、审计和保护桥已接入。
- 旧版 Sophus 已独立保存并构建，修复缺少 `libSophus.so` 的运行依赖。
- 官方 1.17 在线定时器回归复现 `calls=1 sentinel=0 FAIL`；移植 HRT 补丁后 `calls=21 sentinel=50 PASS`。该试验复现调度缺陷，不证明所有历史传感器中断均由它导致。
- 每轮使用独立 rootfs、ROS domain 94、Gazebo partition；测试临时参数不写入旧版 BSON。
- localhost MAVLink GCS 心跳满足地面站健康检查；不发送目标、setpoint 或解锁命令。实际飞行命令仍由单一试验驱动负责。

## 静止与小范围运行证据

`static_stack_02` 连续 60 秒：LiDAR 约 10 Hz、RGB 约 30 Hz、IMU 约 100 Hz、LIVO odom 约 10 Hz、PX4 local position 约 50 Hz；odom 测量年龄 p95 24 ms，readiness 100%，仿真实时率约 1.0。见 `static_stack_117.json`。这不证明运动内存平台化。

`small_motion_05` 完成起飞、前进、侧移、返回、降落；无 armed failsafe。该轮仍采用官方 Harmonic 磁力计兼容路径：

| 运动段（5 秒目标渐变） | Gazebo 实际 ENU 位移 | LIVO XY p95 | PX4 XY p95 | PX4 Z p95 |
| --- | --- | --- | --- | --- |
| NED north | [-0.037, +0.336, -0.006] m | 0.0030 m | 0.0494 m | 0.1030 m |
| NED east | [+0.360, -0.031, -0.021] m | 0.0044 m | 0.0360 m | 0.0878 m |
| 返回 | [-0.304, -0.296, -0.037] m | 0.0033 m | 0.0694 m | 0.1120 m |

误差用前进前 2 秒的固定平移偏置、已知 ENU/NED 方向计算，不在后续飞行上重拟合。端点观察后的完整位移另存 `small_motion_before_mag_fix_117.json`；上表不是把控制目标 0.4 m 当成真实位移。测量时间配对 p95 8 ms。航向误差约 5～6°，运动开始后没有 XY/Z/yaw reset；爬升期间发生过一次已处理的 yaw reset。

本轮采用开阔区域到位后观察模式，不是连续 4 秒严格稳定悬停认证。该审计不认证整个坐标变换。全程拟合曾返回 `certified=true`，现已纠正：全程拟合只能返回 `fitted_alignment_checks_passed`，最终认证使用独立的固定初始变换。

## 修复及失败记录

1. 被动检查原先把 `/clock - timestamp_sample` 的消息年龄当成时钟偏移，错误使用 20 ms 阈值。修正为分别记录测量到发布、发布到最后收到的 clock，并使用 20 ms 测量时间配对覆盖检查；保留 250 ms 新鲜度门限。异步 clock 回调滞后也会制造假的负年龄，不能据此声称 PX4 测量来自未来。配对覆盖率本身不等于独立时钟偏移认证。
2. Gazebo Harmonic 默认磁力计世界场为 NED、输出 gauss；官方 PX4 兼容处理交换轴。Astra 显式指定世界场 ENU/tesla，配套本地仿真桥按 body FLU→FRD、tesla→gauss 转换。官方模型原 gauss 噪声数值也等比例换算为 tesla，没有减小物理噪声。首次转换漏了噪声单位，磁场检查拦住了测试、没有解锁；已修正并保留记录。
3. 最终接口的 `small_mag_enu_02` 和 `obstacle_mag_enu_01` 静止航向误差 p95 约 1°，磁场强度约 0.481 gauss，接近 PX4 参考 0.482 gauss；**未取得该接口下的成功飞行**。
4. `obstacle_mag_enu_01` 因地面高度偏置约 0.25 m 超过 0.15 m 门限，起飞前停止；没有 EGO 控制权交接、没有实际绕障。详见 `obstacle_attempt_117.json`。
5. `EKF2_GPS_CTRL=5` 是水平位置与三维速度，原本就没有 GPS 高度融合。试验中一度把位掩码读错，改为 1；该值关闭的是速度辅助，不能证明 GPS 高度影响。已恢复 5，失败实验保存在 `preflight_diagnostics_117.json`。
6. 气压噪声 0.1→0.3 m 的单因素试验没有解决地面 Z 偏置，已恢复默认测试值 0.1。保留 `--baro-noise-m 0.3` 作为显式对比入口，不能宣称该值修复了高度。

下一项应分解地面高度的固定原点/初始气压偏置与随时间漂移，结合 `ref_alt`、气压高度和测量时间。`ground_height_diagnostics_117.json` 已保存不同测试的分布。没有放宽高度、初始航向、数据新鲜度或障碍物保护来取得通过。

## 命令和日志

在未加载旧 ROS overlay 的终端：

```bash
cd /home/cyf/dev/XT-Drone2-astra
bash astra/build_px4_117.bash
bash astra/build_117.bash
bash astra/run_117.bash static static_check_02 --seconds 60
bash astra/run_117.bash small small_check_02
bash astra/run_117.bash obstacle obstacle_check_02
# 明确的单因素比较，非默认：
bash astra/run_117.bash small baro_compare_02 --baro-noise-m 0.3
```

飞行模式会对本机 SITL 解锁，必须使用从未用过的运行名称。官方 PX4 来源与本地补丁均由构建脚本核对。最终磁力计路径通过环境变量 `ASTRA_GZ_MAG_ENU_TESLA=1` 启用；原官方转换作为默认备用路径保留。

每轮完整日志在 `astra/runtime/<名称>/`：`runtime.json`、`simulator.log`、`preflight_health.json`、`frame_samples.csv`、`receipt_summary.json`、`motion_events.json` 和 PX4 `log/**/*.ulg`。完整日志和构建缓存不进 Git，结构化证据保存在本目录。

27 项桥、启动器和被动时间检查回归通过；此前完整算法的 3 项 CTest 与 11 项适配器/地图测试通过。旧 PX4 提交、已有源码修改状态、参数文件及 251 个 ROS 原源码哈希均未改变，见 `baseline_preservation_check.json`。所有本轮仿真进程已停止。新 PX4 的补丁留在独立 checkout，并以 patch 文件保存在新框架；GitHub 尚未推送。
