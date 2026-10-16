# 操作与排障

[返回文档索引](README.md) · 首次部署先读 [快速启动](QUICKSTART.md)。

## RViz2 交互任务

```bash
# 使用已准备的地图；运行名称每次不同。
cd ~/dev/XT-Drone2-astra
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --external-vision
# 或室内 office4
bash astra/rviz_flight_117.bash
```

启动器自动打开 RViz2 和中文航点窗口，同时启动完整定位、建图、规划和 PX4 链路。
另开终端显示 Gazebo：

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

本机保留旧灰色默认地图时，用下面这条替代室外启动命令，使用已复测的独立彩色目录：

```bash
cd ~/dev/XT-Drone2-astra
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --scan-dir astra/runtime/maps/baylands_park_coloured \
  --external-vision
```

新电脑按快速启动生成的默认地图已含颜色修复，无需这个额外目录。每次只启动一套仿真。

1. 等待起飞前检查通过，任务窗口显示 READY；此时飞机仍未解锁。
2. 在可旋转的三维视图中用 **2D Goal Pose / G** 选 XY，或在中文窗口输入坐标；检查航点队列。
3. 选择“避障返航后降落”“末点降落”或“末点悬停等待追加”，点击 **开始任务**。初始爬升进行航向校准，再下降到巡航高度。
4. 空中新增航点加入队尾；暂停后会保持，继续会请求新规划。末点降落不自动返回起飞点。
5. 可点击“结束任务：避障返航并降落”。请求原地降落时，若处于通行区且不满足下降检查，会转为规划返航。关闭任务窗口仍触发故障保护；不要以关闭窗口代替正常返航。等待落地上锁和运行器清理，再开始下一轮。

| 限制 | 当前交互行为 |
| --- | --- |
| 航点数量 | 每轮最多 8 个；相邻目标至少 0.15 m |
| 航点范围 | 室内默认 3 m；室外全图配置检查 XY ±30 m、距离上限 43 m，还须通过连通空域与实时障碍检查；旧配置 10 m |
| 高度 | 全图配置相对高度 1.50 m，旧室外配置 0.90 m；起始航向对齐爬升 1.60 m |
| 地面编辑 | 默认不限时，不点击开始就不解锁 |
| 空中等待 / 暂停 | 默认 60 秒，可选 30～120 秒；不会无限悬停 |
| 界面存活 | 心跳 2 Hz，失联 2 秒触发保护结束 |
| 电池 | 低于 20% 或电池消息过期触发保护 |

**图层说明**：灰色参考网格每格 0.5 m，室外覆盖 70×70 m，不表示允许飞行边界。青色轮廓／填色来自同一冻结规划端点掩码，已扣除跟踪／规划余量，并保守排除跨越半径限制的格子；这是静态几何范围，点击仍检查实时障碍、传感器与任务状态。场景背景来自导入模型；FAST-LIVO2 彩色观测来自实时传感器；绿色地面格为预检查下降净空区域；航点球到达后变绿表示完成。机载 RGB 是仿真相机画面。它们都不是“整张地图任意位置已认证可飞”的标记。

独立通行场景另显示蓝色巡航区域；可通行不等于可原地下降。启动器依次优先使用存在的 `baylands_whole_map_refined`、`baylands_whole_map`、旧地图；准备和范围见 [全图空域与验证](../astra/validation/WHOLE_MAP_AIRSPACE_117.md)。健康返航通过 EGO，官方 RTL 只在已知直线走廊满足条件时使用，不会穿树返航。

RViz 的按钮名称“2D Goal Pose”只表示点选 XY，显示本身是三维。当前不通过该按钮自由设定 Z。完整交互验收见 [RViz2 飞行报告](../astra/validation/RVIZ_GOAL_FLIGHT_117.md)。

## 自动测试入口

```bash
bash astra/run_117.bash static static_check_01 --seconds 60
bash astra/run_117.bash small small_check_01 --calibrate-ground-height
bash astra/run_117.bash obstacle outdoor_route_01 \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_park \
  --route-json astra/runtime/maps/baylands_park/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus
```

`small` 使用原生 NED 小范围测试，`obstacle` 才执行 EGO 轨迹。EV 默认关闭，加入 `--external-vision` 启用 XY/Z 融合，yaw/速度不融合。`--executor-audit` 记录回调调度；CPU 隔离是仿真调度测试配置，不代表实物优化已完成。

全图准备场景的 `obstacle` 模式默认使用 Jazzy 事件执行器，减少接收积压；其他场景默认不变。可用 `--execution-executor ready-batch` 显式回退。依赖能力在仿真启动前检查，实际选择写入运行记录。验证及限制见[接收积压复测](../astra/validation/WHOLE_MAP_RECEIVER_117.md)。

事件接收现覆盖 RViz 地面选点、起飞前预发送、爬升和降落恢复；区域显示缓存随航点变化更新，命令不变。遇到按开始后报 `receipt age`，先检查本轮 `motion_events.json` 的失败阶段与接收／测量年龄，不能直接归因于算法耗时。用户运行及修复证据见 [交互起飞接收报告](../astra/validation/RVIZ_PREFLIGHT_RECEIVER_117.md)。

小范围入口允许到位后观察，不宣称已通过“连续 4 秒严格稳定悬停”认证。不要把普通 hold 段与严格稳定 hold 混写。

## 扩大场景

下例生成独立东侧八航点场景，不修改默认交互地图：

```bash
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands_expanded.py \
  --source astra/runtime/maps/baylands_park \
  --output astra/runtime/maps/baylands_expanded_east --route-kind east
bash astra/run_117.bash obstacle east_route_01 \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_expanded_east \
  --route-json astra/runtime/maps/baylands_expanded_east/route.json \
  --calibrate-ground-height --external-vision --executor-audit --isolate-estimator-cpus
```

可选 `east`、`west`、`tree`、`shared-tree`；树旁精查选项为 `--refine-descent-prisms`。准备通过只说明几何前置检查成功，不代表路线飞行通过。当前树旁长绕行可能超过单段超时；不要直接延长超时后宣称规划问题解决。

## 运行证据与审计

证据写入 `astra/runtime/<运行名>/`，此目录不上传 Git：

| 文件 | 用途 |
| --- | --- |
| `runtime.json` | 临时参数、配置、进程与基线信息 |
| `preflight_health.json` | 起飞前健康门控 |
| `frame_samples.csv` | Gazebo、LIVO、PX4 位姿与时间记录 |
| `motion_events.json` | 控制交接、航点、保护、降落事件 |
| `livo_timing.csv`、`input_audit.csv` | 算法阶段、输入同步和回调耗时 |
| `planner.log`、`simulator.log`、`log/` | 规划日志、PX4 输出与 ULog |
| `scan_flight_audit.json` | 航迹、曲线、距离、精度与完整路线判定 |
| `external_vision_audit.json` | EV 时间配对、融合标志、重置与误差 |
| `long_route_drift_audit.json`、`long_route_drift.png` | 分阶段和航段漂移 |

停止仿真和 Gazebo GUI 后执行离线审计。在独立环境安装 ULog 依赖，保留系统 ROS Python 包：

```bash
python3 -m venv --system-site-packages astra/runtime/audit_venv
astra/runtime/audit_venv/bin/python -m pip install pyulog
source astra/env_117.bash
astra/runtime/audit_venv/bin/python astra/tools/audit_scan_flight.py astra/runtime/outdoor_route_01
# 以下 EV 审计仅适用于开启 EV 的运行；共享边界审计适用于带边界的运行。
astra/runtime/audit_venv/bin/python astra/tools/audit_external_vision.py astra/runtime/outdoor_route_01
astra/runtime/audit_venv/bin/python astra/tools/audit_shared_envelope.py astra/runtime/outdoor_route_01
# 漂移审计依赖 scan + EV 两份结果。
astra/runtime/audit_venv/bin/python astra/tools/audit_long_route_drift.py astra/runtime/outdoor_route_01
```

判定时同时查看实际位移、全程安全距离、动态约束、航点误差、时间覆盖率、失败事件和落地状态。退出码 0 或“8/8 航点完成”不代表完整精度审计通过。

## 常见问题

| 现象 | 先看什么 / 怎么处理 |
| --- | --- |
| `Passive health gate failed` | 看 `preflight_health.json` 和各节点日志；确认传感器、odom、地图有效；此阶段未发解锁命令 |
| 进入任务后立即退出 | 查 `motion_events.json`、任务窗口/进程日志，区分取消、UI 失联、目标拒绝、门控失败 |
| 起飞后没有横向飞向航点 | 检查 `planner.log` 的目标拒绝原因。边缘点曾通过界面但不满足规划器额外邻域，现已在入队前拒绝；详见 [边缘航点诊断](../astra/validation/RVIZ_EDGE_GOAL_117.md) |
| `Disarmed EV/global-origin initialization failed` | 查当轮 `global_origin.json`：ACK、连续健康确认和输入年龄；未解锁。临时数据问题在原期限内重新等待，持续异常仍拒绝，见原点启动报告 |
| 色块内航点仍被拒绝 | 青色才是扣除规划余量后的静态范围；绿／蓝色是原始降落／通行区域。青色内仍可能因实时障碍或健康状态被拒绝 |
| `No bounded route...` | 查共享边界和起终点；当前日志尚不能区分预算耗尽/拓扑不可达/端点被侵蚀，不能断言电脑慢 |
| odom 偶发延迟 | 比较测量时间、输入回调、LIO/VIO CPU 与墙钟耗时、接收排队；检查 GPU 渲染和 `/dev/shm` |
| 启动器拒绝已有进程 | 正常结束上一轮及其 GUI；仅清理确认属于本轮的进程，不使用全局 `killall` |
| 消息导入或后缀错误 | 新终端只加载 `astra/env_117.bash`，检查消息子模块与构建产物 |
| 地图目录不存在 | 按快速启动重新准备；Git 不包含大型生成资源 |

启动要求 `/dev/shm` 至少 512 MiB 可用；不要盲删其他应用的共享内存。故障定位先取日志，再决定单因素修改。最新未解决项见 [项目上下文](PROJECT_CONTEXT.md)。
