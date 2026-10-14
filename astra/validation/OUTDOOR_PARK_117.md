# Baylands 室外公园（PX4 1.17 SITL）

## 启动

```bash
cd ~/dev/XT-Drone2-astra
bash astra/rviz_outdoor_117.bash
```

中文窗口显示 READY 后，G 点选航点或输入 XY 坐标，再点击开始任务。
默认地面等待不限时，最后一航点可以原地降落或等待追加任务。
已有室内入口 `bash astra/rviz_flight_117.bash` 不变。
一次只运行一个仿真。观察 Gazebo 可另开：

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

## 地图与开放区域

- 来源：[Open Robotics Baylands v3](https://app.gazebosim.org/OpenRobotics/fuel/models/baylands)，
  作者 Cole Biesemeyer，CC BY 4.0。
- 它是公共制作的公园仿真模型，不宣称是真实地点扫描，不是 FAST-LIVO2 自动重建。
- 本地保留 70 × 70 m 公园片区；视觉模型约 6 MB，碰撞几何保持裁切前表面的形状。
- 起飞点使用原模型停车区；对齐到平坦支持地面，范围内不验证复杂地形跟随。
- 初始检查范围是 22 × 22 m，其中起飞点连通的下降净空区域约 343.4 m²。
- 航点离起飞点最多 10 m；仍需保留 0.10 m 下降区域储备，避开实时膨胀障碍物。
- 巡航 0.90 m，初始航向校准爬升 1.60 m。每次最多 8 个航点。
- 橙色长箱是显式加入的测试障碍物，中心 `[3.5,0,1.1]`，尺寸 `[0.8,3,2.2]` m。
  `[0,0] → [7,0]` 直线路径穿过它，EGO 必须自行生成绕行，未提供中间绕行点。

绿色格来自静态碰撞几何的下降净空检查；高度范围 `[0.20,2.0]` m。
检查使用三角形 XY 包围盒与垂直距离的保守连续下界，至少 0.65 m；
以 0.10 m 网格检查后合并为 84 个矩形，合并不改变原区域，避免粗格误撤销
与数万个小格导致的在线检查开销。
运行时仍检查实际位置到三角形的 0.55 m 距离、边界、250 ms 新鲜度、
冻结高度偏置、航向和单一 setpoint 发布者。

完整公园背景和彩色显示地图不进入规划器。EGO 的输入仍是 FAST-LIVO2
传感器观测产生的局部障碍物。室外规划网格扩大到 32 × 32 × 4 m，
局部更新窗口保持 5 m，规划速度 0.20 m/s、加速度 0.25 m/s²。
室外观测到的离地障碍物使用 1.1 m 水平柱状膨胀，地面仍使用原 0.65 m 三维膨胀；
EGO 的 0.2 m 优化距离是膨胀地图外的软目标，不能单独当作硬安全保证。
静态下降净空检查不表示整片区域的定位、控制都已逐点实测通过。
不覆盖动态障碍、强风、地形起伏、旋翼外缘或实物飞行。

## 重新准备地图

2026-10-14 显示修复：生成器现在显式绑定 Ogre2 PBR 彩色纹理，并添加环境光和方向光。
此前世界没有灯光，SDF 纯色材质还覆盖了 COLLADA 纹理，导致 Gazebo/机载 RGB 灰暗。
已有生成目录不会自动更新；需生成新场景目录，不能只重启后假定材质已经改变。
此改动不修改碰撞几何或下降边界，但会改变视觉输入；旧飞行报告不构成新外观的重复验收。

资源和生成文件位于忽略的 `astra/runtime`，不上传大模型到 Git。
原包 150.7 MB，按 Fuel v3 和 SHA-256 固定；完成下载后校验再处理。
需已有 `map_tools_venv` 的 trimesh、fast-simplification、scipy、Pillow，以及：

```bash
astra/runtime/map_tools_venv/bin/python -m pip install pycollada==0.9.3 shapely==2.2.0
astra/runtime/map_tools_venv/bin/python astra/tools/prepare_baylands.py
```

准备程序拒绝覆盖已有场景。模型许可证和修改说明保存在场景的 `LICENSE.txt`。

## 自动路线审计

```bash
bash astra/run_117.bash obstacle baylands_route_check \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_park \
  --route-json astra/runtime/maps/baylands_park/route.json \
  --calibrate-ground-height --executor-audit --isolate-estimator-cpus
python3 astra/tools/audit_scan_flight.py astra/runtime/baylands_route_check
```

审计必须在仿真及 Gazebo GUI 结束后执行。路线为 `[7,0] → [7,4] → [0,4] → [0,0]`。
记录真值、FAST-LIVO2/PX4 测量时间、实际位移、规划曲线、距离、延迟和保护事件。
测试结果将记录在 `outdoor_park_117.json`，正常完成与故障退出分别标明。

开阔区域路线与阻挡路径绕障分别测试。已准备的开阔路线入口：

```bash
bash astra/run_117.bash obstacle baylands_open_check \
  --world-preset baylands --scan-dir astra/runtime/maps/baylands_park_open \
  --route-json astra/runtime/maps/baylands_park_open/route.json \
  --calibrate-ground-height --executor-audit
```

重新生成开阔路线场景可加 `--route-kind open --output astra/runtime/maps/baylands_park_open`。
它与阻挡场景的物理模型相同，只改变待验证路线及其预检查记录。

## 本轮结果（2026-10-13）

- 开阔路线 `[-7,0] → [-7,4] → [0,4] → [0,0]` 四航点均完成并安全降落；
  实际水平路径约 23.02 m，运行中不使用 Gazebo 真值生成 setpoint。
- 全程已知几何最小机体中心距离 3.055 m；四个航点的真值水平误差
  分别约 0.011、0.024、0.014、0.064 m。仅验证这条路线，未逐点验证整片公园。
- 地面冻结参考下，执行阶段 FAST-LIVO2 XY/Z p95 为 0.035/0.007 m，
  PX4 XY/Z p95 为 0.042/0.069 m。测量时间配对 p95 为 8 ms。
- 规划峰值速度/加速度约 0.198 m/s、0.054 m/s²，实际峰值速度约 0.196 m/s；
  setpoint 最大间隔 68.6 ms。未触发 armed failsafe，基线参数保持不变。
- 高度执行桥现在沿用已验证的地面冻结偏移，避免与监督器使用不同原点；
  不随飞行中漂移重新拟合，不放宽 0.20 m 门槛。
- 阻挡箱子的四航点路线在 `baylands_route_117_15` 和 `baylands_route_117_17`
  完整完成，离线全曲线距离、动态约束、实际位移和落地检查均通过。
  第 17 轮实际水平路径约 29.97 m，已知几何最小机体中心距离 1.172 m；
  规划峰值速度/加速度 0.200 m/s、0.243 m/s²，setpoint 最大间隔 86.7 ms。
- 先前重规划曾阻塞心跳 572 ms；已改为共享预算、异步请求和完整曲线检查，
  调时保留几何形状，修复重复重规划丢失速度与换方向失败。
- 第 16 轮仍因 252 ms 定位年龄受控降落，保留为失败证据。
  后续发现共享内存耗尽、图像到算法回调延迟 127.6 ms；备份并释放未使用缓存后，
  第 17 轮图像回调最大延迟 6.6 ms，未再次触发该保护。
  启动前现在要求共享内存至少有 512 MiB 可用；退出时给拥有的进程留出清理时间。
- 详细证据、命令和范围见 [绕障连续性记录](OUTDOOR_CONTINUITY_117.md)。
  当前不认证整片公园、动态障碍或长期可靠性。
- 后续完成箱子正反向、斜向五航点（41.08 m）与新增圆柱四航点（22.07 m），
  距离与动态审计通过；验证时发现并修复重试未切换备选的逻辑问题。
  见 [不同方向和障碍形状验证](OUTDOOR_GENERALIZATION_117.md)。
- 后续四轮扩大试验实际到达起飞点外 23.4 m，但全部中止后降落；
  暴露返程高度差及天然树旁空中规划与下降区域约束不一致的问题。
  已增加隔离扩大场景、枝叶保护柱高度设置与连续距离精查，
  完整大范围路线及天然树木绕行仍未通过，默认入口不扩大开放。
  证据、修改和命令见 [扩大范围验证](OUTDOOR_EXPANSION_117.md)。
