# Astra · ROS 2 自主飞行仿真

基于 ShuWei-2024/XT-Drone2，面向无人机定位、建图、规划与闭环飞行验证。开发分支：`astra/px4-1.17`。

环境：Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · PX4 v1.17.0。

```text
Mid-360 / RGB / IMU → FAST-LIVO2 → 局部地图 → EGO-Planner → PX4 Offboard
```

FAST-LIVO2 可通过 External Vision 向 PX4 提供 XY/Z 位置观测。

## 启动使用

已完成依赖安装、构建和地图准备后，在新终端执行：

```bash
cd ~/dev/XT-Drone2-astra
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --external-vision
```

自动启动完整链路，打开 **RViz2 和中文航点窗口**。本机已复测的彩色地图目录为 `astra/runtime/maps/baylands_park_coloured`，使用时在命令末尾追加 `--scan-dir astra/runtime/maps/baylands_park_coloured`。新生成的默认地图已含颜色修复。

另开终端，打开 **Gazebo 窗口**：

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

等任务窗口显示 **READY**，在 RViz2 按 **G** 点选航点，选择末点降落或悬停，再点击 **开始任务**。支持多个航点、空中追加和暂停/继续；启动后不会自动起飞。结束时点击降落，等待落地上锁和进程清理。一次只运行一套仿真。

室内 Replica office4 已准备好时，可改用 `bash astra/rviz_flight_117.bash`。

## 构建与测试

首次部署须先安装依赖、准备 PX4/DDS Agent 和地图，步骤见下方文档索引中的安装教程。构建命令在仓库根目录执行：

```bash
bash astra/build_px4_117.bash
bash astra/build_117.bash
```

在新终端加载环境，运行回归与 60 秒静止检查：

```bash
cd ~/dev/XT-Drone2-astra
source astra/env_117.bash
python3 -m pytest -q astra/validation \
  astra/packages/astra_px4_offboard/test astra/packages/astra_fast_livo2/test
ctest --test-dir build_117/astra_fast_livo2_core --output-on-failure
bash astra/run_117.bash static static_$(date +%Y%m%d_%H%M%S) --seconds 60
```

静止检查不解锁；回归测试不能替代闭环飞行验收。地图、日志和构建产物在本机生成，不随 Git 下载。

## 当前状态

室内指定路线、室外箱体/圆柱绕障及部分长路线已完成闭环验证。彩色场景复测通过，暂不需要因颜色修复重调 FAST-LIVO2。

重复稳定性仍未通过：扩大区域有偶发规划失败，树旁存在高度漂移和长绕行问题。跨地图、宽范围长期运行和真实硬件尚未认证。具体指标及失败记录保留在验证报告中。

## 文档索引

**[进入文档索引](docs/README.md)**：安装教程、操作说明、模块架构、开发规范、验证报告与人员/AI 接手上下文。

## 许可

框架保留 MIT 许可；EGO-Planner 使用 GPL-3.0，其余依赖与地图遵循各自许可。来源和许可明细见开发文档，不能将整个集成统一视为 MIT。
