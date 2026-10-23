# 仿真规划与执行

[返回仿真索引](README.md)

仿真任务链路为：

```text
RViz2 航点
  -> 任务监督器
  -> EGO-Planner B-spline
  -> 速度/加速度/障碍距离检查
  -> 执行桥
  -> PX4 trajectory_setpoint
```

规划器使用实时局部障碍物地图；背景模型和预先生成的安全边界不能代替传感器观测。地图 readiness、odom 新鲜度、轨迹心跳或 TF 失效时必须撤销轨迹。仿真故障注入和多航点操作见[仿真操作指南](../OPERATIONS.md)。
