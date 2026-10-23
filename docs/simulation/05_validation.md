# 仿真验收与证据

[返回仿真索引](README.md)

验收报告集中在 [`astra/validation`](../../astra/validation/README.md)。每一轮至少保留：

- `runtime.json`：版本、参数、进程和运行配置；
- `preflight_health.json`：起飞前健康门控；
- `frame_samples.csv`：Gazebo、FAST-LIVO2 和 PX4 的时间对齐；
- `motion_events.json`：航点、交接、保护和降落事件；
- ULog、节点日志和离线审计 JSON/图表。

判定时同时看任务是否完成、实际位移、航点误差、障碍距离、动态约束、odom 年龄、保护响应和落地状态。单次成功不能覆盖长时间、跨地图或实物结论。
