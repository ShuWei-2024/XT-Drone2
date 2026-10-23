# 真实传感器驱动

[返回实物索引](README.md)

先启动驱动，不启动解锁和规划。确认实际话题后再写入项目启动文件：

```text
Mid-360 -> PointCloud2
D435i   -> RGB / Depth / IMU
飞控    -> PX4 状态与时间消息
```

检查内容：

- 话题名称、消息类型和 QoS；
- 测量时间戳是否来自传感器测量时刻；
- 点云是否带有效坐标和时间字段；
- RGB、Depth、IMU 的帧率和时间关系；
- 网络/USB 队列是否持续增长；
- 传感器固定后 TF 是否稳定。

驱动成功启动不等于 FAST-LIVO2 输入正确。必须保存 `ros2 topic hz`、`ros2 topic echo`、TF 和输入审计结果。
