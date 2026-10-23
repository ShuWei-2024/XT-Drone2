# 仿真启动与操作

[返回仿真索引](README.md)

推荐入口：

```bash
cd ~/dev/XT-Drone2-astra
bash astra/rviz_outdoor_117.bash outdoor_$(date +%Y%m%d_%H%M%S) \
  --external-vision
```

另开终端显示 Gazebo：

```bash
GZ_PARTITION=astra_px4_117 gz sim -g
```

等待中文任务窗口显示 `READY` 后，在 RViz2 使用 **2D Goal Pose / G** 添加 XY 航点，再点击“开始任务”。按钮名称中的 2D 只表示目标选择平面，RViz2 视图本身是三维。完整地图、航点和返航说明见[仿真操作指南](../OPERATIONS.md)。
