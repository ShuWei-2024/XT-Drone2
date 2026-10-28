# 进程、线程与 ROS 2 职责

[返回架构索引](README.md) · [Offboard 接口](OFFBOARD.md)

本文按当前源码启动关系盘点。没有为盘点启动仿真；线程总数没有实时测量，不能用下面的应用线程结构充当 `ps` 的总线程数。

## 核心业务进程

当前 `obstacle` 模式的核心应用是 **5 个进程，启用 External Vision 后为 6 个**，不含 Agent、仿真器、TF、界面、审计与启动器。它们是实物迁移的功能参考，当前实现仍含 SITL 专用部分。

| 进程 | ROS 节点/功能 | 应用线程结构（不含中间件和第三方后台线程） |
| --- | --- | --- |
| 输入适配器 | `FastLivo2Adapter` | 主线程驱动执行器；最多 4 个回调工作线程；启用 CSV 审计时再有 1 个写盘线程 |
| FAST-LIVO2 | `fastlivo_mapping` / `laserMapping` | 主线程运行 `mapper.run()`；1 个订阅回调线程；计算阶段可使用 OpenMP |
| 局部地图 | `LocalMappingGuard` | 主线程执行 ROS 回调；最多 1 个地图构建工作线程 |
| EGO-Planner | `ego_planner_node` | 默认单线程 ROS 执行器；规划、碰撞检查和心跳回调共用执行路径 |
| 飞行任务 | 监督器 + `Preview` + `GuardedBsplineBridge`；交互时含航点接收节点 | 三者共用进程；主执行路径先处理回调，再顺序 `preview.tick()`、`bridge.tick()`；执行器/底层可有后台线程 |
| External Vision（可选） | `external_vision_sitl.py` | 默认单线程 ROS 执行器，另有中间件后台线程 |

FAST-LIVO2 编译配置将 OpenMP team size 设为最多 4，具体取决于 CPU 核数、OpenMP 支持及运行时；4 包含参与并行的调用线程，不能再简单算成额外 4 个。线程池也可能按需创建。DDS、Gazebo transport、OpenCV 等库会增加后台线程。

## 完整交互仿真的其他执行单元

按“室外地图 + obstacle + EV + RViz 交互、故障注入关闭、已进入任务阶段”盘点，以下 **25 个长期功能执行单元**是静态清单，不是当前系统 PID 实测总数：

| 分类 | 数量 | 内容 |
| --- | --- | --- |
| 核心业务 | 6 | 上表 5 项 + EV |
| 仿真与通信基础设施 | 12 | Gazebo server、PX4 SITL、DDS Agent、传感器 bridge、时钟 bridge、6 个静态 TF 发布器、GCS 心跳 |
| 监测与审计 | 4 | 收件/资源监视、frame auditor、持续 health probe、scan probe |
| 交互显示 | 3 | RViz2、场景显示节点、中文任务窗口 |

每个静态 TF 发布器当前由 launch 单独启动，故计 6 个；多个定时器本身不会各自创建一个进程或一个工作线程。Gazebo 插件运行在 server 内，不按传感器数量另计进程。

实际操作系统进程数还会包括运行器、`ros2 launch/run` 包装、PX4 启动程序、Gazebo 启动包装等；另开 Gazebo GUI、ROS CLI daemon、终端也增加进程。原点初始化和起飞前健康检查是阶段性进程；故障注入会增加 relay/injector。因此不能只报一个固定的全系统 PID 总数。

实物不需要 Gazebo server、Gazebo bridge、SITL PX4 进程和仿真真值审计，但会新增真实传感器驱动。真实飞控运行自己的固件任务，不算作 RK3588 上的 Linux 进程。

## 如何查看真实数量

仿真或实物链路已经启动时，可以只读检查：

```bash
ps -eo pid,ppid,nlwp,comm,args --forest
# 用上面确认的某个 PID 替换 12345：
ps -T -p 12345 -o pid,tid,comm
ros2 node list
ros2 node info /laserMapping
ros2 topic info /fast_livo2/odom --verbose
```

`NLWP` 是该进程的内核线程数，包含第三方后台线程。`ros2 node list` 列的是逻辑节点，不是进程；同时用进程树、运行器日志和节点图判断归属，不能用整机 `ps` 总数当项目进程数。

## ROS 2 在项目中承担什么

| 能力 | 项目里的实际作用 |
| --- | --- |
| 节点和消息类型 | 把定位、地图、规划和执行拆成有输入输出契约的功能单元；C++ 用 rclcpp，Python 用 rclpy |
| Executor / callback group | 调度订阅、定时器等回调；适配器多线程、规划器单线程、任务节点组合执行 |
| 参数和 launch | 加载 YAML、设备话题、保护阈值、命名空间和 remap，组合启动多个进程 |
| 时间 | 提供 header 时间、ROS `/clock` 与 `use_sim_time`；steady clock 用于不依赖仿真时间的存活检查 |
| QoS | 定义可靠/尽力、队列深度和持久性；readiness 使用 transient-local，晚加入者能获得最近状态 |
| TF2 | 发布传感器安装关系，供订阅者查询坐标关系；不自动替代 FAST-LIVO2 算法内部外参 |
| 调试和显示 | 日志、诊断、`ros2 node/topic` 和 RViz2 图层 |

ROS 2 不是操作系统，也不是 SLAM 或飞控。它提供组织和运行机器人软件的框架；算法和保护策略仍由项目代码实现。QoS 和时间 API 不自动消除延迟，也不保证硬实时。

## 源码入口

- [进程启动与角色](../../astra/tools/run_sitl_117.py)
- [Gazebo、PX4 和 6 个静态 TF](../../xtd2_launch/launch/astra_sitl.launch.py)
- [适配器执行器与写盘线程](../../astra/packages/astra_fast_livo2/scripts/astra_fast_livo2_adapter.py)
- [LIVO 主循环和接收线程](../../astra/packages/astra_fast_livo2_core/src/main.cpp)
- [局部地图工作线程](../../astra/packages/astra_fast_livo2/scripts/local_mapping_guard.py)
- [EGO 单线程入口](../../astra/packages/ego_planner_jazzy/plan_manage/src/ego_planner_node.cpp)
- [监督器与 preview/bridge 组合](../../astra/tools/ego_sitl_flight.py)
