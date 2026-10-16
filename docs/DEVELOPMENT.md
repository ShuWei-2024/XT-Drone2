# 开发与维护

[返回文档索引](README.md)

## 目录约定

| 目录 | 职责 |
| --- | --- |
| `astra/packages/` | ROS 包、算法、launch、模块级测试 |
| `astra/tools/` | SITL 监督、场景准备、运行记录与离线审计 |
| `astra/config/` | RViz、路线、依赖与补丁配置 |
| `astra/validation/` | 小型可审查报告、汇总 JSON、回归测试 |
| `astra/runtime/` | 本机模型、下载缓存、原始日志、录屏，Git 忽略 |
| `docs/` | 当前使用、设计、维护与接手文档 |
| `xtd2_*/` | XT-Drone2 框架包 |

## 构建与回归

新终端、仓库根目录执行：

```bash
bash astra/build_117.bash
source astra/env_117.bash
python3 -m pytest -q astra/validation \
  astra/packages/astra_px4_offboard/test astra/packages/astra_fast_livo2/test
ctest --test-dir build_117/astra_fast_livo2_core --output-on-failure
git diff --check
```

可选的完整优化器尾段障碍回归默认不构建，需先启用再运行：

```bash
colcon --log-base log_117 build --build-base build_117 --install-base install_117 \
  --packages-select bspline_opt --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DASTRA_BUILD_PLANNER_REGRESSION=ON
ctest --test-dir build_117/bspline_opt --output-on-failure -R '^tail_obstacle$'
```

Python 回归中包含独立编译的 C++ 搜索/曲线检查；需要 Eigen 和 g++。这些检查不能替代实际闭环飞行。修改算法应重建后在新运行目录复测；修改文档无需重复解锁飞行。

不要把上游所有 lint/相机测试当作当前飞行回归集：部分上游测试有既存问题；扩大范围需明确目的并报告结果。当前入口与实际运行验证见 [验证索引](../astra/validation/README.md)。

## 版本与运行隔离

依赖以 [dependencies.lock.json](../astra/dependencies.lock.json) 为准；更新 PX4 或消息时同时验证消息定义、版本后缀、命名空间和坐标接口。ROS 构建不混用其他 overlay；不对父仓库和子模块任意执行全量更新。

SITL 使用 ROS domain 94、Gazebo partition `astra_px4_117`、`x500_0` namespace。每轮独立 rootfs/参数配置，运行器保留基线并清理所属进程。不要在飞行中修改执行源码或二进制，也不要通过多发一套 setpoint 绕过控制交接。

## 修改与证据要求

1. 写清待验证问题，先阅读原始日志；区分事实、线索和假设。
2. 单因素修改，保留输入/配置和成功、失败两类运行证据。
3. 评估测量时间配对、实际轨迹、完整曲线、终点误差、恢复行为与内存。
4. 更新相关使用说明；报告写入 `astra/validation/`，在索引增加入口。
5. 更新 AI 上下文的状态与下一步，避免新接手者重复过时诊断。

禁止用飞行中重新拟合原点抹去漂移；禁止把只完成任务改写为精度认证。保护阈值改变须有理由、对照与审计，不能作为让失败运行“通过”的快捷方式。

## Git 工作流

开发分支为 `astra/px4-1.17`，`origin` 为老师团队仓库。检查后提交具体文件：

```bash
git status --short
git diff
git add <本次代码和文档>
git commit -m 'Describe the change and validation'
git push origin astra/px4-1.17
```

推送仅更新此分支；合入团队主分支需要单独 PR 和审核。不要强制推送。子模块版本改变需提交父仓库指针；构建产物、资产、录屏和完整日志不提交。

## 许可与来源

框架保留 [MIT 许可](../LICENSE) 和 [上游介绍](README.upstream.md)。EGO-Planner 的 [GPL-3.0 许可](../astra/packages/ego_planner_jazzy/LICENSE) 及 [来源说明](../astra/packages/ego_planner_jazzy/UPSTREAM.md) 保留；FAST-LIVO2、vikit、Sophus 保留各自来源与许可。整个集成不能概括为所有模块都使用 MIT。

Baylands 数据遵循 CC BY 4.0，Replica 使用研究条款。准备程序生成许可与修改说明，资源不随本仓库分发。发布资源或迁移实物前另行核对许可和适用范围。
