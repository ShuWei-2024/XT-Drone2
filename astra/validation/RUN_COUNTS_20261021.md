# 测试次数核对（2026-10-21）

统计时点：2026-10-21 第 6 轮长时运动复测后；依据各运行目录的 `run_summary.json`、`runtime.json`、`motion_events.json` 和执行源码 manifest。

“任务完成”要求 `flight_complete=true`、`fault` 为空、退出码为 0，并核对终态 `disarmed`。此处只统计任务流程，不代表精度、全图适用性或实物验收全部通过。故障注入应按预期恢复判定，不混入正常飞行完成率。

## 最近相关测试

| 分组 | 次数 | 任务完成 | 任务未完成 |
| --- | ---: | ---: | ---: |
| 室内矩形锁高、默认参数组（源码阶段有迭代） | 4 | 3 | 1 |
| 室内锁高诊断：低气压噪声、关闭 GNSS 速度融合、输出预测器时间常数 | 5 | 5 | 0 |
| 先前同配置室外长路线：约 71 m、返航和 300 s 保持 | 3 | 3 | 0 |
| 延长连续运动试验（目标 1500 s） | 1 | 0 | 1 |

### Baylands 固定 129 项配置收尾复测

`acceptance_baylands_20261021_01/02/03/04/05/06` 的 129 项执行源码 manifest 逐项相同，使用同一路线、同一 PX4 参数覆盖、同一估计器 YAML 和同一事件执行器。第 4 轮在解锁前因 EV/全局原点初始化位置变化超过 `0.03 m` 退出；第 6 轮是单独的 1500 s 长时运动目标，不与正常短路线完成次数混算。

| 运行 | 分类 | 结果 |
| --- | --- | --- |
| `acceptance_baylands_20261021_01` | 正常飞行 | 约 71.321 m，三个航点、返航、300 s 保持、降落上锁 |
| `acceptance_baylands_20261021_02` | 正常飞行 | 约 71.433 m，三个航点、返航、300 s 保持、降落上锁 |
| `acceptance_baylands_20261021_03` | 受保护中止 | 第二长航段高度差超过 `0.20 m`，请求受控降落并上锁；不是 PX4 failsafe |
| `acceptance_baylands_20261021_04` | 启动门控 | 未解锁；原点初始化位置变化超过 `0.03 m`，不计为飞行失败 |
| `acceptance_baylands_20261021_05` | 正常飞行 | 约 71.229 m，三个航点、返航、300 s 保持、降落上锁 |
| `acceptance_baylands_20261021_06` | 长时运动受保护中止 | 1500 s 目标；实际运动 362.886 s、55.925 m，2/3 航点；Odom 测量年龄 268 ms，受控降落上锁，无 armed failsafe |

按短路线真正进入飞行的 4 轮统计：3 轮正常完成、1 轮按既有高度保护中止；按短路线 5 次启动尝试统计：3 轮完成、1 轮受保护中止、1 轮未解锁。第 6 轮另计为长时运动未完成、保护动作完成。这个分组支持指定 Baylands 短路线的重复流程结论，但不证明 Odom 长时延迟、高度漂移根因、无限内存稳定或跨地图通用性。详细指标见[Baylands 固定配置收尾复测](BAYLANDS_ACCEPTANCE_20261021.md)和[Baylands 长时运动中止审计](BAYLANDS_LONG_MOTION_20261021.md)。

不同组不合并计算当前配置成功率。此前“最近五轮”统计包含不同源码阶段的室内诊断轮；当前固定配置按上面的 Baylands 分组单独统计，不与 `normal_long_acceptance_20261020_*` 合并。

### 室内默认参数组明细

| 运行目录 | 结果 | 版本说明 |
| --- | --- | --- |
| `horizontal_rectangle_lockz_20261021_01` | 完成并降落上锁 | 较早源码阶段 |
| `height_lockz_default_repeat_20261021_01` | 数据过期中止，随后降落上锁 | 后续补充执行桥诊断记录，不能宣称故障根因已修复 |
| `height_lockz_default_repeat_20261021_02` | 完成并降落上锁 | `execution_sources_unchanged=false`，不作为干净固定版本重复验收 |
| `height_lockz_default_repeat_20261021_03` | 完成并降落上锁 | 最新默认飞行基线的一次干净记录 |

四轮 PX4 覆盖参数和估计器 YAML 相同，但执行源码 manifest 不同。严格按 `_03` 完整源码 manifest、默认参数和相同路线统计，本组仅有 1 次干净成功记录；不能把 3/4 当作最新固定版本的可靠性。

### 室内诊断明细

五轮全部完成并上锁：

- `height_lockz_lownoise_20261021_01`
- `height_lockz_nognssvel_20261021_01`
- `height_lockz_nognssvel_repeat_20261021_01`
- `height_lockz_taupos010_20261021_01`
- `height_lockz_tauvel010_20261021_01`

### 室外明细

`normal_long_acceptance_20261020_01/02/03` 的执行源码 manifest 相同，三轮均完成，见 `NORMAL_LONG_ACCEPTANCE_117.md`。

`normal_30min_motion_20261020_01` 配置为 1500 秒连续运动目标，未完成；故障为 `PX4/LIVO height drift exceeds frozen ground offset by 0.20 m`，最终事件为 `native_rtl_disarmed`。文件名含 30min 不代表已经飞满 30 分钟。本轮源码和试验条件不同，不合并进前述三轮成功率。

## 历史记录范围

当前 `astra/runtime` 下共 262 份 `run_summary.json`：10 份静态测试，23 份带故障注入记录，其余 229 份涵盖不同版本的飞行、诊断与启动尝试。早期运行未必使用统一的故障注入标记，因此这只是记录清点，不能把 262 当成当前版本的正常飞行次数，也不能把历史退出码直接作为飞行成功率。
