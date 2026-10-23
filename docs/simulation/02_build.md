# 仿真构建与回归

[返回仿真索引](README.md)

```bash
cd ~/dev/XT-Drone2-astra
bash astra/build_px4_117.bash
bash astra/build_117.bash
source astra/env_117.bash
python3 -m pytest -q astra/validation \
  astra/packages/astra_px4_offboard/test \
  astra/packages/astra_fast_livo2/test
ctest --test-dir build_117/astra_fast_livo2_core --output-on-failure
```

构建后先运行不解锁的静止检查，再运行受控 SITL 路线。测试结果写入 `astra/runtime/<run>/`；不要把旧运行目录复制到新机器来绕过地图和依赖检查。
