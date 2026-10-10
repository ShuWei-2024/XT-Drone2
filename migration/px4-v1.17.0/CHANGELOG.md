# PX4 v1.17.0 迁移记录

- 原版本：v1.15.3，提交 `80171359656faf881dc643022f61ebde734d7cb6`。
- 目标版本：v1.17.0，提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`。
- 工作分支：`migration/px4-v1.17.0`。
- 操作时间：2026-10-10T00:18:48.193464+08:00（北京时间）。
- 完整本地备份：`/home/wwc/px4-migration-backups/20261010-001848-v1.15.3`。

## 记录规则

每个自定义文件修改保存修改前后 SHA-256、目的和统一差异补丁；验证结果另行记录。
上游版本切换涉及 9248 个路径，逐文件清单见 `upstream-files-v1.15.3-to-v1.17.0.tsv`。上游详细修改可通过 `git diff v1.15.3 v1.17.0 -- 文件路径` 查看。

## 已执行操作

1. 保存原工作区状态、子模块状态、VS Code 设置及原有修改补丁。
2. 将原来未跟踪的旧版本残留目录移动到备份中的 `untracked/`，清单见 `metadata.json`。
3. 创建原版本备份分支，切换到以官方 v1.17.0 为基础的迁移分支。

## 验证状态

版本已切换；子模块同步、编译及 ROS 2 通信验证待完成。

4. 备份已从 v1.17 删除的旧 libuavcan 子模块工作区，避免作为未跟踪源码误提交。

5. 将 v1.15 的 SITL 编译目录移动到备份；v1.17 使用新编译目录。

### 文件修改 001: `.vscode/settings.json`

恢复用户原有 Jazzy Python 搜索路径；将 ROS 发行版设为 Jazzy，并指定仿真虚拟环境解释器。

差异见 `file-changes/001-.vscode__settings.json.patch`。

### 文件修改 002: `Tools/setup/xtdrone2_env.sh`

提供可重复的 VS Code 终端环境初始化，加载 Jazzy、Python 虚拟环境及 ROS 内置的 Gazebo Harmonic 命令。

差异见 `file-changes/002-Tools__setup__xtdrone2_env.sh.patch`。

### 文件修改 003: `XTDrone2/xtd2_communication/xtd2_communication/px4_topics.py`

根据实际 px4_msgs MESSAGE_VERSION 生成 DDS 话题后缀；版本 0 保留原有名称。

差异见 `file-changes/003-XTDrone2__xtd2_communication__xtd2_communication__px4_topics.py.patch`。

### 文件修改 004: `XTDrone2/xtd2_communication/xtd2_communication/multirotor_communication.py`

将通信节点的六个 DDS 接口绑定到对应消息版本，修复 PX4 v1.17 本地位置订阅及姿态设定点发布的话题名称。

差异见 `file-changes/004-XTDrone2__xtd2_communication__xtd2_communication__multirotor_communication.py.patch`。

### 文件修改 005: `XTDrone2/xtd2_test/xtd2_test/communication_test/accel_test.py`

同步加速度示例中的本地位置订阅，避免在 v1.17 上因仍订阅旧话题而无法获取位置。

差异见 `file-changes/005-XTDrone2__xtd2_test__xtd2_test__communication_test__accel_test.py.patch`。

### 文件修改 006: `XTDrone2/xtd2_test/package.xml`

声明示例节点实际使用的消息包和通信辅助模块，确保 colcon 构建与运行依赖顺序正确。

差异见 `file-changes/006-XTDrone2__xtd2_test__package.xml.patch`。

### 文件修改 007: `XTDrone2/.gitmodules`

记录 px4_msgs 对应的 release/1.17 分支；实际提交另由 Git 子模块指针固定。

差异见 `file-changes/007-XTDrone2__.gitmodules.patch`。

### 子模块修改: `XTDrone2/xtd2_third_party_pkgs/px4_msgs`

`18cbccedb6c304b180bca8e81a4a601758fc9de9` → `86d8239e962f6939e05c3737784f60c02fa884db`。六种通信消息定义逐字节验证与 PX4 v1.17.0 一致。

### 文件修改 008: `Tools/setup/xtdrone2_agent.sh`

单独构建与默认 PX4 XRCE v2 客户端兼容的 Agent v2.4.3，复用 Jazzy 的 DDS 依赖，安装到用户目录并保留原 Agent v3 源码。

差异见 `file-changes/008-Tools__setup__xtdrone2_agent.sh.patch`。

### 文件修改 009: `Tools/setup/xtdrone2_env.sh`

将本项目隔离安装的 Agent v2.4.3 加入终端 PATH，供原 XTDrone2 启动文件调用。

差异见 `file-changes/009-Tools__setup__xtdrone2_env.sh.patch`。

6. 备份 libevents 切换后已不再引用的旧 nlohmann_json 工作区，避免误作为源码提交。

### 文件修改 010: `PX4-Autopilot/Tools/setup/xtdrone2_agent.sh`

关闭未使用的 Agent P2P 配置，避免要求额外安装 XRCE Client 开发包；默认 PX4 UDP 通信不依赖此功能。

差异见 `file-changes/010-PX4-Autopilot__Tools__setup__xtdrone2_agent.sh.patch`。

### 文件修改 011: `PX4-Autopilot/Tools/setup/xtdrone2_agent.sh`

系统 spdlog 1.12/fmt 9 不兼容 Agent 2.4.3 的格式化代码；使用官方 SuperBuild 构建随版本匹配的 spdlog 1.9.2，同时复用 Jazzy 的 FastDDS/FastCDR。

差异见 `file-changes/011-PX4-Autopilot__Tools__setup__xtdrone2_agent.sh.patch`。

### 文件修改 012: `XTDrone2/requirements-px4-v1.17.txt`

虚拟环境无法读取系统 transforms3d；固定并记录通信节点启动所需的附加 Python 依赖。

差异见 `file-changes/012-XTDrone2__requirements-px4-v1.17.txt.patch`。

### 文件修改 013: `PX4-Autopilot/Tools/setup/xtdrone2_smoke_test.py`

添加真实端到端验证：启动无界面 Gazebo、PX4 和 Agent，检查 XTDrone2 接收有效位置数据，并清理本次启动的进程。

差异见 `file-changes/013-PX4-Autopilot__Tools__setup__xtdrone2_smoke_test.py.patch`。

### 文件修改 014: `PX4-Autopilot/Tools/setup/xtdrone2_smoke_test.py`

保持仿真进程输入管道打开，避免非交互 SSH 中 PX4 命令行反复读取 EOF 产生大量提示符日志。

差异见 `file-changes/014-PX4-Autopilot__Tools__setup__xtdrone2_smoke_test.py.patch`。

### 文件修改 015: `XTDrone2/xtd2_gz_sim/models/x500_base/model.sdf`

从官方 PX4 v1.17.0 对应 Gazebo 模型补入 X500 磁力计，修复旧模型启动后无罗盘、水平位置无效。

差异见 `file-changes/015-XTDrone2__xtd2_gz_sim__models__x500_base__model.sdf.patch`。

### 文件修改 016: `XTDrone2/xtd2_gz_sim/worlds/aruco.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/016-XTDrone2__xtd2_gz_sim__worlds__aruco.sdf.patch`。

### 文件修改 017: `XTDrone2/xtd2_gz_sim/worlds/default.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/017-XTDrone2__xtd2_gz_sim__worlds__default.sdf.patch`。

### 文件修改 018: `XTDrone2/xtd2_gz_sim/worlds/indoor1.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/018-XTDrone2__xtd2_gz_sim__worlds__indoor1.sdf.patch`。

### 文件修改 019: `XTDrone2/xtd2_gz_sim/worlds/lawn.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/019-XTDrone2__xtd2_gz_sim__worlds__lawn.sdf.patch`。

### 文件修改 020: `XTDrone2/xtd2_gz_sim/worlds/rover.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/020-XTDrone2__xtd2_gz_sim__worlds__rover.sdf.patch`。

### 文件修改 021: `XTDrone2/xtd2_gz_sim/worlds/walls.sdf`

为世界启用 Gazebo 磁力计系统，使 PX4 1.17 的 X500 磁传感器产生数据。

差异见 `file-changes/021-XTDrone2__xtd2_gz_sim__worlds__walls.sdf.patch`。

### 文件修改 022: `PX4-Autopilot/Tools/setup/xtdrone2_smoke_test.py`

扩展端到端验证，支持实际 XTDrone2 世界与现有车辆启动文件，检查旧仿真资源迁移后的位置状态。

差异见 `file-changes/022-PX4-Autopilot__Tools__setup__xtdrone2_smoke_test.py.patch`。

### 文件修改 023: `XTDrone2/xtd2_gz_sim/worlds/baylands.sdf`

为使用单引号属性的世界同步启用磁力计系统，覆盖 Windy 和 Baylands 的 PX4 1.17 仿真配置。

差异见 `file-changes/023-XTDrone2__xtd2_gz_sim__worlds__baylands.sdf.patch`。

### 文件修改 024: `XTDrone2/xtd2_gz_sim/worlds/windy.sdf`

为使用单引号属性的世界同步启用磁力计系统，覆盖 Windy 和 Baylands 的 PX4 1.17 仿真配置。

差异见 `file-changes/024-XTDrone2__xtd2_gz_sim__worlds__windy.sdf.patch`。

### 文件修改 025: `XTDrone2/docs/PX4_V1_17.md`

补充固定版本、真实启动命令、重建步骤、验证范围、备份与两仓库提交说明。

差异见 `file-changes/025-XTDrone2__docs__PX4_V1_17.md.patch`。

### 文件修改 026: `XTDrone2/README.md`

在仓库首页提供 PX4 1.17 迁移说明入口，保留原 XTDrone2 README 内容。

差异见 `file-changes/026-XTDrone2__README.md.patch`。

### 文件修改 027: `PX4-Autopilot/migration/px4-v1.17.0/README.md`

整理逐文件记录索引、构建与端到端验证证据及提交状态，便于用户审查和一并上传。

差异见 `file-changes/027-PX4-Autopilot__migration__px4-v1.17.0__README.md.patch`。

### 文件修改 028: `PX4-Autopilot/migration/px4-v1.17.0/README.md`

将迁移说明链接指向记录包内的最终代码副本，使 Windows 导出和 GitHub 浏览都可访问。

差异见 `file-changes/028-PX4-Autopilot__migration__px4-v1.17.0__README.md.patch`。

### 文件修改 029: `PX4-Autopilot/migration/px4-v1.17.0/record_change.py`

将逐文件记录工具随源码保存，支持后续修改自动产生差异、原因、时间与哈希记录。

差异见 `file-changes/029-PX4-Autopilot__migration__px4-v1.17.0__record_change.py.patch`。

### 文件修改 030: `PX4-Autopilot/migration/px4-v1.17.0/README.md`

在记录索引中说明后续可复用的逐文件记录工具。

差异见 `file-changes/030-PX4-Autopilot__migration__px4-v1.17.0__README.md.patch`。

### 文件修改 031: `PX4-Autopilot/migration/px4-v1.17.0/COLCON_IGNORE`

迁移记录包含代码快照，添加构建忽略标记，防止将快照内的 package.xml 识别为正式 ROS 包。

差异见 `file-changes/031-PX4-Autopilot__migration__px4-v1.17.0__COLCON_IGNORE.patch`。

### 文件修改 032: `XTDrone2/migration/px4-v1.17.0/COLCON_IGNORE`

修复复制迁移记录后出现重复 xtd2_test 包的问题：仅将记录目录排除出 colcon 包发现，保留正式源码和全部审计记录。

差异见 `file-changes/032-XTDrone2__migration__px4-v1.17.0__COLCON_IGNORE.patch`。

### 外部变化补记 033: `PX4-Autopilot/.vscode/settings.json`

检测到上次记录后新增 ROS2.distro=jazzy 编辑器设置；补记该变化，保留当前文件内容。

差异见 `file-changes/033-PX4-Autopilot__observed-settings.json.patch`。
