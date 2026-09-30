# Sharpa Teleop

基于 **Pixi + ROS 2 Jazzy** 的 Manus 手套遥操作项目。基于官方 Sharpa 运动学模型与目标函数，将双手关键点转换为 SharpaWave 关节目标；同一套生产者可接入 **MuJoCo 仿真** 或 **真实机械手安全输出**。

> 标准拓扑固定为两个终端：终端 1 运行唯一生产者 `pixi run manus`，终端 2 选择**一个**消费者：`pixi run sim` 或 `pixi run real [left|right]`。`real` 默认双手，直接消费重定向目标；它不启动 MuJoCo。软件保护不能替代实体急停。

## 数据链路

```text
终端 1：pixi run manus（唯一生产者组）
Manus 手套
  │ 本项目原生 C++ `manus_ros`（直接 Manus SDK）
  ▼
/manus/{left,right}/raw_poses（原始采集频率，每手 25 个关键点）
  │ retarget（直接订阅，官方模型、目标函数与原版 IPOPT）
  ▼
/sharpa/{left,right}/command（22 个关节，rad；保留输入时间戳）
  ├─────────────────────────────────────────────────────┐
  │                                                     │
  ▼                                                     ▼
终端 2：pixi run sim                         终端 2：pixi run real [left|right]
mujoco_sim                                  sharpa_output
  │ 500 Hz 指数平滑 → 位置执行器             │ 500 Hz 指数平滑 → 原生 SDK
  ▼                                           ▼
MuJoCo 反馈                                  已选定的真实机械手
```

`manus.launch.py` 只启动项目本地 `manus_ros` 和 `retarget`。Manus 每次真实采集回调直接发布原始位姿，不再经过 250 Hz 重采样或 40 ms 插值缓冲；重定向直接消费原始话题。项目运行路径中没有中间网络帧转发或伪终端。生产者组内任一已启动进程退出，launch 会关闭该组。`sim` 和 `real` 是独立的最新目标消费者，不启动输入或重定向，也不串联。

仿真只发布 `/sim/sharpa/{left,right}/joint_states` 反馈；真实输出直接订阅 `/sharpa/{left,right}/command`。不要在同一 ROS 域把 `sim` 和 `real` 当作同一条链路的两个阶段运行。

## 环境与外部依赖

- Linux x86-64；当前验证环境为 Ubuntu 24.04。
- 已安装 [Pixi](https://pixi.sh/)。ROS 2 Jazzy 和 `manus_ros` 所需的 C++ 工具链通过默认 Pixi 环境提供，不要求系统预装 ROS。
- `native/manus_ros` 是本项目的本地 C++ 适配器：它直接调用 Manus SDK 并发布 ROS 2 `PoseArray`，不需要客户端专用 Pixi 环境、终端伪设备或网络帧转发。
- [Sharpa Manus SDK](https://github.com/sharpa-robotics/sharpa-manus-sdk) 保持在项目外部：其 `client/ManusSDK` 的授权头文件和库只在构建/运行时被包含和链接，操作者标定文件也从外部 `client` 目录（或 `calibration_dir`）读取。本项目不会复制、暂存或重新授权这些 SDK/标定资源。
- 真机另需与设备固件兼容的 **Sharpa Wave SDK**。默认路径为 `/opt/sharpa-wave-sdk`；可通过 `SHARPA_WAVE_SDK` 或 `native_sdk_root` 指定。项目不会自动回退到 Manus 仓库附带的旧硬件 SDK。
- 关节遥操作连接使用 `SharpaWaveConfig.disable_sync_time=true` 和 `disable_tactile=true`，关闭 SDK 启动时的 HTTPS 设备校时和触觉初始化；保留关节限位、格式校验、SDK 故障停用及健康退出回零。
- 仿真和真机统一使用“最新目标 + 500 Hz 指数平滑”：`q_next = q_prev + (1 - exp(-dt / tau)) * (target - q_prev)`，默认 `tau = smoothing_time_sec = 0.02` 秒，`dt` 为实际经过的单调时间。新目标立即替换旧目标，不回放历史轨迹；两端使用同一函数，不限制固定速度、步长、加速度或 jerk。20 ms 时间常数约需 60 ms 接近目标变化量的 95%；大角度变化会产生较大速度，不能当作安全限速。
- **已删除 1 rad/s 限速和 500 ms 输入 Watchdog。** 输入断流后，仿真与真机会继续接近最后一个有效目标并持续保持；不会因消息年龄停用，旧但首次收到且格式有效的递增时间戳目标也可接受。不要依赖停止手套输入来停止机械手，必须显式停用、退出输出进程或使用实体急停。500 Hz 是主机侧循环目标，不是硬件伺服、网络发包频率或硬实时保证。
- 输入目标仍按 URDF 上下限裁剪；NaN/Inf、名称或数量错误、未来及乱序时间戳仍拒收。实测反馈和最终 SDK 下发边界严格检查限位，SDK 故障仍触发停用。`rawmanus` 的过期标签仅用于显示，不会控制机械手。
- `sim` 另需 Sharpa 模型仓库中的 `wave_01` URDF、MuJoCo XML 和网格。`real` 的输出消费者在运行时**不需要**模型仓库或 MuJoCo。
- 真手套采集需要有效的 **Manus SDK-component 许可证**、已连接的手套及操作者标定文件。
- 只有 MuJoCo 窗口需要图形显示环境；无显示器时给 `sim` 传入 `headless:=true`。

默认目录布局：

```text
sharpa/
├── sharpa_teleop/          # 本项目
├── sharpa-manus-sdk/       # 官方 SDK
└── sharpa-urdf-usd-xml/    # 官方模型（仅仿真需要）
```

`pixi.toml` 使用项目相对路径设置：

| 环境变量 | 默认值 | 用途 |
| --- | --- | --- |
| `SHARPA_MANUS_SDK` | `$PIXI_PROJECT_ROOT/../sharpa-manus-sdk` | Manus 协议、重定向库、URDF |
| `SHARPA_MANUS_CALIBRATION_DIR` | 空（使用 `$SHARPA_MANUS_SDK/client`） | Manus 标定文件目录覆盖 |
| `SHARPA_WAVE_SDK` | `/opt/sharpa-wave-sdk` | 独立机械手 SDK |
| `SHARPA_MODELS` | `$PIXI_PROJECT_ROOT/../sharpa-urdf-usd-xml` | MuJoCo 模型与网格；仅 `sim` |
| `RETARGET_PYTHON` | `$PIXI_PROJECT_ROOT/.pixi/envs/retarget/bin/python` | 独立重定向解释器 |
| `ROS_DOMAIN_ID` | `42` | 默认 ROS 通信域 |

### 为什么使用两个 Pixi 环境？

| 环境 | Python | 职责 |
| --- | --- | --- |
| `default` | 3.12 | ROS 2 Jazzy 节点、MuJoCo，以及构建和运行本地 `manus_ros` |
| `retarget` | 3.10 | 运行上游重定向库及官方原版 IPOPT 求解器 |

上游重定向二进制使用 Python 3.10 ABI，不能直接加载到 Jazzy 的 Python 3.12 进程。项目以隔离子进程连接两者，避免 ROS 的 `PYTHONPATH` 污染重定向环境。原生 Manus 适配器与 ROS 节点在同一默认环境中构建和运行，因此可直接发布 ROS 数据。

重定向直接启动官方 SDK 的左右手优化进程，不替换其数值求解器，也不在启动时编译自定义目标函数。工作进程启动后发送 `READY`；每帧仍等待官方求解完成，避免将初始零值或上一帧结果当作新结果发布。

## 安装与构建

在项目根目录执行：

```bash
# 安装默认与重定向环境，使用项目锁文件中的依赖
pixi install --all

# 独立构建 ROS 包；sim 和 real 不会触发本地 Manus 适配器构建
pixi run build

# 可选：单独构建本地 Manus ROS 适配器
pixi run manus-build

# 检查 ROS、重定向库和 Sharpa SDK 的导入
pixi run doctor

# 可选：执行回归测试
pixi run test
```

标准手套生产者只有 `pixi run manus`：它依赖并自动执行 `build` 与 `manus-build`，然后启动完整生产者组。`sim` 和 `real` 都不会触发本地 Manus 适配器构建或要求 Manus client 标定文件；`real` 仍按其 `sdk_root` 参数使用外部重定向 URDF。`doctor` 只检查依赖，不代表许可证、手套连接或机械手通信已经就绪。

本地适配器构建产物为 `build/manus-native/manus_ros`。`scripts/manus_client.py` 以 Release 模式和 `-DMANUS_SDK=<外部 SDK>/client/ManusSDK` 配置 `native/`；不会暂存上游源文件、生成项目内协议桥接代码或修改外部 SDK。旧客户端环境的 CMake 缓存会在首次新配置时刷新。

如需单独启动已构建的适配器，可使用 `pixi run manus-native -- --ros-args -p calibration_dir:=/path/to/calibration`；它是调试入口，不是标准生产者命令。

更多命令说明：

```bash
pixi run help
```

## 快速开始：两终端工作流

### 标准 MuJoCo 仿真

终端 1 启动唯一的采集、ROS 桥接和重定向生产者：

```bash
pixi run manus
```

终端 2 只启动交互式 MuJoCo 消费者：

```bash
pixi run sim
```

`sim` 不启动 `retarget` 或 `sharpa_output`，也不会发现、使能或移动真实机械手。无窗口运行时：

```bash
pixi run sim headless:=true
```

`sim` 只等待 `/sharpa/{left,right}/command`；它不会生成演示动作。若要低层测试仿真，可由另一个符合接口约束的发布者提供这些命令。

### 直接查看 Manus 原始关键点

排查“人手伸直，但机械手某些手指仍弯曲”时，先绕过重定向：

```bash
pixi run rawmanus
```

此命令只启动 Manus 原生采集和三维关键点窗口，不启动重定向、机械手模型或真机输出。窗口直接订阅 `/manus/{left,right}/raw_poses`，每手显示 25 个关键点、编号及按手指着色的骨架连线。“原始”指采集适配器发布的数据：已转换为手根相对坐标并重排关键点顺序，但没有插值、关节求解或滤波。窗口使用 MuJoCo 渲染，不执行动力学。

若 `pixi run manus` 已经运行，使用订阅模式，**不要再启动第二个采集客户端**：

```bash
pixi run rawmanus with_client:=false
```

- 点 `0` 为手根；拇指 `1–4`（橙）、食指 `5–9`（青）、中指 `10–14`（绿）、无名指 `15–19`（粉）、小指 `20–24`（黄）。
- 左右手分开展示；显示偏移仅用于排版，不修改 ROS 数据。等待输入、正常更新和数据过期会明确区分，过期骨架不冒充实时姿态。
- 可用鼠标旋转、缩放视角。关闭窗口或按 Ctrl+C 会退出本次启动的诊断组；订阅模式不会停止已有的外部生产者。
- 若原始骨架里的无名指、小指已经弯曲，先检查 Manus 佩戴与标定；若原始骨架伸直、重定向后才弯曲，再检查映射与求解。此窗口本身不能证明真机反馈正常。

### 已有外部原始 ROS 发布者

若已有其他进程或主机在同一 ROS 域发布原始位姿，可只启动重定向：

```bash
pixi run manus with_client:=false
```

此模式只启动一组 `retarget`，适用于已有原始 ROS 发布者或集成测试。外部发布者必须在 `/manus/{left,right}/raw_poses` 提供每手 25 个有限 `geometry_msgs/PoseArray` 位姿、递增源时间戳，以及固定根相对 `frame_id`：`manus_left_hand` 或 `manus_right_hand`。

### 仿真模型与行为

`sim` 使用外部模型：

```text
wave_01/dual_sharpa_wave/dual_sharpa_wave.xml
```

- 从左右手 URDF 读取关节名称和限位，按名称映射 XML 执行器。
- 保留厂商手指动力学、碰撞配置和位置执行器参数。
- 双手共 **44 个手指关节**；当前不跟踪腕部或头部位姿，因此固定模型中额外的腕部、头部自由度。
- 在内存中解析外部网格路径，不修改或复制模型仓库资源。
- 控制默认 500 Hz，物理步长 `0.002 s`，积分器 `implicitfast`；窗口同步为 30 Hz。回调只替换最新目标，控制周期经 20 ms 时间常数指数平滑后驱动位置执行器。
- 没有输入超时或速度上限。停止输入后仍接近并保持最后目标；没有目标时保持初始实测位置。
- 反馈来自实际 MuJoCo `qpos/qvel`，不是目标位置回显。
- ROS 消息使用墙钟时间戳；目前不发布 `/clock`，不应为这条链路开启 `use_sim_time`。

## 真实机械手输出

> 首次实机运行前，检查机械手固定、工作空间、默认左右手序列号、左右手对应关系和实体急停。完全退出 Sharpa 控制软件（包括托盘后台 `pilot_sdk`），避免占用 SDK 发现端口 UDP `54321`。不要同时运行其他控制机械手的程序。

终端 1 保持唯一生产者：

```bash
pixi run manus
```

终端 2 直接将重定向命令送入安全输出：

```bash
pixi run real        # 默认双手
pixi run real left   # 仅左手
pixi run real right  # 仅右手
```

`real` 不启动 MuJoCo，不订阅仿真 `qpos`，也不需要模型仓库或图形显示。它直接订阅 `/sharpa/{left,right}/command`，且不使用仿真显示或模型启动参数。

默认左手 SN 为 `C55C9039C55F`，右手 SN 为 `CC549038CC57`。双手模式等待两侧都有有效目标才自动使能；目标不会随时间过期，任一侧断流也不会停用。单手模式只连接、校验和使能所选侧，忽略另一侧命令。SDK 或反馈异常仍会停用所有已选手。

`real` 默认注入 `dry_run:=false`、`auto_enable:=true` 和 `return_to_zero_on_exit:=true`，并且只为所选侧传入序列号。自动使能只尝试一次；显式停用或故障会取消等待。给未选侧提供非空 SN 是不兼容的，命令会被拒绝。

可覆盖选中侧设备或以安全预览方式调试输出：

```bash
# 使用另一套真机 SDK
pixi run real native_sdk_root:=/opt/sharpa-wave-sdk

# 输出节点的 dry-run 预览：不应移动真机
pixi run real dry_run:=true

# 只覆盖已选侧的设备
pixi run real left left_serial:=LEFT_SN
pixi run real right right_serial:=RIGHT_SN
```

`dry_run:=true` 是当前直接输出路径的调试预览。`real` 不会生成空的 `<arg>:=` 覆盖；需要自定义序列号时只传入选中侧。

正常 Ctrl+C 或 SIGTERM 时，健康且已使能的输出使用相同指数平滑回到 `0 rad`，实测位置进入回零容差后停用；不再按 1 rad/s 限速。回零总时限与 SDK 发现/通信故障处理仍保留，它们不是输入 Watchdog。故障、SIGKILL、掉电或 SDK 阻塞时不能保证回零，故障后不会自动重新使能。

推荐先在 `real` 终端按 Ctrl+C，等待回零、停用和退出，再停止 `manus`。**先停止生产者不会停止机械手**；输出会继续保持最后目标。零位指关节 `0 rad`，不是自动标定，回零期间必须保持工作空间清空。

`/sharpa/enable` 保留给调试和恢复，不是标准启动步骤。显式停用会取消等待中的自动使能；现场恢复时可按需调用：

```bash
pixi run ros2 service call /sharpa/enable std_srvs/srv/SetBool '{data: false}'
pixi run ros2 service call /sharpa/enable std_srvs/srv/SetBool '{data: true}'
```

## 录制一条 SpeedTest 数据

先运行 `pixi run manus`，再在项目目录的另一个终端运行：

```bash
pixi run record-speed-test
```

做完一条动作后按 **Ctrl+C**，保存到当前目录的 **`SpeedTest.HDF5`**。也可用 `pixi run record-speed-test --seconds 30` 定时录制。脚本为 `scripts/record_speed_test.py`，只订阅数据，不发布指令、不使能硬件；无需启动 `sim` 或 `real`。已有文件不会覆盖，重新录制前先移动旧文件，或用 `--output` 指定另一个文件。

文件按 `left/`、`right/` 分组，每侧包含：

| 分组 | 来源 | `values` 形状 | 用途 |
| --- | --- | --- | --- |
| `manus_raw` | `/manus/{side}/raw_poses` | `N × 25 × 7` | 后续从原始 Manus 输入开始测试整条链路 |
| `sharpa_joint` | `/sharpa/{side}/command` | `N × 22` | 后续跳过重定向，直接测试关节目标跟随 |

位姿列顺序为 `x,y,z,qx,qy,qz,qw`，位置单位 m；关节角单位 rad，顺序保存在 `joint_names` 属性。每组保存 `stamp_ns`、接收 ROS 时间 `received_ros_ns`、单调相对时间 `elapsed_ns`，单位 ns。各话题独立采样，不按数组下标强行配对；原始位姿与关节结果可按源时间戳关联。过载丢弃、best-effort 传输和录制边界可能导致部分输入没有对应输出。历史录制中的 `manus_input` 分组属于旧重采样路径，新录制不再生成它。

这里的 `sharpa_joint` 是未经过输出端裁剪的重定向目标，**不是机械手实际反馈**。这一个脚本只保存测试输入，不提供回放或实际跟随速度报告；后续两种测试都可读取同一文件，再另行测量实际反馈。退出时会打印各组帧数，空组会警告；文件属性 `complete=true` 表示正常停止并完成缓存写入，不保证所有话题都有数据。

## ROS 接口

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/manus/left/raw_poses`、`/manus/right/raw_poses` | `geometry_msgs/PoseArray` | `manus_ros` 直接发布的原始数据；每手 25 个关键点 |
| `/sharpa/left/command`、`/sharpa/right/command` | `sensor_msgs/JointState` | 官方原版 IPOPT 重定向输出；每手 22 个关节目标，单位 rad，保留输入时间戳 |
| `/sharpa/left/target`、`/sharpa/right/target` | `sensor_msgs/JointState` | `sharpa_output` 校验后的目标，不是测量值 |
| `/sharpa/left/joint_states`、`/sharpa/right/joint_states` | `sensor_msgs/JointState` | 真实 SDK 读取的关节反馈；仅 `real` 的真实输出模式 |
| `/sim/sharpa/left/joint_states`、`/sim/sharpa/right/joint_states` | `sensor_msgs/JointState` | MuJoCo 关节位置 rad、速度 rad/s；仅 `sim` |
| `/sharpa/enable` | `std_srvs/srv/SetBool` | 输出节点的可选调试/恢复使能与停用服务；标准 `real` 启动不需要调用 |

流式话题使用 **best-effort、volatile、keep-last**。通常深度为 1；`retarget` 输入深度为 4，吸收短时调度抖动。查看话题时建议显式选择 best-effort QoS：

```bash
pixi run ros2 topic list
pixi run ros2 topic echo /sim/sharpa/left/joint_states --qos-reliability best_effort
```

`manus_ros` 在真实 SDK 采集回调写入 `header.stamp`，发布手根相对坐标下的 25 个有限位置和归一化四元数。它按实际采集频率发布，没有额外频率定时器；`retarget` 直接订阅 `/raw_poses`，没有 `/poses` 重采样话题或 40 ms 插值缓冲。

重定向左右手独立并行，每手保留一个正在计算的姿态和至多 4 个待处理姿态；过载时丢弃最旧的待处理姿态，不中断当前求解、不无限积压。只有官方求解完成后才发布对应输入源时间戳的结果，不重复求解旧帧凑频率。停止输入后不再生成新关节结果，但两个消费者继续跟随最后目标。

数值后端为官方原版 IPOPT，保留模型、权重、状态更新和 `alpha=0.2` 关节滤波。实际结果频率由原始采集、IPOPT 吞吐及负载决定，不承诺每个输入都能完成求解。仿真与真机在各自 500 Hz 循环中使用同一 20 ms 时间常数平滑；输入消息年龄不再触发拒收或停用，关节限位、格式与顺序校验及 SDK 故障处理仍保留。

**历史 IPOPT 基线（当时的队列配置不同，不代表当前吞吐）**：同一份约 10 秒 `SpeedTest.HDF5` 的 1× 原始位姿回放对比（隔离 ROS 域、无真机），由 60 ms 缓冲/8 帧待处理改为 40 ms/仅最新帧后，左右源时间到关节输出的中位延迟从 102.94 / 101.87 ms 降为 54.64 / 54.80 ms，P99 从 110.85 / 110.96 ms 降为 60.95 / 61.27 ms。位姿仍约 250 Hz，该片段未出现超过 6 ms 的插值源时间间隔。关节输出间隔 P99 则从 14.71 / 14.28 ms 增至 17.32 / 17.93 ms，不能把延迟下降理解为吞吐或间隔抖动同时改善。在共同源时间网格上，新旧关节结果差异 RMSE 为 0.25° / 0.56°；相对录制关节的 RMSE 从 3.52° / 2.08° 变为 3.49° / 2.13°。这是单段录制实测，不保证其他动作或负载下的数值。

## 配置与参数路由

配置文件在启动时读取；修改后重启相应节点：

- [`src/sharpa_teleop/config/teleop.yaml`](src/sharpa_teleop/config/teleop.yaml)：真机指数平滑和输出生命周期参数。
- [`src/sharpa_teleop/config/sim.yaml`](src/sharpa_teleop/config/sim.yaml)：MuJoCo 节点参数。

### 主要节点参数

| 节点 | 参数 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `sharpa_output` | `dry_run` | YAML 中为 `true`；`real` 覆盖为 `false` | 是否禁止真机连接与运动 |
| `sharpa_output` | `left_serial` / `right_serial` | YAML 中为空；`real` 注入已选侧默认 SN | 明确选定机械手 |
| `sharpa_output` | `auto_enable` | YAML 中为 `false`；`real` 覆盖为 `true` | 一次性等待已选侧有效目标后自动使能，无输入年龄限制 |
| `sharpa_output` | `return_to_zero_on_exit` | YAML 中为 `false`；`real` 覆盖为 `true` | 正常退出时健康已使能输出回零后停用 |
| `sharpa_output` | `homing_timeout_sec` / `homing_tolerance_rad` | `10.0` / `0.02` | 回零总时限与实测收敛容差（rad） |
| `sharpa_output` / `mujoco_sim` | `smoothing_time_sec` | `0.02` | 指数平滑时间常数，秒；不是固定速度上限 |
| `sharpa_output` / `mujoco_sim` | `control_hz` / `feedback_hz` | `500.0` / `30.0` | 最新目标平滑执行与实测反馈频率 |

### Launch 参数

| 启动方式 | 可用参数与职责 |
| --- | --- |
| `pixi run manus` | `config`、`sdk_root`、`calibration_dir`、`worker_python`、`with_client`、`project_root`。`sdk_root` 同时路由给本地适配器和重定向；空的 `calibration_dir` 默认使用 `<sdk_root>/client`；`with_client:=false` 跳过本地适配器，保留外部原始 ROS 发布者；`project_root` 用于稳健定位适配器脚本。 |
| `pixi run rawmanus` | `sdk_root`、`calibration_dir`、`with_client`、`project_root`。仅原生采集与原始关键点窗口；`with_client:=false` 只订阅已有数据，不构建或连接 Manus SDK。 |
| `pixi run sim` | `config`、`models_root`、`headless`、`control_hz`、`smoothing_time_sec`、`feedback_hz`。仅仿真消费者；默认 500 Hz、20 ms 时间常数、30 Hz 反馈。 |
| `pixi run real [left\|right]` | `config`、`sdk_root`、`native_sdk_root`、`dry_run`、`auto_enable`、`return_to_zero_on_exit`、`homing_timeout_sec`、`homing_tolerance_rad`、`left_serial`、`right_serial`、`control_hz`、`smoothing_time_sec`。仅真机消费者；默认 500 Hz、20 ms 时间常数。 |

Launch 参数覆盖 YAML 同名值。`sim` 不接受生产者或真机生命周期参数；`real` 不接受 MuJoCo 或生产者参数。重定向的 `startup_timeout_sec=60`、`response_timeout_sec=0.5` 仅检测工作进程启动或请求无响应，不是输入断流 Watchdog；没有消息时不触发这些计时器。

常用覆盖示例：

```bash
# 使用已存在的直接 ROS 原始位姿发布者
pixi run manus with_client:=false

# 使用外部标定目录
pixi run manus calibration_dir:=/path/to/calibration

# 仅仿真使用另一份模型并禁用窗口
pixi run sim models_root:=/path/to/sharpa-urdf-usd-xml headless:=true

# 真机输出使用另一份原生硬件 SDK
pixi run real native_sdk_root:=/opt/sharpa-wave-sdk
```

长期更换 Manus SDK 位置时，修改 `pixi.toml` 默认环境的 `SHARPA_MANUS_SDK` 配置后重新运行 `pixi run manus-build`；`sdk_root` 会同时路由给本地适配器和重定向，并让适配器优先加载该根目录的 `client/ManusSDK/lib`。标定目录可通过 `calibration_dir` 或 `SHARPA_MANUS_CALIBRATION_DIR` 覆盖。`sdk_root` 指向 Manus/重定向仓库，不是独立机械手 SDK。

### ROS 通信域隔离

默认通信域为 `42`。每个域只应有一个 `manus` 生产者；实机域中不要同时使用测试发布者或不受控消费者。所有参与通信的终端必须使用相同的 `ROS_DOMAIN_ID`：

```bash
# 终端 1
ROS_DOMAIN_ID=76 pixi run manus

# 终端 2
ROS_DOMAIN_ID=76 pixi run sim
```

## 项目结构

```text
pixi.toml / pixi.lock            环境、任务与依赖锁文件
native/CMakeLists.txt            本地 Manus SDK → ROS 2 适配器构建
native/manus_ros.cpp             MANUS SDK 生命周期与直接 raw_poses 发布
native/manus_pose.hpp            关键点顺序、根坐标及旋转转换
scripts/
  workspace.py                   构建、启动、依赖检查和帮助
  manus_client.py                本地适配器构建与运行，不复制外部 SDK
src/sharpa_teleop/
  launch/manus.launch.py         原始频率采集与官方 IPOPT 生产者组
  launch/rawmanus.launch.py      可选原生采集与原始关键点诊断窗口
  launch/sim.launch.py           MuJoCo 消费者
  launch/real.launch.py          安全输出消费者
  config/                        节点配置
  sharpa_teleop/
    raw_manus.py                 原始关键点与指骨连线三维显示，不经过重定向
    retarget.py                  ROS 与独立重定向进程桥接
    retarget_worker.py           官方重定向库适配
    sharpa_output.py             真实输出及 dry-run
    safety.py                    关节、时间戳格式校验与指数平滑
    sim_model.py                 外部 MuJoCo 模型加载与动力学
    mujoco_sim.py                仿真 ROS 接口和窗口
tests/                           安全、重定向、仿真和输出生命周期回归测试
```

## 验证范围与已知限制

当前原始频率直连与 20 ms 指数平滑验证：

- `pixi run build` 成功，`pixi run test` 的 22 项回归测试通过，包括断流后继续跟随、目标反向替换、实际经过时间平滑、限位与时间戳校验、SDK 故障停用及容差回零。
- 隔离 ROS 域 119 中，录制原始位姿以非等间隔约 **105.26 Hz** 发布；真实 ROS 重定向节点使用官方 IPOPT，左右关节结果约 **83.19 / 79.15 Hz**。保留输入时间戳，不经重采样；这是该次回放的吞吐，不是真手套采集频率保证。
- 同时运行实际 MuJoCo 节点及记录后端替换硬件 SDK 的输出节点，左右下发约 **499.92 Hz**，仿真控制约 **499.99 Hz**；停止输入后测量 1 秒，两端每手仍各下发 500 次并收敛到同一最后目标。首次使用 10 秒前的有效递增时间戳目标也可使能并跟随，不再因消息年龄拒收。原生硬件 SDK 未加载，未连接或驱动真机。
- 实际 `sim` GLFW 窗口正常显示双手；关闭窗口后 launch 退出码为 0。订阅模式的实际 `manus` launch 仅启动重定向，其 ROS 订阅为 `/manus/{left,right}/raw_poses`。
- 数据与窗口截图在本地 `build/native-rate-follow/smoke.json`、`simulation.png`。官方求解过程中仍出现 `nlp_f` 的 Inf 诊断；本次输出通过有限值与收敛检查，没有修改官方求解器以隐藏诊断。该验证不证明真机运动安全、网络实时性或所有姿态的优化稳定性。

真机遥操作验证（2026-09-30，操作者提供日志与运行反馈）：

- `pixi run real` 成功连接 WaveSE-L-01 与 WaveSE-R-01；两手固件均为 **3.0.10**，日志显示与 **Sharpa SDK 5.0.11** 的版本检查通过。
- 输出先保持禁用，随后记录 `Automatic enable complete; tracking selected hands`；操作者确认右手真机遥操作正常。未量化真机延迟、网络下发频率或全关节范围，不据此宣称长期稳定或硬实时。
- 此前右手实测角度超出 URDF 范围会阻止使能；本项目没有放宽反馈限位或裁剪实测反馈来绕过该保护。该次成功不能单独证明标定是此前超限的唯一原因。

历史 IPOPT 恢复验证（当时仍有前级重采样）：

- 当时 `pixi run test` 的 26 项回归测试通过；移除加速求解器专属测试，保留首帧同步、输入校验、仿真与真机输出安全回归。当前链路的验证另列，历史数量不代表当前测试集合。
- 官方工作进程双手各处理 320 帧，关节名称、有限值、帧号及左右手对应检查通过。右手使用诊断时采到的微小变化片段循环回放；恢复后的全部右手结果与原 IPOPT 对照结果一致，最大角度差为 0。
- 同一回放稳定区间内，右手逐帧关节变化 RMS 为 **0.00251°**，最大单帧变化 **0.00736°**；左右请求并行提交后的往返耗时中位数 **12.27 ms**。这些是离线片段结果，不保证当前佩戴者或所有姿态的频率、延迟与稳定性。
- 结果保存在本地 `build/right-jitter-diagnosis/restored_ipopt.json` 和 `.npz`。未连接或使能真机；前级重采样与后级 500 Hz 输出设置均未修改。

原始关键点诊断窗口验证：

- `pixi run rawmanus --show-args` 完成 ROS 包及本地采集适配器构建，并正确列出启动参数。已有的 CLI 路由回归测试通过。
- 在隔离 ROS 域 119 使用录制的 `manus_raw` 双手关键点运行订阅模式，实际 GLFW 窗口显示双手彩色骨架、点编号及 LIVE 状态；停止发布后，两手骨架隐藏并显示红色 STALE。窗口管理器正常关闭后，launch 退出码为 0。
- 验证未连接手套或机械手；使用无效 SDK 路径仍可运行 `with_client:=false`。窗口截图保存在本地 `build/rawmanus-smoke/live.png`、`stale.png`。录制数据验证不代替当前佩戴者的实时标定检查。

历史限速与输入 Watchdog 输出层验证（两项现已移除）：

- 当时 `pixi run test` 的 27 项 Python 回归测试通过。旧输出插值测试随旧实现删除；覆盖单帧启动、目标反向替换、到达目标后持续下发但不刷新 Watchdog、调度延误不追赶大步。当前实现和验证见本节开头。
- 在隔离 ROS 域 119 中，用约 **102.56 Hz** 的非等间隔合成关节目标驱动真实 ROS 输出节点 30 秒；硬件接口替换为零 I/O 延时的记录后端，未加载原生 SDK、未连接或使能真机。左右记录接口调用均约 **499.73 Hz**，首次输入到首个运动指令约 **3.11 / 3.12 ms**；单步最大 **0.002 rad**。间隔 P99 为 **2.46 / 2.47 ms**，最大约 **13.32 ms**，不是硬实时保证。
- 停止输入后约 **501.27 ms** 停用，之后不再下发。结果保存于本地 `build/speedtest-results/latest_follow_500hz.json` 和对应 `.npz`；这些数据验证主机输出逻辑，不代表真机网络时序或电机响应。
- 本机直接使用多线程执行器 `spin()` 时，曾出现回调调度饥饿及提前断流。保留双线程隔离 SDK 调用与输入回调，改为每次 `spin_once()` 分发后让出 100 μs；上述持续测试使用此调度方式，不靠放宽 Watchdog 达标。

历史 L-BFGS-B 加速后端的离线验证（该后端已移除，以下频率不代表当前 IPOPT）：

- 此前 `pixi install -e retarget`、`pixi run build` 和当时的 28 项 Python 回归测试通过；当前测试结果见本节开头。
- 默认输入为 250 Hz，源时间缓冲仍为 40 ms。在隔离 ROS 域 107 中，将 `SpeedTest-250hz.HDF5` 的 2,477 帧/手循环回放 3 遍，预热后测量约 30 秒。左右手各收到并输出 7,431 帧，无丢帧、无重复源时间戳，实际输出为 **250.025 / 250.024 Hz**。
- `/poses` 发布到 `/command` 接收的中位延迟为 **4.01 / 4.18 ms**，P99 为 **9.38 / 9.88 ms**；这些数值不包含前级 40 ms 缓冲。输出间隔 P99 为 **6.23 / 6.06 ms**，最大 **16.28 / 16.47 ms**。这是持续平均 250 Hz，不是硬实时每 4 ms 必达的保证。
- 该次全部 7,431 帧/手均为有限值、源时间递增、关节名称与 URDF 顺序匹配，原始结果已在关节限位内，无须裁剪。相对原算法按相同录制帧对齐、排除最初 50 帧后，轨迹 RMSE 为 **8.11° / 6.68°**，最大关节差异 **47.58° / 36.52°**。消除历史运动项后的静态目标函数均值约增加 **3.13% / 4.03%**，不能将提频称为无损优化。
- 报告保存在本地 `build/speedtest-results/250_ros_verified.json`、对应 `.npz` 和 `250_quality_verified.json`。未连接真机；更换算法后的动作应先在仿真中确认，再决定是否用于真实机械手。其他动作、系统负载或原始输入断流下的频率与质量不由此次测试保证。

以下是直接 ROS 路径在使用原 IPOPT 后端时的历史记录：
- 隔离 ROS 域中以合成原始 `PoseArray` 驱动 `manus with_client:=false`：8 秒收到左右手 1963 / 1962 帧插值位姿和同等数量的真实官方重定向结果。持续重发旧时间戳后，位姿和关节输出停止，未将接收流量冒充新鲜源数据。
- 真实手套直接 ROS 采样通过：重新插拔接收器恢复 SDK 初始化后，15 秒窗口内左右手原始位姿约 116.89 / 117.01 Hz，插值位姿约 249.99 / 250.00 Hz；原始消息最大观测年龄分别为 0.861 / 1.147 ms。每手只有一个 `manus_native` 原始发布者，25 关键点、有限值、单位四元数、根坐标和时间戳检查通过；左右标定文件成功加载。
- 同次测量左右真实重定向输出约 194.98 / 244.09 Hz，求解结果时间戳均能匹配输入插值位姿；直连 ROS 不代表求解器已达到稳定 250 Hz。运行中旧 ZMQ 2044 端口没有监听者；SIGINT 后整个生产者组退出码为 0。未启动或使能真实机械手。

以下记录全部来自切换到直接 ROS 传输之前，只作历史背景：它们不证明当前 `manus_ros` 路径，也不是 ZMQ/Protobuf 的运行说明。

### 切换前历史验证记录

- 双级时间插值更新：构建与回归测试通过；新增关节时间插值边界、四元数 SLERP 及上游递增 frame_id 回归覆盖。
- 在隔离 ROS 域中，用合成原生格式 ZMQ 数据驱动旧 `manus with_client:=false`，运行旧桥接和官方重定向库。10 秒采样收到左手 160 帧位姿、160 帧重定向命令、88 帧通过校验的直接 dry-run 目标，以及 298 帧 MuJoCo 反馈；匹配的输出目标与重定向位置及源时间戳一致。
- 输入桥接和重定向各只有一个发布者；仿真不再发布用于驱动真机的命令话题。
- 停止 MuJoCo 后，生产者与直接 dry-run 输出继续工作，5 秒收到 78 帧重定向命令和 68 帧通过校验的直接目标；真实输出消费者启动时使用不存在的仿真模型路径，证明该路径不依赖 MuJoCo 模型资源。
- 旧原生客户端伪终端接入实测：在隔离 ROS 域启动完整旧生产者，5 秒收到左/右手 474 / 406 帧有效位姿和 242 / 230 帧重定向关节命令。SIGINT 后旧生产者组退出码为 0，旧传输端口释放；未启动真机输出。
- 双级插值实测（隔离 ROS 域、旧 Manus 客户端，未启用真机）：一次 15 秒窗口内原始左/右手约 103.73 / 105.15 Hz，插值位姿均为 250.00 Hz；真实重定向结果为 233.67 / 249.69 Hz，尚未达到双手稳定各 250 Hz。录制的真实姿态序列直接回放官方优化器，左右手约 284.65 / 243.87 次求解每秒，右手 P99 求解耗时约 6.206 ms。
- 合法 250 Hz 关节斜坡经真实输出定时器和假硬件后端验证，重采样约 500.00 Hz，间隔中位数 1.999 ms；停止输入后仍触发 watchdog。当时部分真实重定向结果超出 URDF 限位，被输出端拒收，未证明真实 Manus 到硬件的连续 500 Hz 执行；后续目标接收规则已改为裁剪有限越界角度，不扩大关节限位。
- Pixi 环境安装、ROS 包构建、旧 Manus C++ 客户端构建，以及 ROS、官方重定向库、Sharpa Python SDK 的导入。
- 合成 Manus Protobuf → 官方重定向库 → ROS → dry-run 输出，以及合成 Manus Protobuf → 官方重定向库 → ROS → MuJoCo 动力学。
- 双手独立仿真目标跟踪、错误指令拒绝、断流保持、窗口启动和渲染。
- 接入 Manus 许可证后的历史采样：4 秒内收到左手 467 帧、右手 470 帧有效 25 关键点数据；另一次 5 秒 ROS 采样收到左右手 240 / 238 条有效关节目标和每手 147 条仿真反馈。
- 独立 Sharpa Wave SDK `5.0.11` 的只读设备发现：成功解析 `0302` 心跳并发现固件 `3.0.10` 的左手；未连接控制通道、使能或发送运动指令。
- 历史 headless MuJoCo → dry-run 输出和模拟硬件生命周期试验；未连接真实机械手。

## 常见问题

### `No compatible license found`

这是 Manus SDK 的授权提示，不是 ROS 或 Pixi 编译错误。连接或配置包含 SDK component 权限的有效许可证，并按照厂商流程连接、标定手套。不要同时运行另一个 MANUS Core 或 Core Integrated 实例。

### 只看仿真，是否需要 Manus 许可证？

`sim` 本身只是命令消费者，不启动 Manus 客户端；用符合接口的外部命令发布者测试仿真时不需要 Manus 许可证。标准两终端手套工作流仍需要许可证，因为 `manus` 生产者需要采集和重定向手套数据。

### 窗口无法打开或没有显示环境

只对仿真使用：

```bash
pixi run sim headless:=true
```

该模式仍运行物理仿真并发布 ROS 反馈，但不创建窗口。`real` 没有 MuJoCo 窗口或显示设置。

确认终端 1 正在运行 `pixi run manus`（或订阅模式下已有原始发布者），并确认所有终端使用相同 `ROS_DOMAIN_ID`。用 best-effort QoS 检查 `/manus/{left,right}/raw_poses` 和 `/sharpa/{left,right}/command`。仿真反馈在 `/sim/sharpa/*/joint_states`，真实反馈在 `/sharpa/*/joint_states`；不再有重采样 `/poses` 话题。断流不会停用输出，停机必须显式操作。

### `cannot arm: timed out ... waiting for selected HAND device(s): left`

这是原生 SDK 没有发现指定设备，不是 Manus 断流，也不是另一只手的关节越限导致。先核对 SN、左右手、设备供电和网段，再检查发现端口：

```bash
ss -ulpn 'sport = :54321'
```

如果显示 `pilot_sdk` 或另一个控制程序，请完全退出该程序（包括托盘后台）再重启 `pixi run real`。输出节点会锁定启动失败状态；释放端口后仅调用 `/sharpa/enable` 不会重新连接。重启后它会重新等待新鲜的 `/sharpa/{left,right}/command` 再自动使能一次。不要关闭关节限位检查绕过发现失败。

### `HeartPacket Unsupported protocol version: 0302`

心跳已到达，但当前硬件 SDK 不支持设备的心跳协议。Manus 仓库附带的 `SharpaWaveSDK_4.6.6` 在固件 `3.0.10` 上出现此错误；本机独立安装的 `/opt/sharpa-wave-sdk`（`5.0.11`）已验证能解析该心跳。用 `native_sdk_root` 指定厂商提供的兼容 SDK；不要覆盖 Manus 重定向库或绕过协议检查。`pixi run doctor` 会显示实际导入的硬件 SDK 路径。

### `target expired while arming`

输出会同时检查原始源时间和本地接收时间；使能结束时若目标已过期，会请求停用且不会自动重新使能。不要通过调大 `timeout_sec` 掩盖断流。检查已选侧的 `/sharpa/{left,right}/command` 是否持续更新，以及是否存在重复生产者、越限或重定向失败。

### 无法导入重定向库或 NumPy 报 Python 版本错误

执行 `pixi install --all` 和 `pixi run doctor`。不要在 Jazzy 的 Python 3.12 进程中直接加载上游 Python 3.10 二进制，也不要混用系统 ROS、其他 Conda 环境的 Python 路径。

### 上游 shared-memory 警告

V4.0 重定向库在退出时可能报告 Python `resource_tracker` 的 shared-memory 警告。项目会清理自身工作进程；不要通过关闭保护检查或伪造输出绕过上游运行错误。

## 安全与许可

- 硬件输出检查关节数量、名称顺序、有限值、源时间戳与本地接收时效；有限的上游目标角度裁剪到 URDF 限位内，硬件反馈仍严格校验。健康且已使能的实机仅在 Ctrl+C/SIGTERM 时按速度限制回到全关节 `0 rad` 后停用；故障或过期输入会跳过回零，且不会自动重新使能。
- 网络故障、SDK 阻塞、进程被 `SIGKILL` 或掉电时，软件不能保证回零或电机已经停用。
- 本项目不是硬实时或安全认证控制器。实机操作必须保留可用的实体急停和现场监护。
- `native/manus_ros` 是本项目的本地适配器源代码；官方 Manus SDK 头文件、库和操作者标定，以及官方重定向二进制和模型资源均保留在外部仓库并适用各自许可证。构建只会包含/链接或读取这些外部资源，不会复制、暂存、重新授权或随本项目分发它们；使用或分发前请阅读上游 `License` / `LICENSE.txt` 和 `NOTICE.txt`。
