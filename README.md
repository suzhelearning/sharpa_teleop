# Sharpa Teleop

基于 **Pixi + ROS 2 Jazzy** 的 Manus 手套遥操作项目。基于官方 Sharpa 运动学模型与目标函数，将双手关键点转换为 SharpaWave 关节目标；同一套生产者可接入 **MuJoCo 仿真** 或 **真实机械手安全输出**。

> 标准拓扑固定为两个终端：终端 1 运行唯一生产者 `pixi run manus`，终端 2 选择**一个**消费者：`pixi run sim` 或 `pixi run real [left|right]`。`real` 默认双手，直接消费重定向目标；它不启动 MuJoCo。软件保护不能替代实体急停。

## 数据链路

```text
终端 1：pixi run manus（唯一生产者组）
Manus 手套
  │ 本项目原生 C++ `manus_ros`（直接 Manus SDK）
  ▼
/manus/{left,right}/raw_poses（25 个关键点 PoseArray）
  │ manus_input（40 ms 源时间缓冲，按 250 Hz 重采样）
  ▼
/manus/{left,right}/poses
  │ retarget（官方目标函数，编译内核 + L-BFGS-B）
  ▼
/sharpa/{left,right}/command（22 个关节，rad；保留输入时间戳）
  ├─────────────────────────────────────────────────────┐
  │                                                     │
  ▼                                                     ▼
终端 2：pixi run sim                         终端 2：pixi run real [left|right]
mujoco_sim                                  sharpa_output
  │ /sim/sharpa/{left,right}/joint_states    │ 安全校验、使能、原生 SDK 输出
  ▼                                           ▼
MuJoCo 反馈                                  已选定的真实机械手
```

`manus.launch.py` 启动项目本地的 `manus_ros`、`manus_input` 和 `retarget`。`manus_ros` 直接调用 Manus C++ SDK、管理其生命周期和退出信号，并直接发布 ROS 原始位姿；项目运行路径中没有中间网络帧转发、生成协议桥接代码或伪终端。它们是一个生产者组：组内任一已启动进程退出，launch 会关闭该组，避免留下重复或半截输入链路。`sim` 只启动 MuJoCo 消费者；`real` 只启动安全输出消费者。两者都不再启动输入桥接或重定向节点。

仿真只发布 `/sim/sharpa/{left,right}/joint_states` 反馈；真实输出直接订阅 `/sharpa/{left,right}/command`。不要在同一 ROS 域把 `sim` 和 `real` 当作同一条链路的两个阶段运行。

## 环境与外部依赖

- Linux x86-64；当前验证环境为 Ubuntu 24.04。
- 已安装 [Pixi](https://pixi.sh/)。ROS 2 Jazzy 和 `manus_ros` 所需的 C++ 工具链通过默认 Pixi 环境提供，不要求系统预装 ROS。
- `native/manus_ros` 是本项目的本地 C++ 适配器：它直接调用 Manus SDK 并发布 ROS 2 `PoseArray`，不需要客户端专用 Pixi 环境、终端伪设备或网络帧转发。
- [Sharpa Manus SDK](https://github.com/sharpa-robotics/sharpa-manus-sdk) 保持在项目外部：其 `client/ManusSDK` 的授权头文件和库只在构建/运行时被包含和链接，操作者标定文件也从外部 `client` 目录（或 `calibration_dir`）读取。本项目不会复制、暂存或重新授权这些 SDK/标定资源。
- 真机另需与设备固件兼容的 **Sharpa Wave SDK**。默认路径为 `/opt/sharpa-wave-sdk`；可通过 `SHARPA_WAVE_SDK` 或 `native_sdk_root` 指定。项目不会自动回退到 Manus 仓库附带的旧硬件 SDK。
- 关节遥操作连接使用 `SharpaWaveConfig.disable_sync_time=true` 和 `disable_tactile=true`，关闭 SDK 启动时的 HTTPS 设备校时和触觉初始化；安全时效检查仍使用电脑侧 ROS 时间戳和单调时钟。关节控制、限位、断流保护及退出回零逻辑不变。
- 真机输出以 `control_hz=500.0`（2 ms）对带时间戳的关节轨迹做线性插值，再应用 `max_velocity=1.0` rad/s 限速；反馈发布仍为 30 Hz。`sharpa_output.buffer_delay_sec=0.08` 是相对于输入源时间的总延迟，包含前级缓冲及求解耗时，不是再额外增加 80 ms。缺少前后两帧时保持最后下发位置，不外推；输入断流仍触发停止。500 Hz 是主机侧循环目标，并非硬件伺服或网络发包频率保证。
- 仿真和真机接收关节目标时，按各关节 URDF 上下限裁剪有限角度，再进入各自的目标处理流程；例如 `-0.52559` 在下限为 `-0.5236` 时裁剪为 `-0.5236` rad。NaN/Inf、名称或数量错误、过期、未来及乱序时间戳仍拒收。硬件实测位置和最终 SDK 下发边界仍严格检查限位，不裁剪反馈来掩盖异常；断流保护和限速不变。
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
| `retarget` | 3.10 | 运行上游重定向库、编译目标函数并执行数值优化 |

上游重定向二进制使用 Python 3.10 ABI，不能直接加载到 Jazzy 的 Python 3.12 进程。项目以隔离子进程连接两者，避免 ROS 的 `PYTHONPATH` 污染重定向环境。原生 Manus 适配器与 ROS 节点在同一默认环境中构建和运行，因此可直接发布 ROS 数据。

`retarget` 环境包含 C 编译器。启动时为左右手分别编译官方目标函数及解析梯度，加载后立即删除临时生成文件，不修改或分发外部 SDK。两手编译完成后才发送 `READY`；启动等待上限为 60 秒，与单帧 `timeout_sec` 分开。

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

`sim` 不启动 `manus_input`、`retarget` 或 `sharpa_output`，也不会发现、使能或移动真实机械手。无窗口运行时：

```bash
pixi run sim headless:=true
```

`sim` 只等待 `/sharpa/{left,right}/command`；它不会生成演示动作。若要低层测试仿真，可由另一个符合接口约束的发布者提供这些命令。

### 已有外部原始 ROS 发布者

若已有其他进程或其他主机在同一 ROS 域直接发布原始位姿，可让标准生产者只启动重采样和重定向：

```bash
pixi run manus with_client:=false
```

此模式仍只启动一组 `manus_input` 和 `retarget`，适用于已有直接 ROS 发布者或安全集成测试。外部发布者必须在 `/manus/{left,right}/raw_poses` 提供每手 25 个有限 `geometry_msgs/PoseArray` 位姿、正确源时间戳，以及对应的固定根相对 `frame_id`：`manus_left_hand` 或 `manus_right_hand`。

### 仿真模型与行为

`sim` 使用外部模型：

```text
wave_01/dual_sharpa_wave/dual_sharpa_wave.xml
```

- 从左右手 URDF 读取关节名称和限位，按名称映射 XML 执行器。
- 保留厂商手指动力学、碰撞配置和位置执行器参数。
- 双手共 **44 个手指关节**；当前不跟踪腕部或头部位姿，因此固定模型中额外的腕部、头部自由度。
- 在内存中解析外部网格路径，不修改或复制模型仓库资源。
- 物理步长为 `0.002 s`，积分器为 `implicitfast`；窗口同步为 `30 Hz`。
- 指令过期后保持当前仿真姿态；新鲜指令到达后恢复跟踪。
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

默认左手 SN 为 `C55C9039C55F`，右手 SN 为 `CC549038CC57`。双手模式要求两侧都有新鲜、有效目标才会自动使能；任一已选手断流会停用所有已选手。单手模式只连接、校验和使能所选侧，且会忽略另一侧的 `/sharpa/*/command`。由于实机路径没有仿真器，单手实机模式不会出现“未使用的右手仿真器”告警。

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

正常 Ctrl+C 或 SIGTERM 时，健康且已使能的真实输出会按 `max_velocity` 将所有已选关节缓慢回到 `0 rad`，确认收敛后再停用。launch 为此保留 30 秒 SIGINT 等待和额外 10 秒 SIGTERM 等待。故障、看门狗停用、SIGKILL、掉电或 SDK 阻塞时不能保证回零；故障或过期输入会跳过回零，且不会自动重新使能。

停止顺序必须是：**先在 `real` 终端按 Ctrl+C，等待回零、停用和退出，再停止 `manus`**。若先停止生产者，输入看门狗会触发故障停用，后续退出不再主动回零。零位指各关节 `0 rad`，不是自动标定；回零期间必须保持工作空间清空。

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
| `manus_input` | `/manus/{side}/poses` | `N × 25 × 7` | 保存实际送入重定向的插值位姿 |
| `sharpa_joint` | `/sharpa/{side}/command` | `N × 22` | 后续跳过重定向，直接测试关节目标跟随 |

位姿列顺序为 `x,y,z,qx,qy,qz,qw`，位置单位 m；关节角单位 rad，顺序保存在 `joint_names` 属性。每组同时保存原始消息 `stamp_ns`、本机接收 ROS 时间 `received_ros_ns`、从录制开始计算的单调时间 `elapsed_ns`，单位均为 ns。各话题独立采样，不按数组下标强行配对；插值位姿与关节结果可按源时间戳关联。ROS 使用 best-effort，录制边界及传输丢帧可能造成部分时间戳无法配对。

这里的 `sharpa_joint` 是未经过输出端裁剪的重定向目标，**不是机械手实际反馈**。这一个脚本只保存测试输入，不提供回放或实际跟随速度报告；后续两种测试都可读取同一文件，再另行测量实际反馈。退出时会打印各组帧数，空组会警告；文件属性 `complete=true` 表示正常停止并完成缓存写入，不保证所有话题都有数据。

## ROS 接口

| 接口 | 类型 | 内容 |
| --- | --- | --- |
| `/manus/left/raw_poses`、`/manus/right/raw_poses` | `geometry_msgs/PoseArray` | `manus_ros` 直接发布的原始数据；每手 25 个关键点 |
| `/manus/left/poses`、`/manus/right/poses` | `geometry_msgs/PoseArray` | 每手 25 个关键点；按源时间缓冲重采样为 250 Hz，位置线性插值、旋转最短路径 SLERP |
| `/sharpa/left/command`、`/sharpa/right/command` | `sensor_msgs/JointState` | 基于官方目标函数的加速重定向输出；每手 22 个关节目标，单位 rad，保留输入时间戳 |
| `/sharpa/left/target`、`/sharpa/right/target` | `sensor_msgs/JointState` | `sharpa_output` 校验后的目标，不是测量值 |
| `/sharpa/left/joint_states`、`/sharpa/right/joint_states` | `sensor_msgs/JointState` | 真实 SDK 读取的关节反馈；仅 `real` 的真实输出模式 |
| `/sim/sharpa/left/joint_states`、`/sim/sharpa/right/joint_states` | `sensor_msgs/JointState` | MuJoCo 关节位置 rad、速度 rad/s；仅 `sim` |
| `/sharpa/enable` | `std_srvs/srv/SetBool` | 输出节点的可选调试/恢复使能与停用服务；标准 `real` 启动不需要调用 |

流式话题使用 **best-effort、volatile、keep-last**。通常深度为 1；`retarget` 的位姿订阅深度为 4，用于吸收 250 Hz 下的短时调度抖动。查看话题时建议显式选择 best-effort QoS：

```bash
pixi run ros2 topic list
pixi run ros2 topic echo /sim/sharpa/left/joint_states --qos-reliability best_effort
```

`manus_ros` 在 SDK 采集回调的本地 ROS 时间写入 `header.stamp`，并把数据转换到实际固定的根相对坐标系：`manus_left_hand` 或 `manus_right_hand`。每条原始消息都有 25 个有限位置和归一化四元数。`manus_input` 保留这些 header 时间戳作为源时间，以 `buffer_delay_sec=0.04` 建立姿态插值缓冲；只有真实输入前后帧包围查询时间时才发布 `/poses`，不跨过最新真实帧外推，也不以重复旧数据刷新时效。

重定向左右手独立并行求解，每手保留一个正在计算的姿态和至多 4 个待处理姿态，按顺序计算；持续过载时丢弃最旧的待处理姿态，不中断当前求解，不无限积压。真实求解完成后才发布对应源时间戳的关节结果，不通过重复旧结果或关节插值凑输出频率。长时间断流超过缓冲能力时不能保证持续插值输入，严禁靠重标时间戳伪装新鲜数据。

数值后端使用编译后的官方目标函数和解析梯度，以 L-BFGS-B 求解，最多 20 次迭代；保留官方模型、代价权重及 `alpha=0.2` 的关节滤波。首帧建立初值及数值线搜索失败时，使用原 IPOPT 对同一帧重新求解。与原 SDK 一样，允许有限的迭代上限近似解，不宣称每帧均已收敛。**目标函数未更改，但求解算法已更换，关节轨迹不保证与原 IPOPT 一致。** 关节限位、速度限制、时间戳校验和看门狗仍由原安全输出链路执行。

**历史基线（原 IPOPT，非当前后端）**：同一份约 10 秒 `SpeedTest.HDF5` 的 1× 原始位姿回放对比（隔离 ROS 域、无真机），由 60 ms 缓冲/8 帧待处理改为 40 ms/仅最新帧后，左右源时间到关节输出的中位延迟从 102.94 / 101.87 ms 降为 54.64 / 54.80 ms，P99 从 110.85 / 110.96 ms 降为 60.95 / 61.27 ms。位姿仍约 250 Hz，该片段未出现超过 6 ms 的插值源时间间隔。关节输出间隔 P99 则从 14.71 / 14.28 ms 增至 17.32 / 17.93 ms，不能把延迟下降理解为吞吐或间隔抖动同时改善。在共同源时间网格上，新旧关节结果差异 RMSE 为 0.25° / 0.56°；相对录制关节的 RMSE 从 3.52° / 2.08° 变为 3.49° / 2.13°。这是单段录制实测，不保证其他动作或负载下的数值。

## 配置与参数路由

配置文件在启动时读取；修改后重启相应节点：

- [`src/sharpa_teleop/config/teleop.yaml`](src/sharpa_teleop/config/teleop.yaml)：原始位姿重采样、重定向和安全输出的节点参数。
- [`src/sharpa_teleop/config/sim.yaml`](src/sharpa_teleop/config/sim.yaml)：MuJoCo 节点参数。

### 主要节点参数

| 节点 | 参数 | 默认值 | 含义 |
| --- | --- | --- | --- |
| `manus_input` | `output_hz` / `buffer_delay_sec` | `250.0` / `0.04` | 保留原始 header 源时间的重采样频率和插值缓冲 |
| `retarget` | `timeout_sec` | `0.5` | 重定向结果有效时限 |
| `sharpa_output` | `dry_run` | YAML 中为 `true`；`real` 覆盖为 `false` | 是否禁止真机连接与运动 |
| `sharpa_output` | `left_serial` / `right_serial` | YAML 中为空；`real` 注入已选侧默认 SN | 明确选定机械手 |
| `sharpa_output` | `auto_enable` | YAML 中为 `false`；`real` 覆盖为 `true` | 一次性等待新鲜已选侧目标后自动使能 |
| `sharpa_output` | `return_to_zero_on_exit` | YAML 中为 `false`；`real` 覆盖为 `true` | 正常退出时健康已使能输出回零后停用 |
| `sharpa_output` | `homing_timeout_sec` / `homing_tolerance_rad` | `10.0` / `0.02` | 回零总时限与实测收敛容差（rad） |
| `sharpa_output` | `timeout_sec` | `0.5` | 源时间和本地接收时间的有效时限 |
| `sharpa_output` | `max_velocity` | `1.0` | 每关节指令速度上限，rad/s |
| `mujoco_sim` | `timeout_sec` / `feedback_hz` | `0.5` / `30` | 断流保持时限与反馈发布频率 |

### Launch 参数

| 启动方式 | 可用参数与职责 |
| --- | --- |
| `pixi run manus` | `config`、`sdk_root`、`calibration_dir`、`worker_python`、`with_client`、`project_root`。`sdk_root` 同时路由给本地适配器和重定向；空的 `calibration_dir` 默认使用 `<sdk_root>/client`；`with_client:=false` 跳过本地适配器，保留外部原始 ROS 发布者；`project_root` 用于稳健定位适配器脚本。 |
| `pixi run sim` | **仅** `config`、`models_root`、`headless`。它只是 MuJoCo 命令消费者。 |
| `pixi run real [left\|right]` | `config`、`sdk_root`、`native_sdk_root`、`dry_run`、`auto_enable`、`return_to_zero_on_exit`、`homing_timeout_sec`、`homing_tolerance_rad`、`left_serial`、`right_serial`。它只是安全输出消费者。 |

Launch 参数覆盖 YAML 同名值。`sim` 不接受生产者或输出控制参数；`real` 不接受 MuJoCo 或生产者参数。

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
  launch/manus.launch.py         本地适配器、输入重采样和重定向生产者组
  launch/sim.launch.py           MuJoCo 消费者
  launch/real.launch.py          安全输出消费者
  config/                        节点配置
  sharpa_teleop/
    manus_input.py               原始 PoseArray → 250 Hz ROS 重采样
    retarget.py                  ROS 与独立重定向进程桥接
    retarget_worker.py           官方重定向库适配
    retarget_solver.py           编译官方目标函数并执行 L-BFGS-B 求解
    sharpa_output.py             真实输出及 dry-run
    safety.py                    关节、时间戳和速度约束
    sim_model.py                 外部 MuJoCo 模型加载与动力学
    mujoco_sim.py                仿真 ROS 接口和窗口
tests/                           安全、重定向、仿真和输出生命周期回归测试
```

## 验证范围与已知限制

当前加速后端的离线验证：

- `pixi install -e retarget`、`pixi run build`、`pixi run test` 通过，Python 回归测试 28 项。
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

确认终端 1 正在运行 `pixi run manus`（或 `with_client:=false` 时已有直接 ROS 原始位姿发布者），并确认所有终端使用相同的 `ROS_DOMAIN_ID`。先用 best-effort QoS 检查 `/manus/{left,right}/raw_poses`，再检查 `/manus/{left,right}/poses` 和重定向命令。仿真反馈只在 `sim` 的 `/sim/sharpa/*/joint_states`；真实反馈只在 `real` 的 `/sharpa/*/joint_states`。

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
