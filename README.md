# Sharpa Teleop

将 **Manus 数据手套**的双手动作转换为 **SharpaWave 灵巧手**关节目标，并通过 **MuJoCo 仿真**或**真实机械手**观察和执行。

项目使用 **Pixi** 管理依赖、**ROS 2 Jazzy** 传递数据：每手 25 个关键点输入，经官方运动学模型、目标函数与原版 IPOPT 求解器转换为每手 22 个关节角，双手共 44 个关节。目前不跟踪腕部或头部位姿。

> **真机安全须知：** `pixi run real` 默认连接真实设备，并在收到所选手的有效目标后自动使能。当前没有输入超时停用机制，也没有固定速度上限；停止手套或重定向进程后，机械手仍会接近并保持最后目标。软件保护不能替代实体急停。

## 数据链路

```text
终端 1：pixi run manus

Manus 手套
    │ 本项目原生 C++ 适配器，直接调用 Manus SDK
    ▼
/manus/{left,right}/raw_poses       每手 25 个关键点
    │ retarget → 独立 Python 3.10 工作进程 → 官方 IPOPT
    ▼
/sharpa/{left,right}/command       每手 22 个关节目标，rad
    │
    ├── 终端 2：pixi run sim
    │       最新目标 → 指数平滑 → MuJoCo 位置执行器
    │       反馈：/sim/sharpa/{left,right}/joint_states
    │
    └── 终端 2：pixi run real [left|right]
            最新目标 → 指数平滑 → Sharpa Wave SDK
            反馈：/sharpa/{left,right}/joint_states
```

标准工作流是 **一个生产者 + 一个消费者**，第二个终端选择 `sim` 或 `real`：

- `manus` 只启动采集与重定向。组内任一已启动进程退出，launch 会关闭该组。
- `sim`、`real` 独立消费关节命令，不启动采集或重定向，也不串联。
- `real` 不启动 MuJoCo，不读取仿真关节状态。
- Manus 按 SDK 实际采集回调发布，重定向直接订阅原始话题，没有中间重采样、插值缓冲或网络帧转发。
- 重定向保留输入时间戳，并等待官方求解完成后才发布结果；过载时不保证每个输入都有输出。

## 环境与随仓库资源

### 运行要求与资源边界

- **Linux x86-64**；项目已有 Ubuntu 24.04 运行记录。
- 已安装 [Pixi](https://pixi.sh/)。ROS 2 Jazzy 和 C++ 工具链由 Pixi 提供，不要求系统预装 ROS。
- 本项目以 `sharpa_teleop/` 为唯一上层目录。克隆本仓库会同时取得 `pixi.toml`、`pixi.lock` 和 `vendor/`；不需要维护兄弟目录、额外克隆 SDK/模型，或将这些资源安装到 `/opt`。
- `vendor/` 中的三个本地 wheel 包含获授权的 [Sharpa Manus SDK](https://github.com/sharpa-robotics/sharpa-manus-sdk) 采集与 V4.0 重定向资源、[Sharpa Wave URDF/USD/XML](https://github.com/sharpa-robotics/sharpa-urdf-usd-xml) 标准双手模型及网格、[Sharpa Wave SDK](https://github.com/sharpa-robotics/sharpa-wave-sdk) 5.0.11。官方链接用于标明来源，不是额外安装步骤。许可证及 NOTICE 随资源原样保留。
- 真手套采集仍需要有效的 **Manus SDK-component 许可证**、已连接手套和适合当前操作者的标定。
- MuJoCo 交互窗口和原始关键点窗口需要图形显示环境；仿真可使用 `headless:=true`。

目录布局：

```text
sharpa_teleop/
├── pixi.toml / pixi.lock
├── vendor/                   # 三个压缩资源 wheel，随仓库分发
│   ├── sharpa_teleop_manus_resources-4.0.0-py3-none-linux_x86_64.whl
│   ├── sharpa_teleop_model_resources-1.0.0-py3-none-linux_x86_64.whl
│   └── sharpa_teleop_wave_resources-5.0.11-py3-none-linux_x86_64.whl
├── .pixi/envs/               # pixi install 生成，包含解包后的资源
└── calibration/              # 操作者私有标定，不随仓库分发
```

`pixi.toml` 将这些 wheel 声明为相对路径 PyPI 依赖，`pixi install` 自动解包到项目内 `default` 环境的 `site-packages`；运行入口通过激活变量使用安装后的目录，不需要安装钩子或 Git LFS。原始 Manus 集成库超过 100 MiB，压缩 wheel 避免将该大文件直接提交到 Git。`real` 不加载仿真模型，但会读取已安装 Manus 资源中的关节 URDF。

### 为什么有两个 Pixi 环境？

| 环境 | Python | 职责 |
| --- | --- | --- |
| `default` | 3.12 | ROS 2 节点、MuJoCo、原生 Manus 适配器构建与运行 |
| `retarget` | 3.10 | 官方重定向二进制和原版 IPOPT 求解器 |

上游重定向二进制依赖 Python 3.10 ABI，不能直接加载到 Jazzy 的 Python 3.12 进程中。项目通过隔离子进程连接两者，避免 ROS 的 `PYTHONPATH` 污染重定向环境，不替换官方数值求解器。

## 安装与构建

克隆本仓库（含 `vendor/`）后，在唯一的 `sharpa_teleop/` 根目录执行：

```bash
# 按 pixi.lock 安装 default 与 retarget 环境；不构建 ROS、不连接设备
pixi install --all --locked

# 构建 ROS 包
pixi run build

# 仅在使用真手套采集时构建原生 Manus 适配器
pixi run manus-build

# 检查 ROS、重定向库及硬件 SDK 的导入
pixi run doctor
```

不带参数的 `pixi install` 默认只安装 `default`，其中已包含三套资源；要同时准备官方重定向所需的 Python 3.10 `retarget` 环境，请使用 `--all`。`--locked` 要求清单与已提交锁文件一致，不更新锁定依赖。两套 Python 不会被合并，资源 wheel 只安装到 `default`，工作进程按项目内路径读取重定向资源。

`pixi install` 安装环境并解包本地资源，不会自动构建 ROS 或原生适配器，不启动 ROS，不发现、使能或移动设备，也不会下载或验证 Manus 许可证、手套连接或操作者标定。`doctor` 同样不发现、使能或运动设备；导入成功不代表许可证、手套连接、操作者标定或机械手通信就绪。

常用任务：

| 命令 | 用途 |
| --- | --- |
| `pixi run manus` | 构建并启动采集与重定向生产者 |
| `pixi run sim` | 构建 ROS 包并启动仿真消费者 |
| `pixi run real [left\|right]` | 构建 ROS 包并启动真机消费者 |
| `pixi run rawmanus` | 构建并启动原始关键点诊断窗口，默认同时启动采集 |
| `pixi run manus-build` | 单独构建原生 Manus 适配器 |
| `pixi run manus-native` | 单独运行原生采集适配器，供低层调试 |
| `pixi run record-speed-test` | 录制原始位姿和重定向关节目标 |
| `pixi run test` | 运行现有 Python 回归测试 |
| `pixi run help` | 查看命令帮助 |

原生适配器以 Release 模式构建，产物为 `build/manus-native/manus_ros`，直接包含和链接 `$SHARPA_MANUS_SDK/client/ManusSDK` 中的已安装资源。`sim`、`real` 不触发原生适配器构建；`rawmanus with_client:=false` 也不构建或校验 Manus SDK。

## 快速开始

### 1. 仿真遥操作

终端 1 启动采集与重定向：

```bash
pixi run manus
```

终端 2 启动仿真：

```bash
pixi run sim
```

无窗口运行：

```bash
pixi run sim headless:=true
```

`sim` 不会发现、使能或移动真实机械手，也不会生成演示动作。尚未收到有效命令时保持初始位置；收到命令后跟随最新有效目标。

仿真加载 `wave_01/dual_sharpa_wave/dual_sharpa_wave.xml`，保留厂商手指动力学、碰撞配置和位置执行器参数，固定未跟踪的腕部及头部自由度。默认物理步长为 `0.002 s`，积分器为 `implicitfast`，窗口同步为 30 Hz。关节和执行器按名称映射，限位使用 URDF、XML 关节及执行器范围的交集。模型资源中的网格路径在内存中解析，不修改资源文件。

### 2. 真机遥操作

启动前确认：

1. 机械手固定可靠、工作空间清空，实体急停可用。
2. 核对左右手对应关系和序列号。
3. 使用随仓库分发且与设备固件兼容的 Sharpa Wave SDK。
4. 完全退出其他控制程序，包括 Sharpa 控制软件的托盘后台 `pilot_sdk`，避免占用发现端口 UDP `54321`。

终端 1：

```bash
pixi run manus
```

终端 2，根据设备选择其中一条：

```bash
pixi run real         # 双手
pixi run real left    # 仅左手
pixi run real right   # 仅右手
```

命令默认使用以下设备，换设备时必须覆盖：

| 侧别 | 默认序列号 |
| --- | --- |
| 左手 | `C55C9039C55F` |
| 右手 | `CC549038CC57` |

```bash
pixi run real left left_serial:=LEFT_SN
pixi run real right right_serial:=RIGHT_SN
# 仅在有意覆盖随仓库 SDK 根目录时使用
pixi run real native_sdk_root:=/path/to/compatible/sharpa-wave-sdk
```

默认硬件 SDK 根目录由 `SHARPA_WAVE_SDK` 指向项目内已安装的 Wave 资源；普通克隆不需要 `/opt` 安装。

`real` 默认设置 `dry_run:=false`、`auto_enable:=true`、`return_to_zero_on_exit:=true`。双手模式等待两侧都有有效目标；单手模式只连接、校验并使能所选侧，忽略另一侧命令。单手命令不接受未选侧的非空序列号。

自动使能只尝试一次。显式停用或故障会取消等待，故障后不会自动重新使能。SDK 或反馈异常会触发对所有已选手的停用。

若只想检查目标校验而不连接或驱动真机：

```bash
pixi run real dry_run:=true
```

`dry_run` 不加载硬件后端、不发布实测反馈，也不模拟真实跟随动态；它不是机械手运动安全验证。当前 dry-run 按双手处理，自动使能等待双侧有效目标，即使命令中指定了 `left` 或 `right`。

### 3. 正常停机

先在 **`real` 终端**按 Ctrl+C，等待回零、停用和退出，再停止 `manus`。

正常 Ctrl+C/SIGTERM 时，健康且已使能的输出尝试用同一指数平滑回到全关节 `0 rad`，实测位置进入容差后停用。默认回零总时限为 10 秒、容差为 `0.02 rad`。零位不是自动标定，回零期间也必须保持工作空间清空。

**不要通过停止手套输入来停机。** 故障、SIGKILL、掉电或 SDK 阻塞时不能保证回零或电机已停用；紧急情况使用实体急停。

可选的调试/恢复服务：

```bash
# 显式停用，也会取消等待中的自动使能
pixi run ros2 service call /sharpa/enable std_srvs/srv/SetBool '{data: false}'

# 仅在排除故障、确认设备状态和工作空间后显式使能
pixi run ros2 service call /sharpa/enable std_srvs/srv/SetBool '{data: true}'
```

该服务不是标准启动步骤。设备发现等启动失败需要处理原因后重启输出进程，不能仅靠服务重新连接。

## 诊断与数据录制

### 查看原始关键点

```bash
pixi run rawmanus
```

该命令只启动采集与关键点窗口，不启动重定向、机械手动力学或真机输出。若 `manus` 已运行，使用订阅模式，避免第二个采集客户端：

```bash
pixi run rawmanus with_client:=false
```

窗口显示每手 25 个关键点、编号和彩色手指骨架：手根为点 `0`，拇指 `1–4`，食指 `5–9`，中指 `10–14`，无名指 `15–19`，小指 `20–24`。这里的“原始”是适配器发布的数据，已转为手根相对坐标并重排顺序，不是未修改的 SDK 世界坐标。

窗口使用 MuJoCo 渲染，不执行动力学；等待、实时和过期状态会区分显示。过期状态仅用于诊断，不控制机械手。关闭窗口或 Ctrl+C 会退出本次诊断组，不停止订阅模式下已有的外部生产者。

若人手伸直但机械手某些手指仍弯曲：先检查原始骨架。原始骨架已弯曲时优先检查佩戴和标定；原始骨架正常而重定向结果异常时，再检查映射与求解。

### 使用已有原始 ROS 发布者

```bash
pixi run manus with_client:=false
```

只启动重定向，不启动本项目采集适配器；该 Pixi 任务仍会执行其声明的构建依赖。外部发布者应在相同 ROS 域提供：

- `/manus/{left,right}/raw_poses` 上的 `geometry_msgs/PoseArray`，每手恰好 25 个位姿。
- 与适配器一致的关键点顺序和手根相对坐标，位置单位 m。
- 有限位置及可归一化的非零四元数。
- 正值、每手严格递增且不在未来的源时间戳。
- `frame_id` 使用 `manus_left_hand` 或 `manus_right_hand`；重定向保留该字段，但不会据此变换输入坐标。

### 录制 SpeedTest

先运行 `pixi run manus`，在另一个终端执行：

```bash
# Ctrl+C 正常停止并保存到当前目录 SpeedTest.HDF5
pixi run record-speed-test

# 或定时录制到另一个新文件
pixi run record-speed-test --seconds 30 --output session.HDF5
```

录制器只订阅，不发布指令、不使能硬件；无需启动 `sim` 或 `real`。已有文件不会覆盖。

HDF5 按 `left/`、`right/` 分组：

| 子分组 | 来源 | `values` 形状 |
| --- | --- | --- |
| `manus_raw` | `/manus/{side}/raw_poses` | `N × 25 × 7` |
| `sharpa_joint` | `/sharpa/{side}/command` | `N × 22` |

位姿列为 `x,y,z,qx,qy,qz,qw`，位置单位 m；关节角单位 rad，关节顺序存于 `joint_names` 属性。每组另存 `stamp_ns`、`received_ros_ns`、`elapsed_ns`，单位 ns，分别为源时间、接收 ROS 时间和相对录制起点的单调时间。

各话题独立采样，不按数组下标配对，可按源时间戳关联。`complete=true` 表示正常停止并完成缓存写入，不保证所有话题都有数据或没有丢帧。`sharpa_joint` 是输出端裁剪前的重定向目标，**不是实测反馈**。该工具不提供回放或实际跟随速度报告；历史文件中的 `manus_input` 属于旧重采样路径，新录制不生成它。

## 配置

### 路径与环境变量

| 环境变量 | 默认路径/值 | 用途 |
| --- | --- | --- |
| `SHARPA_MANUS_SDK` | `<default-site-packages>/sharpa_teleop_manus_resources` | Manus SDK、重定向库和关节 URDF |
| `SHARPA_MANUS_CALIBRATION_DIR` | `$PIXI_PROJECT_ROOT/calibration` | 操作者私有标定目录 |
| `SHARPA_WAVE_SDK` | `<default-site-packages>/sharpa_teleop_wave_resources` | 硬件 SDK |
| `SHARPA_MODELS` | `<default-site-packages>/sharpa_teleop_model_resources` | 仿真模型 |
| `RETARGET_PYTHON` | `$PIXI_PROJECT_ROOT/.pixi/envs/retarget/bin/python` | 重定向解释器 |
| `ROS_DOMAIN_ID` | `42` | ROS 通信域 |

`<default-site-packages>` 为 `$PIXI_PROJECT_ROOT/.pixi/envs/default/lib/python3.12/site-packages`。`pixi.toml` 为已安装资源、解释器和 ROS 域定义默认值；标准工作流不依赖兄弟目录。只有有意替换资源版本时才覆盖相应路径，并在变更 Manus SDK 后重新运行 `pixi run manus-build`，确保构建链接和运行加载的 SDK 匹配。

`calibration/` 用于本机操作者的私有标定，不能随仓库分发。操作者应将获授权的标定复制到该目录，或按 SDK 流程使用 SDK 自身标定。原生适配器从该目录读取 `Calibration_left.mcal`、`Calibration_right.mcal`；缺失文件会告警并保留 SDK 标定，但告警消失或保留 SDK 标定都不能等同于当前操作者已正确标定。

### Launch 参数与 YAML

参数在启动时读取，修改后需重启节点。

| 命令 | 主要 launch 参数 |
| --- | --- |
| `manus` | `config`、`sdk_root`、`calibration_dir`、`worker_python`、`with_client`、`project_root` |
| `rawmanus` | `sdk_root`、`calibration_dir`、`with_client`、`project_root` |
| `sim` | `config`、`models_root`、`headless`、`smoothing_time_sec`、`control_hz`、`feedback_hz` |
| `real` | `config`、`sdk_root`、`native_sdk_root`、`dry_run`、`auto_enable`、`return_to_zero_on_exit`、`homing_timeout_sec`、`homing_tolerance_rad`、`smoothing_time_sec`、`control_hz`、`left_serial`、`right_serial` |

示例：

```bash
pixi run manus calibration_dir:=/path/to/calibration
pixi run sim models_root:=/path/to/sharpa-urdf-usd-xml headless:=true
pixi run real right right_serial:=RIGHT_SN auto_enable:=false
```

默认 YAML 为 `src/sharpa_teleop/config/teleop.yaml` 和 `src/sharpa_teleop/config/sim.yaml`。**Launch 中映射的参数覆盖 YAML，launch 默认值也参与覆盖**；调整这些参数应显式传入 `<arg>:=<value>`，不能只修改 YAML。

`teleop.yaml` 的节点默认值是 `dry_run: true`、`auto_enable: false`、`return_to_zero_on_exit: false`，但标准 `real` 启动会覆盖为真机、自动使能和正常退出回零。`feedback_hz`、`startup_timeout_sec`、`future_tolerance_sec` 等未映射为 `real` launch 参数的输出节点设置通过 YAML 配置。

`sdk_root` 指向已安装的 Manus/重定向资源；`native_sdk_root` 才是已安装的硬件 SDK，二者不可混用。可通过以下命令查看启动参数，不启动运行节点：

```bash
pixi run sim --show-args
pixi run real left --show-args
```

### 通信域

所有参与通信的终端必须使用相同 `ROS_DOMAIN_ID`。每个域只应有一个手套生产者；实机域不要混入测试发布者或不受控输出节点。

默认值由 `pixi.toml` 的 `[activation.env]` 固定为 `42`。更换通信域时修改该值，再重新启动所有参与节点；仅在 `pixi run` 前加 `ROS_DOMAIN_ID=76` 会被当前激活配置覆盖，不应依赖这种写法。

## ROS 接口

以下 `{side}` 为 `left` 或 `right`：

| 接口 | 类型 | 含义 |
| --- | --- | --- |
| `/manus/{side}/raw_poses` | `geometry_msgs/PoseArray` | 每手 25 个手根相对关键点位姿 |
| `/sharpa/{side}/command` | `sensor_msgs/JointState` | 每手 22 个重定向关节目标，rad，保留输入时间戳 |
| `/sharpa/{side}/target` | `sensor_msgs/JointState` | 输出节点校验、裁剪后的目标，不是测量值 |
| `/sharpa/{side}/joint_states` | `sensor_msgs/JointState` | 真实硬件关节位置反馈，仅非 dry-run 输出 |
| `/sim/sharpa/{side}/joint_states` | `sensor_msgs/JointState` | MuJoCo 实际 `qpos/qvel`，单位 rad、rad/s |
| `/sharpa/enable` | `std_srvs/srv/SetBool` | 输出节点使能/停用服务 |

真机单手模式只订阅、发布所选侧接口。流式话题采用 **best-effort、volatile、keep-last**；通常深度为 1，重定向输入深度为 4。

```bash
pixi run ros2 topic list
pixi run ros2 topic echo /sharpa/right/command --qos-reliability best_effort
pixi run ros2 topic echo /sim/sharpa/right/joint_states --qos-reliability best_effort
```

链路使用 ROS 墙钟时间戳，仿真不发布 `/clock`，不要为这条链路开启 `use_sim_time`。跨主机运行时需保持时钟同步，未来时间戳默认拒收。

## 控制策略与安全边界

仿真与真实输出统一使用“最新目标 + 指数平滑”：

```text
q_next = q_prev + (1 - exp(-dt / tau)) * (target - q_prev)
```

- 默认控制循环为 500 Hz，`tau = smoothing_time_sec = 0.02 s`；`dt` 使用实际经过的单调时间。
- 新目标立即替换旧目标，不回放历史轨迹。20 ms 时间常数约需 60 ms 接近阶跃变化量的 95%，这不是实机跟随延迟保证。
- 不限制固定速度、步长、加速度或 jerk。较大角度变化会产生较大速度，指数平滑不是安全限速。
- 没有输入 Watchdog，也不因消息年龄拒收。格式有效且时间戳递增的旧目标仍可接受；断流不会停用输出。
- 检查关节数量、URDF 名称顺序、有限值和源时间戳；有限目标超限时裁剪。拒收 NaN/Inf 关节值，以及非正值、重复、乱序或默认不允许的未来时间戳。
- 硬件实测反馈和最终 SDK 下发边界严格检查限位，不裁剪反馈来绕过异常。
- 真机连接关闭 SDK 的设备校时和触觉初始化：`disable_sync_time=true`、`disable_tactile=true`；本项目是关节遥操作，不提供触觉反馈。
- SDK/反馈故障触发停用，正常退出回零仍有超时和实测容差检查；这些时限不是输入 Watchdog。

500 Hz 是主机侧循环目标，不代表硬件伺服频率、网络发包频率或硬实时保证。本项目不是安全认证控制器；SDK 阻塞、网络故障、进程强制终止和掉电可能使软件无法完成停用，现场必须有急停和监护。

## 常见问题

### `No compatible license found`

这是 Manus 授权问题。检查许可证是否包含 SDK-component 权限，并按厂商流程连接和标定手套。不要同时运行其他 MANUS Core / Core Integrated 实例。

### 仿真没有动作，或窗口无法打开

确认生产者在运行，两端 ROS 域一致，原始位姿与关节目标话题有数据。`sim` 自身不生成动作；无显示环境时使用 `headless:=true`。真实反馈与仿真反馈是不同话题，不能互相替代。

单独用符合接口约束的关节发布者测试 `sim` 不需要 Manus 许可证；标准手套工作流仍需要。

### `cannot arm: timed out ... waiting for selected HAND device(s)`

检查指定序列号、左右手、供电和网段，再检查发现端口：

```bash
ss -ulpn 'sport = :54321'
```

若端口被 `pilot_sdk` 或其他控制程序占用，完全退出它们后重启 `real`。启动发现失败不会仅靠 `/sharpa/enable` 重新连接；不要关闭关节限位检查绕过问题。

### `HeartPacket Unsupported protocol version: 0302`

这是硬件 SDK 与设备固件的协议兼容性问题。使用随仓库分发且与设备固件兼容的 Wave SDK，或有意以 `native_sdk_root` 覆盖为获授权的兼容版本；不要覆盖重定向库或绕过协议检查。`pixi run doctor` 会显示实际导入的硬件 SDK 路径。

### NumPy / 重定向库 Python 版本错误

执行 `pixi install --all --locked` 和 `pixi run doctor`。不要在 Python 3.12 ROS 进程直接加载 Python 3.10 重定向二进制，也不要混用系统 ROS 或其他 Conda 环境的 Python 路径。

### 官方求解器诊断与退出警告

官方 IPOPT 可能出现 `nlp_f` 的 Inf 诊断，重定向工作进程退出时也可能报告 shared-memory `resource_tracker` 警告。检查求解结果和工作进程错误，不应通过隐藏诊断、伪造输出或关闭校验来绕过问题。一次成功运行不证明所有姿态的优化稳定性。

## 项目结构

```text
pixi.toml / pixi.lock              环境、任务与依赖锁文件
vendor/
  *.whl                          固定版本 SDK、重定向及模型资源轮包
calibration/                       操作者私有标定，不随仓库分发
native/
  CMakeLists.txt                  原生适配器构建
  manus_ros.cpp                   Manus SDK 生命周期与 raw_poses 发布
  manus_pose.hpp                  关键点顺序、坐标与旋转转换
  test_manus_pose.cpp             原生位姿转换检查
scripts/
  workspace.py                   构建、启动、帮助与依赖检查
  manus_client.py                原生适配器构建与运行
  record_speed_test.py            HDF5 录制
  package_vendor.py              维护者重建授权资源轮包；安装时不执行
src/sharpa_teleop/
  launch/                        生产者、诊断、仿真和真机启动文件
  config/                        默认节点配置
  sharpa_teleop/
    retarget.py                  ROS 与独立工作进程桥接
    retarget_worker.py           官方重定向库适配
    raw_manus.py                 原始关键点诊断窗口
    mujoco_sim.py                仿真 ROS 接口与窗口
    sim_model.py                 随仓库模型加载与动力学
    sharpa_output.py             真机输出、dry-run 与启停生命周期
    safety.py                    校验和指数平滑
  setup.py / package.xml         ROS 包元数据与节点入口
tests/                           现有 Python 回归测试
```

## 验证与许可

可运行 `pixi run test` 检查现有安全边界、输出启停、重定向和仿真回归；通过测试或 `doctor` 不等于完成实机安全验证。真机延迟、网络下发频率、全关节运动范围和长期稳定性需要在具体设备与现场条件下测量。

项目内依赖集成已用全新临时目录验证：仅复制源代码、锁文件和资源 wheel，不复制已有环境、构建产物或个人标定，`pixi install --all --locked`、ROS 构建及 `doctor` 均成功。隔离 ROS 域中的官方 IPOPT 双手求解和实际 headless MuJoCo 目标跟随通过；现有 22 项回归测试通过，原生采集适配器重新构建成功。该验证未启动真机输出，不代表已验证新主机的手套许可证或实机运动。

本仓库维护方已确认三套 `vendor/` 资源可随本仓库分发。该确认不改变任何接收者的授权边界：接收者仍必须遵守各 `vendor/` 资源原始的 `License`、`LICENSE.txt` 与 `NOTICE.txt`，并保留其中已有的许可和通知。个人操作者标定不随仓库分发；获得授权的标定只能由相应操作者按其许可使用。本项目源代码与上游资源的授权范围不同，ROS 包元数据标记为 `Proprietary`，不能将本项目或 `vendor/` 资源默认视为可自由再分发。

### 维护者更新资源

普通用户只需安装已提交的 wheel，不执行打包脚本。维护者获取新的授权原始资源后，可重建快照：

```bash
pixi run package-vendor --manus-sdk /path/to/authorized/sharpa-manus-sdk \
  --models /path/to/sharpa-urdf-usd-xml --wave-sdk /path/to/authorized/sharpa-wave-sdk
```

打包脚本保持厂商文件内容及相对布局，不带入 `.git`、Python 缓存、个人标定或旧版嵌套硬件 SDK。Manus 包保留本项目所需的头文件、Integrated 库、官方求解器、URDF 及其网格；模型包按标准双手 XML 和 URDF 的引用收集网格；Wave 包保留原始 SDK 分发包，排除生成缓存。

更新快照时同时调整打包脚本中的资源包版本、`pixi.toml` 对应 wheel 路径，并执行 `pixi install --all` 更新锁文件；将三个 wheel、清单、锁文件及打包脚本一起提交。删除已被替代的旧 wheel，避免安装源并存。资源轮包版本用于固定项目快照，不能代替厂商 SDK 自身的版本信息和授权。
