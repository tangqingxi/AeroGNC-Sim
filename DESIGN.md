# AeroGNC-Sim 详细设计（DESIGN）

> 本文是长期技术设计说明：模块边界、接口约定、设备参数治理与扩展规则。
> 顶层架构基线以 [`ARCHITECTURE.md`](ARCHITECTURE.md) 为准；本文不重复其冻结决策，
> 只把已确认的设计思想展开为可执行约定。协作与代码规范见 [`AGENTS.md`](AGENTS.md)。

## 1. 项目愿景与使用方式

AeroGNC-Sim 是模块化 MATLAB GNC 科研仿真平台。研究者通过**选择与组合**模块构建实验：

```matlab
aero_setup();

cfg = aero_default_config();
cfg.tspan = [0 10];
cfg.x0 = 初始状态;
cfg.initial_command = 初始指令;      % 数值，或字段均为数值的结构体

model = shc09_model(options);        % Aircraft 型号（含 Components 装配）
result = aero_sim(model, cfg);       % 统一仿真内核
```

目标：更换机型、设备型号、制导/导航/控制算法时，原则上不修改 `Simulation` 核心。
SHC09 是第一个完整机型实例，不是平台的唯一对象。

## 2. 模块功能边界

| 模块 | 负责 | 不负责 |
|---|---|---|
| `src/Simulation/` | 虚拟时钟、事件调度、分段 `ode45`、指令锁存、测量发布、Topic 通信、日志、结果持久化 | 任何机型气动、设备型号、控制算法 |
| `src/Aircraft/<TYPE>/` | 整机装配、质量惯量、几何、气动、完整动力学、状态定义、标准输出 | 通用设备动态、控制算法、场景 |
| `src/Components/` | 可复用设备型号参数与通用设备动力学 | 安装位置、通道编号、机型专属效能、控制器参数 |
| `src/GNC/` | Guidance、Navigation 与四级 Control（Position/Velocity/Attitude/Rate）+ Allocation | 物理对象、场景 |
| `src/Scenarios/` | 任务、参考条件与组件装配 | 物理模型、控制算法 |

## 3. Aircraft 与 Components 的组合关系

```text
Aircraft 装配（shc09_model）
   ├── 选择设备型号：ServoDevices.KST_X08H / MotorDevices.MAD_M6C06_320KV_V2
   ├── 装配参数：质量、惯量、舵面几何、力臂、通道顺序、PWM/锁存频率
   ├── 气动模型：涵道、机翼、六舵面、固定片、陀螺
   ├── 通用设备动力学：aero_servo_dynamics / aero_motor_dynamics
   └── 输出：model.dynamics / model.project / model.outputs
```

组合原则：

- 设备型号只提供设备自身参数；Aircraft 决定用哪个型号、装在哪里、编号与方向如何。
- 通用动力学只接收参数，不读取文件、不判断品牌、不引用 SHC09。
- 新增机型时复用同一套 Components；新增设备型号时只加参数文件夹与装配选择。

## 4. 设备型号参数库

目录使用 MATLAB 原生包（`+` 文件夹）组织，避免注册器与全局搜索：

```text
src/Components/Actuators/Servo/+ServoDevices/+KST_X08H/parameters.m
src/Components/Actuators/Motor/+MotorDevices/+MAD_M6C06_320KV_V2/parameters.m
```

调用：`ServoDevices.KST_X08H.parameters()`。

每个型号文件记录：名称、类别、动力学 `model_type`、固有规格与单位、可用动态参数、
来源与可信等级、必要的物理限制。**不允许**记录 SHC09 安装位置、CS1–CS6 编号、
PWM 中值、单台舵机方向或控制器增益。

## 5. 设备参数模板（复制即用）

```matlab
function p = parameters()
%PARAMETERS <厂商> <型号> 设备参数
p = struct();
p.device_name = '<厂商> <型号>';
p.device_class = 'servo';          % 'servo' | 'motor'
p.model_type = 'first_order';

p.tau_s = 0.03;                    % 一阶时间常数，s
p.rate_limit_rad_s = deg2rad(600); % 舵机角速率上限，rad/s（电机用 accel_limit_rad_s2）
p.range_rad = [-Inf, Inf];         % 设备行程能力；装配限制（如 ±40°）不写在这里
% 电机型号额外声明：omega_range_rad_s（运行范围）、omega_stop_rad_s=0、stop_threshold_rad_s

p.provenance = struct( ...
    'tau_s', '仿真假设 / 实测 / 厂家规格', ...
    'rate_limit_rad_s', '仿真假设 / 实测 / 厂家规格', ...
    'range_rad', '来源说明');
p.notes = '替换为实测值无需修改通用动力学。';
end
```

## 6. 通用执行器动力学与 mode 扩展

一阶模型（本轮唯一实现）：

```text
舵机：δ̇ = clamp((δ_cmd − δ)/τ, ±ω_max)，限位处导数置零
电机：Ω̇ = clamp((Ω_cmd − Ω)/τ, ±a_max)，目标先按转速范围限幅
```

`model_type` 必须显式声明；V0.1 只实现 `'first_order'`，其他值或缺失时通用动力学立即报错
（`aero:components:modelType`），不得静默退化为其他模型。后续 `second_order`、`identified`、
`nonlinear` 在真实需要时再实现，不预建空文件。不同 mode 可以有不同状态维数，由装配决定。

**电机停机语义**：`omega_range_rad_s` 的下限（200 rad/s）是最低**运行**转速，不是物理下限。
停机指令（`command ≤ stop_threshold_rad_s`，默认 1 rad/s）的目标是物理停机转速 0，
允许真实零转速保持；启动指令再从 0 加速到运行范围。状态投影只限制 `[0, 最大转速]`，
不得把停机状态抬到 200 rad/s。

设备型号与动力学 mode 相互独立：同一舵机可用不同精度模型，同一模型可服务不同设备。

## 7. 六维舵面与独立电机指令

**七维统一控制向量已废弃。** 指令由两条独立信号组成：

```matlab
command.servos = zeros(6,1);   % CS1..CS6 目标偏角，rad
command.motor  = 0;            % 旋翼目标转速，rad/s
```

- Simulation 只做通用校验、锁存与传递，**不识别** `servos` / `motor` 字段名，也不预设字段数量。
- 结构体命令要求：单层结构，所有字段为非空有限实数；数值命令接口保持兼容。
- 舵面与电机可以有不同的更新周期与锁存频率（SHC09 装配：舵面 200 Hz、电机 50 Hz）。
- 更新一条指令不覆盖另一条：组件只写自己的字段，锁存组件只复制自己的字段。
- 指令解耦不代表物理解耦：六舵面效能依赖出口流速 `Ve`，`Ve` 又由旋翼推力决定，
  这一耦合只存在于 `shc09_forces_and_moments` 的物理计算中。

## 8. Simulation 与 Aircraft 接口

```matlab
model.name        = 'SHC09';
model.x0          = 20×1 初始状态;
model.dynamics    = @(t, x, command, env) dx;
model.project     = @(x) xp;        % 可选：四元数归一化/执行器限幅
model.outputs     = @(t, x, env) truth;
model.state_layout, model.parameters, model.aero, model.initial_command
```

- `dynamics` 必须无副作用：不修改输入、不读写文件、不消耗随机数、不更新控制器。
- `project` 只做数值约束（四元数范数、行程、转速），由内核在每个积分段后调用。
- `metadata`（可选）由机型提供，内核原样写入 `result.model_metadata`；内核不解释字段。
  SHC09 记录模型版本、质量/惯量及来源、设备型号与动态参数、生效行程、覆盖值和
  外部气动数据文件 SHA256（可核查标识）。
- 结构体命令在运行期由内核通用校验：组件每次更新后检查 `bus.command` / `bus.held_command`
  必须为有限数值结构；非法值立即报 `aero:kernel:command`。
- `env` 可选字段：`wind_ned_m_s`、`force_ned_n`、`torque_body_nm`。
- 内核不解释状态含义；`state_layout` 是 SHC09 自己的定义。

## 9. SHC09 已冻结的几何与物理方向

- 机体系 FRD：+X 机头、+Y 右侧、+Z 涵道轴向下，原点在重心；惯性系 NED。
- 质量 `m = 2.1 kg`；惯量 `J_frd = diag([0.0438, 0.0436, 0.005006]) kg·m²`
  （Gazebo/PX4 FRD 参考值，未实机辨识）。
- 六舵面方位角 0/60/120/180/240/300°，`r_i = [l2 cosφ; l2 sinφ; l1]`，
  正向旋转轴 `a_i = [cosφ; sinφ; 0]`，实机接线编号与 CS1–CS6 一致。
- `l1 = 0.267 m`（轴向力臂）、`l2 = 0.066 m`（偏航力臂）。
- 力矩效能（无量纲）：

```text
B_M = [-1, -1/2, 1/2, 1, 1/2, -1/2;
        0, -√3/2, -√3/2, 0, √3/2, √3/2;
        1,  1,    1,   1, 1,    1]
B_tau = d_cs · Ve² · diag([l1, l1, l2]) · B_M
B_acc = J_frd⁻¹ · B_tau
```

  三者不得混用。`B_M` 可由几何导出：第 i 列 = `diag([l1,l1,l2])⁻¹ (r_i × f_i)`，
  其中正偏角气动合力方向 `f_i = [-sinφ_i; cosφ_i; 0]`。
- 旋翼正转：绕 +Z 右手正向，推力沿 −Z，空气动力反扭矩沿 −Z；固定片反扭矩反向补偿。
- 舵机：KST X08H，8 V，装配行程约 ±40°（可配置为非对称上下界），实机 PWM 更新率 200 Hz。
- 舵面真实偏角、旋翼真实转速是连续状态，由通用动力学推进；目标指令是锁存输入。

## 10. 参数归属、单位与可信等级

| 参数 | 单位 | 归属 | 可信等级 |
|---|---|---|---|
| 质量、惯量 | kg、kg·m² | Aircraft 装配 | 参考值，未辨识 |
| 力臂、方位角 | m、rad | Aircraft 装配 | 实机定义 |
| 舵机设备能力 `range_rad` | rad | 设备型号（KST 未声明，取 `[-Inf, Inf]`） | 未声明 |
| 舵面装配行程 [lower upper] | rad | Aircraft 装配 `servo_deflection_limit_rad`（可非对称） | 实机定义 |
| **生效行程** = 能力 ∩ 装配 | rad | Aircraft 计算一次 `servo_effective_range_rad`，作为唯一 `range_rad` 传给通用动力学 | 唯一权威 |
| `B_M` | — | Aircraft 几何 | 实机接线与几何 |
| 涵道/机翼系数 | SI | Aircraft 气动数据 | **外部数据，许可待确认** |
| 舵机 τ、速率 | s、rad/s | 设备型号 | 名义参数 |
| 电机 τ、转速范围 | s、rad/s | 设备型号 | 名义参数 |
| PWM/锁存频率 | Hz | Aircraft 装配 | 实机配置 |
| 控制器参数 | — | Control（未实现） | — |

实验覆盖通过 `shc09_parameters(overrides)` 或 `shc09_model(struct('overrides', ...))`，
覆盖内容记录在 `p.overrides` 并随结果保存。

## 11. 扩展、目录治理与禁止事项

- 新增机型：`src/Aircraft/<TYPE>/`，实现 `model` 装配与动力学，复用 Components。
- 新增设备：只增加 `+ServoDevices/+<MODEL>/parameters.m` 或 `+MotorDevices/+<MODEL>/parameters.m`，
  并在装配中选择；一般不改通用动力学。
- 新增算法：在 `src/GNC/{Guidance,Navigation,Control}` 下按需创建，不在本轮预建。
- `aero_setup.m` 只显式登记已实现模块目录，禁止递归 `addpath`。
- 禁止平行目录（`models/`、`core/`、`physics/`、第二套 `simulation/`）；
  禁止把设备型号、机型动力学、控制器写在同一文件；禁止为兼容历史保留两套同功能实现。
- 气动原始表不得在许可确认前进入仓库；只能通过外部数据接口加载。

## 12. 已实现与未实现边界

**已实现**

- Simulation 内核（分段 `ode45`、多速率事件、锁存、测量延迟、结果与容量策略）。
- 通用结构体命令容器（`command` 可为数值或有限数值字段结构体）。
- Servo/Motor 通用一阶动力学（含 `model_type` 校验、速率/行程/转速限幅、停机与启动语义）
  与两个设备型号参数。
- SHC09 20 状态非线性六自由度模型：刚体、涵道、机翼、六舵面、陀螺、反扭矩、风扰。
- 悬停配平（不依赖气动表，检查非对称生效行程上下界）、几何/效能/动力学/执行器/停机状态/
  指令独立性/输出语义/元数据/集成/目录治理测试。
- Communication：Latest-Value Topic Bus（注册、发布、读取、契约校验、可用/过期/时间倒退处理、
  真值隔离），V1.0 始终启用；结果快照、有界发布日志与边界校验已补齐。
- GNC-Baseline：Position P、Velocity PID、Attitude P、Rate PID 四级独立控制器 + 独立 PINV
  分配；两个闭环场景与 WTR 导出。全项目共 38 项测试（详见 §15）。

**未实现 / 依赖外部条件**

- SHC09 真实气动系数表：再分发许可未确认，真实表不入库；物理气动验证依赖用户提供数据文件。
  外部文件默认 `trust='unverified'`，只有显式 `verified` 才允许物理验证；合成系数固定 `synthetic`。
- 高级 GNC（INDI、MPC、复杂分配）、真实传感器模型与传感器噪声、EKF2/ECL、场景库扩展。
- 二阶/辨识/非线性执行器 mode。
- 实机参数辨识（惯量、舵机 τ/速率、电机 τ/转速范围仍为参考值或名义值）。

**结论边界**：本轮验证的是“方程实现、坐标与符号、数值积分、执行器动态与接口”，
不是“实机参数精度”。不得把与参考模型一致表述为实机精度认证。

## 13. 当前目录布局与文件归属

```text
AeroGNC-Sim/
├── ARCHITECTURE.md   顶层基线（含全项目目录治理条款）
├── DESIGN.md         本文
├── AGENTS.md         Agent 协作与代码规范
├── README.md         用户文档
├── aero_setup.m      生产路径登记（不加载 tests）
├── src/
│   ├── Simulation/
│   │   ├── aero_sim.m             运行入口
│   │   ├── aero_integrate.m       连续积分（单文件，不另建目录）
│   │   ├── Events/                调度、测量、组件 schema
│   │   ├── Configuration/         默认配置、配置校验、命令校验
│   │   ├── Communication/         Topic 总线与数据契约
│   │   ├── Results/               保存、容量、路径安全
│   │   └── Internal/              环境信息、项目根
│   ├── Aircraft/SHC09/
│   │   ├── shc09_model.m          装配入口
│   │   ├── shc09_parameters.m     唯一参数来源
│   │   ├── shc09_dynamics.m       六自由度方程
│   │   ├── shc09_outputs.m        标准输出
│   │   ├── shc09_hover_equilibrium.m 悬停平衡点
│   │   ├── Aerodynamics/          涵道、机翼、舵面、系数表、力/力矩合成
│   │   └── Internal/              状态布局、投影、四元数、元数据、命令解析
│   ├── GNC/
│   │   ├── Guidance/              hover_hold、body_rate_step
│   │   ├── Navigation/            ideal_state（IdealState 适配器）
│   │   └── Control/
│   │       ├── Position/          position_p
│   │       ├── Velocity/          velocity_pid
│   │       ├── Attitude/          attitude_p
│   │       ├── Rate/              rate_pid
│   │       ├── Allocation/        pinv_allocator
│   │       └── Internal/          control_defaults、topic_specs、四元数/推力辅助
│   ├── Scenarios/                 hover_scenario、rate_step_scenario
│   │   └── Internal/              build_config、experiment_signature、ideal_sensor_sample、latch_*
│   └── Components/Actuators/{Servo,Motor}/
│       ├── aero_*_dynamics.m      通用动力学（不绑定品牌）
│       └── +<Class>Devices/+<MODEL>/parameters.m
├── tools/wtr/                     WTR 离线导出与独立校验（不属于 Simulation）
├── tests/
│   ├── run_tests.m                唯一入口（登记表与磁盘一致性检查）
│   ├── helpers/                   断言与测试辅助
│   ├── fixtures/                  测试替身（生产路径不加载）
│   ├── Simulation/{Core,Events,Communication,Results}/
│   ├── Aircraft/SHC09/
│   ├── GNC/                       四级控制器单元、链路、多速率、两类场景
│   ├── Export/WTR/                导出契约、锁存时间轴与覆盖保护
│   └── test_project_layout.m      目录、入口、重复定义检查
├── examples/
└── results/                       gitignore
```

### 13.1 新文件放置决策

| 新内容 | 放置位置 |
|---|---|
| 运行级能力（时钟、调度、积分） | `src/Simulation/` 或其职责子目录 |
| 新配置项/校验规则 | `src/Simulation/Configuration/` |
| 结果存储/容量/路径逻辑 | `src/Simulation/Results/` |
| Topic 总线/数据契约 | `src/Simulation/Communication/` |
| 新机型 | `src/Aircraft/<TYPE>/`：入口、参数、动力学、输出放根，物理子目录按领域 |
| 机型内部辅助（状态索引、投影、元数据） | `src/Aircraft/<TYPE>/Internal/` |
| 新设备型号 | `src/Components/Actuators/<Class>/+<Class>Devices/+<MODEL>/parameters.m` |
| 新设备动力学类别 | `src/Components/Actuators/<Class>/` 下的通用函数 |
| Guidance 参考生成 | `src/GNC/Guidance/` |
| Navigation 估计适配器 | `src/GNC/Navigation/` |
| 控制律/分配 | 四级控制分别放 `src/GNC/Control/{Position,Velocity,Attitude,Rate}/`；`Allocation/` 独立；纯数学辅助放 `Control/Internal/` |
| 场景装配 | `src/Scenarios/`，共享构建放 `Scenarios/Internal/` |
| 离线导出工具 | `tools/wtr/`（不得被 Simulation 依赖） |
| 新测试 | `tests/Simulation/{Core,Events,Results}/`、`tests/Aircraft/<TYPE>/`、`tests/GNC/` 或 `tests/Export/` |
| 测试替身 | `tests/fixtures/` |

### 13.2 依赖方向

`Aircraft → Components + Simulation 接口`；`Components` 不依赖 `Aircraft`；
`Simulation` 不依赖 `Aircraft`/`Components`；`tests` 可依赖全部；`examples` 只依赖生产代码。

### 13.3 新增或移动文件后的自检

- `aero_setup.m` 是否登记新目录（禁止 `genpath`，不登记 tests）；
- `run_tests.m` 的登记表是否与磁盘 `test_*.m` 一致；
- `which -all` 是否仍唯一，旧名称是否已不可解析；
- 是否产生重复实现、空目录或按轮次命名的目录；
- README/DESIGN/AGENTS 是否同步。

## 14. Communication Topic Bus（V1.0）

定位：Guidance / Navigation / Control 等模块通过 Topic 名称交换数据，不读取彼此内部状态。
V1.0 中 Bus 由 `aero_sim` 创建并始终启用，是唯一的运行时通信状态。

### 14.1 接口

| 接口 | 语义 |
|---|---|
| `bus = aero_topic_bus()` | 创建总线（handle 对象）；不启动时钟或线程 |
| `bus.register(name, spec)` | 注册 Topic 与数据契约；重名、非法名称、真值命名违规立即报错 |
| `bus.publish(name, data, sample_t, available_t, now, source)` | 发布最新值；`now` 必填，且 `now >= available_t`，禁止提前发布未来消息 |
| `msg = bus.read(name, now)` | 严格读取：未注册/未发布/尚不可用/过期分别报错 |
| `[msg, status] = bus.tryRead(name, now)` | 非抛出式读取，`status` 为 `ok`/`unknown_topic`/`missing`/`not_available`/`stale` |
| `msg = bus.latest(name, 'diagnostic')` | 仅诊断；必须显式传 `'diagnostic'`，正式消费者应使用 read/tryRead |
| `bus.enableLog(topics, maxRecords)` | 可选的发布日志（默认关闭），只记录显式 Topic 的实际发布；拒绝真值 Topic；总量上限防止无限增长 |
| `snap = bus.logSnapshot()` / `aero_communication_snapshot(bus)` | 返回普通值快照（日志 / 各 Topic 最后消息），不含 handle |
| `bus.topicNames()` / `bus.getSpec(name)` / `bus.sequence()` | 注册列表、契约与全局发布序号 |

契约由 `aero_topic_spec(fields, dimensions, units, options)` 构造，声明字段名、尺寸、单位、
是否允许真值（`allow_ground_truth`）与最大消息年龄（`max_age_s`）。

### 14.2 消息结构

```text
message.name               Topic 名称
message.data               载荷结构体（字段与契约一致）
message.sample_time_s      采样/来源时刻
message.available_time_s   对订阅者可用的时刻（≥ sample_time_s）
message.sequence           全局发布顺序
message.topic_sequence     该 Topic 的独立版本号
message.source             可选的来源标识
```

### 14.3 规则

- **Latest-Value**：每个 Topic 只保存最新值，不建历史队列。
- **时间**：同一 Topic 的 `sample_time_s` 不得倒退；相等允许，后者覆盖并递增序号。
  同一时刻连续发布的最终值确定（最后发布者胜出）。
- **可用性**：`read` 在 `now < available_time_s` 时返回 `not_available`，不泄露"未来"数据。
- **过期**：契约 `max_age_s` 有限时，`now - sample_time_s` 超过即 `stale`。
- **契约校验**：载荷必须是标量结构体，字段集合与契约一致，数值实数、尺寸匹配、全部有限；
  单位只声明不换算。
- **真值隔离**：普通 Topic 不允许承载真值；只有以 `truth/` 开头且声明 `allow_ground_truth=true`
  的 Topic 才能发布真值；普通 GNC Topic 命中真值标志时直接报错。
  **"truth/" 前缀只是命名约定，不是访问控制**：真实状态仍只允许 Sensor 回调与明确授权的
  测试适配器读取，生产 GNC 不得读取真值。
- **无自有时间/调度**：总线不创建 Timer、线程、网络或文件 I/O；时间由 Simulation 传入。

### 14.5 与 `aero_sim` 的集成（V1.0 单一 Bus）

- `aero_sim` 始终创建并持有**唯一** `aero_topic_bus`，作为 `bus.topics` 提供给组件；
  不存在 `communication.enabled` 关闭模式，也不维护第二套消息状态。
- Topic 由 `cfg.communication.topics`（cell of `struct('name',…,'spec',…)`）显式注册；
  `command_topic` 与 `held_command_topic` 必须指向已注册 Topic。
- Sensor：`topic` 为必填字段；在**发布事件**（测量真正可用）时发布
  `struct('value', v(:), 'sample_time_s', sampleTime)`，`available_time` 等于当前事件时刻，
  因此不会提前发布未来样本。运行时不再有 `bus.measurements` 共享数组。
- 命令：内核在 `bus.command` / `bus.held_command` **发生变化**时分别发布到
  `command_topic` / `held_command_topic`（数值命令打包为 `struct('value', cmd(:))`，
  结构体命令原样发布）。两者 Topic 分离，锁存频率与计算频率可以不同，原有锁存逻辑不变；
  GNC 模块之间应通过 Topic 交换参考/估计/指令，不得把 `bus.command` 当作通用数据通道。
- `result.measurements` 是测量的**唯一权威记录**（内核在发布事件追加，随仿真时间增长，
  不设 `max_records` 上限）；`result.communication` 是**普通值快照**（`aero-comm-snapshot/1`：
  Topic 列表、发布计数、每个 Topic 最后一条消息、可选日志），**不保存 Bus 句柄**，可直接 `save/load`。
  Topic 日志只是有界分析副本（受 `max_records` 限制），不作为第二份权威数据。
- 可选日志：`cfg.communication.log = struct('enabled',false,'topics',{{}},'max_records',10000)`；
  只记录所选 Topic 的**实际发布**（名称/载荷/采样时间/可用时间/全局与 Topic 序号），
  超过 `max_records` 的发布计入 `dropped` 而不增长；拒绝真值 Topic；开关日志不改变轨迹与时序。
- 启动校验：命令 Topic 与锁存命令 Topic 必须不同；传感器 Topic 不得重复、不得与命令 Topic 冲突；
  运行时组件不得删除或替换 `bus.topics`（唯一实例检查）。
- 内核不解释 Topic 字段名，不为 SHC09 或任何控制算法硬编码契约。

### 14.6 已知限制

1. 仅 Latest-Value；日志只用于分析与离线核查，不是历史消息队列，也不提供回放/逐样本订阅。
2. 日志容量由 `max_records` 限制；超出部分只计数不保存，长时高频仿真需自行设置容量。
3. 契约只校验结构、尺寸与有限性，不校验物理合理性（量程、单位换算由模块负责）。
4. `truth/` 前缀不是权限系统；真实隔离依赖 Simulation 的数据边界与模块纪律。
5. 仅在 MATLAB R2025b 下验证了快照 `.mat` 保存/加载；不承诺跨 MATLAB 版本兼容。
6. 传感器采样必须有 Topic 契约；V1.0 不支持“无 Topic 的旁路测量”。

## 15. GNC 四级控制器与 WTR 导出

参考：`mengchaoheng/DuctedFanUAV-Autopilot` @ `2eb3366757…`（`df-main`）与
`tangqingxi/Ducted-Fan-SHC09-matlab` @ `43b6c7ae…`、WindTrace @ `bcdb6a8e…`。
PX4 归一化力矩/推力增益不直接搬用；平台控制量全部使用 SI 单位。

### 15.1 四级独立目录与职责

```text
src/GNC/
├── Guidance/            hover_hold、body_rate_step（只给参考，不做追踪）
├── Navigation/          ideal_state（IdealState 测试适配器，非 EKF）
└── Control/
    ├── Position/        position_p     位置 P → 速度参考
    ├── Velocity/        velocity_pid   速度 PID → 姿态/推力
    ├── Attitude/        attitude_p     姿态 P → 机体系角速度参考
    ├── Rate/            rate_pid       角速度 PID → 力矩需求
    ├── Allocation/      pinv_allocator 六舵面 PINV + 电机推力
    └── Internal/        control_defaults、topic_specs、四元数/推力辅助
src/Scenarios/            hover_scenario、rate_step_scenario（仅装配）
```

命名规则：目录表达领域，文件名表达职责和算法；不再使用 `gnc_` 前缀。
四级控制器是四个独立组件，各自拥有参数、状态、频率与单元测试；Allocation
独立可替换，不得把四级数学重新合并到一个函数。

### 15.2 控制律与单位

- **Position P**：`v_ref = v_ff + Kp_pos·(p_ref − p_est)`；水平限幅按 PX4
  `constrainXY` 语义优先保证位置反馈方向，垂直方向分别限制上升/下降；速度限幅
  参数归 Position，无积分状态。
- **Velocity PID**：`a_cmd = a_ff + Kp_vel·e_v + I_vel − Kd_vel·a_est`；
  `a_est` 来自 `navigation/state.acceleration`（测量差分，非 Aircraft 真值）；
  `Kd` 为无量纲（输出与输入均 m/s²）。垂直积分在 PID 前钳位到 ±g；垂直推力
  到上/下限时按 PX4 语义冻结对应误差；水平方向使用 Tracking Anti-Windup
  （`2/Kp_x`）；总推力、最小推力、水平推力余量和最大倾角共同约束控制需求。
- **Attitude P**：PX4 默认 reduced-attitude 误差（`quat_error_default`），
  `yaw_weight` 缩放偏航误差并在偏航增益侧补偿；`ω_ref = Kp_att·e + ω_yaw_rate
  + ω_ref_model`；`ω_yaw_rate` 由世界系 yaw_rate 旋转到参考机体系；参考模型为
  PX4 默认二阶临界阻尼 ZOH 形式（`MC_REF_W_N=50 rad/s`，`MC_REF_FF=0`），
  输出不再混入角加速度；最后按 `rate_limit` 逐轴限幅。
- **Rate PID**：`τ = Kp_rate·e_rate + I_rate + Kff·ω_ref − Kd_rate·α_est`；
  **不做控制器本地力矩限幅**，执行器约束由 PINV 负责；积分限幅、条件积分和
  `i_factor=max(0,1−(e_ω/400°)²)`；抗饱和只从 `control/allocation_status.saturated`
  读取，不访问分配器内部状态。
- **PINV**：`B_tau = d_cs·Ve²·diag([l1,l1,l2])·B_M`（N·m/rad），
  `δ = δ_trim + pinv(B_tau)·τ_d`；`τ_d` 是**相对配平的力矩增量**，零力矩回到配平；
  `residual = τ_d − B_tau·(δ−δ_trim)`，`saturated=sign(residual)`（限幅内为 0）。
  电机 `Ω=sqrt(T/k_T0)` 独立求解，六舵面与电机不混成七维分配。

### 15.3 Topic 契约

| Topic | 发布者 | 订阅者 | 字段（SI） |
|---|---|---|---|
| `sensor/state` | 理想传感器 | Navigation | `value`=[p;v;q;ω]、`sample_time_s` |
| `navigation/state` | IdealState | 四级控制器 | p、v、a、q、ω、α、`ideal=1` |
| `guidance/reference` | HoverHold | Position | p、v、a、yaw、yaw_rate |
| `guidance/rate_reference` | BodyRate | Rate | ω、α |
| `control/velocity_setpoint` | Position | Velocity | `velocity`、`acceleration_ff`、`yaw`、`yaw_rate` |
| `control/attitude_setpoint` | Velocity | Attitude、Allocation | q、`thrust_n`、`yaw`、`yaw_rate` |
| `control/rate_setpoint` | Attitude | Rate | ω、α |
| `control/torque_setpoint` | Rate | Allocation | `torque` |
| `control/allocation_status` | Allocation | Rate、诊断 | δ、achieved、residual、saturated、motor、Ve_nom |

总线与契约由 `src/GNC/Control/Internal/topic_specs.m` 唯一定义；不得建立包含
所有控制器变量的“大结构体”或第二套通信入口。

### 15.4 组合与状态

Guidance 的参考类型决定启用链：

| 参考类型 | 启用模块 |
|---|---|
| Position | Position → Velocity → Attitude → Rate → Allocation |
| Velocity | Velocity → Attitude → Rate → Allocation |
| Attitude | Attitude → Rate → Allocation |
| BodyRate | Rate → Allocation |

当前 V1.0 只实现 Position（HoverHold）与 BodyRate（阶跃）两种 Guidance；
Velocity/Attitude 参考组合由同一组四级组件按需装配，不虚构额外制导算法。

每个控制器只读写自己的 `bus.control_state.<level>`（velocity/rate 有积分状态，
attitude 有参考模型状态，position 无状态）；跨模块数据只能经 Topic，不得用这些
状态字段传值，也不得修改 Simulation 内核。

### 15.5 频率与顺序

默认：Position 100 Hz、Velocity 100 Hz（可独立配置）、Attitude 200 Hz、
Rate 400 Hz、Allocation 400 Hz、舵面锁存 200 Hz、电机锁存 50 Hz。
同一时刻执行顺序：position → velocity → attitude → rate → allocation → latches；
下游只读取已发布的上游输出，不发生未来数据泄漏。

### 15.6 WTR 导出

`tools/wtr/wtr_export` 从完整 Topic 日志（要求 `dropped==0`）与 `result.state` 生成
`windtrace.flight.v1`：

- `actuator_servos_pwm` 使用真实 `latch_servos` 事件时刻，`actuator_motors_pwm`
  使用 `latch_motor` 事件时刻；不使用状态输出时间轴，不插值；
- manifest 的实验签名覆盖场景、参考、方法、种子、机型版本、完整增益、求解器
  （含 `InitialStep`）、环境、时间容差、时间区间、气动来源/可信等级/SHA256、
  参数覆盖，以及组件的 `kind/period/priority/offset/delay/latches/topic` 和实际初态；
- 参数记录机型/环境/气动/执行器/四级增益与限幅/PINV/频率/求解器/场景，包括
  `rates.guidance_hz`、`allocation.ve_min_m_s`、`control.rate.saturation_tolerance`、
  涵道/阻尼/扩散器参数；未知值不填默认，名义值不标实测；
- `.wtr` 复用 `aero_results_path`、`aero_results_check`，临时 ZIP 位于目标 Results
  目录同一文件系统，容量检查后原子改名；拒绝覆盖 `.mat`/`.wtr`，失败不删除权威 `.mat`。

### 15.7 限制

1. 增益为仿真名义值；未实机辨识，未做鲁棒性/蒙特卡洛评估。
2. 气动为合成 fixture，物理闭环验收 **BLOCKED**；不得表述为真实气动或实机验证。
3. Navigation 为 IdealState，无传感器噪声/偏置；加速度由测量差分得到。
4. 未实现 INDI/MPC/CA16/可选姿态误差模式/增益调度；WTR 固定 v1 协议。
