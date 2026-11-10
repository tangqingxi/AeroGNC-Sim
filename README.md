# AeroGNC-Sim

AeroGNC-Sim 是一个面向研究者的 MATLAB 飞行控制与制导仿真平台。它把机型物理、
执行器动态、GNC 算法、实验场景和结果交付分成可替换的模块，用于算法对比、
模型替换、闭环场景实验和结果复盘。

SHC09 涵道式尾座无人机是平台的首个完整机型，但平台本身不绑定 SHC09：机型参数、
气动模型和执行器设备都位于独立目录，GNC 与 Simulation 通过通用接口工作。

## 1. 系统总体架构

```text
Guidance（参考生成）
        ↓
Navigation（状态估计）
        ↓
Control
├── Position
├── Velocity
├── Attitude
├── Rate
└── Allocation
        ↓
Aircraft（完整机型动力学） + Components（舵机/电机动态）
        ↓
Simulation（调度、锁存、数值积分）
        ↓
Results（原始结果与发布日志）
        ↓
WTR（派生可视化数据） → WindTrace
```

职责边界：

- **Simulation** 只负责时钟、事件调度、锁存、积分和结果组装，不包含气动方程。
- **Aircraft** 提供完整机型动力学、状态布局、物理参数和气动模型。
- **Components** 提供通用舵机/电机动态与设备型号参数。
- **GNC** 提供参考、估计和控制算法；控制算法不能读取 Aircraft 真值。
- **Scenarios** 只负责实验装配，不实现控制律。
- **WTR** 负责离线打包与数据契约检查，不参与在线控制。

唯一的模块间通信机制是 `src/Simulation/Communication/aero_topic_bus.m` 的
Latest-Value Topic Bus；模块不通过全局变量或 Aircraft 内部字段交换数据。

## 2. 源码目录地图

```text
AeroGNC-Sim/
├── src/
│   ├── Simulation/
│   │   ├── aero_sim.m              仿真入口
│   │   ├── aero_integrate.m        分段 ode45 积分
│   │   ├── Events/                 调度、组件模板
│   │   ├── Configuration/          默认配置与校验
│   │   ├── Communication/          Topic Bus 与数据契约
│   │   ├── Results/                结果路径、容量、保存
│   │   └── Internal/               项目根与环境信息
│   ├── Aircraft/
│   │   └── SHC09/
│   │       ├── shc09_model.m       机型装配
│   │       ├── shc09_parameters.m  物理参数唯一来源
│   │       ├── shc09_dynamics.m    六自由度动力学
│   │       ├── shc09_outputs.m     标准输出
│   │       ├── shc09_hover_equilibrium.m 悬停配平
│   │       ├── Aerodynamics/       涵道、机翼、舵面、力/力矩
│   │       └── Internal/           状态布局、坐标、元数据
│   ├── Components/
│   │   └── Actuators/
│   │       ├── Servo/              通用舵机动力学与 KST 设备
│   │       └── Motor/              通用电机动力学与 MAD 设备
│   ├── GNC/
│   │   ├── Guidance/
│   │   │   ├── hover_hold.m        位置保持参考
│   │   │   └── body_rate_step.m    角速度阶跃参考
│   │   ├── Navigation/
│   │   │   └── ideal_state.m       IdealState 测试适配器（非 EKF）
│   │   └── Control/
│   │       ├── Position/position_p.m
│   │       ├── Velocity/velocity_pid.m
│   │       ├── Attitude/attitude_p.m
│   │       ├── Rate/rate_pid.m
│   │       ├── Allocation/pinv_allocator.m
│   │       └── Internal/           契约、默认参数、四元数/推力辅助
│   └── Scenarios/
│       ├── hover_scenario.m        悬停位置保持场景
│       ├── rate_step_scenario.m    角速度阶跃场景
│       └── Internal/               配置构建、签名、传感器、锁存
├── tools/wtr/                      WTR 导出、校验、JSON/SHA/几何辅助
├── tests/                          回归测试与测试入口
├── examples/                       可运行示例
├── results/                        仿真输出（gitignore）
├── README.md
├── ARCHITECTURE.md
├── DESIGN.md
└── AGENTS.md
```

## 3. 功能位置索引

| 功能 | 主要源码 | 职责 |
|---|---|---|
| 仿真入口 | `src/Simulation/aero_sim.m` | 执行一次完整仿真并组装结果 |
| 数值积分 | `src/Simulation/aero_integrate.m` | 事件边界分段 `ode45` |
| 事件调度 | `src/Simulation/Events/aero_schedule.m` | 多速率组件执行顺序与锁存 |
| Topic Bus | `src/Simulation/Communication/aero_topic_bus.m` | 唯一模块通信机制 |
| 默认配置 | `src/Simulation/Configuration/aero_default_config.m` | 构造默认仿真配置 |
| 结果保存/容量 | `src/Simulation/Results/` | 路径安全、容量策略、MAT 保存 |
| SHC09 装配 | `src/Aircraft/SHC09/shc09_model.m` | 建立机型模型与初态 |
| SHC09 参数 | `src/Aircraft/SHC09/shc09_parameters.m` | 质量、惯量、几何、执行器参数 |
| SHC09 动力学 | `src/Aircraft/SHC09/shc09_dynamics.m` | 连续六自由度方程 |
| SHC09 气动 | `src/Aircraft/SHC09/Aerodynamics/` | 涵道、机翼、舵面力/力矩 |
| 舵机动力学 | `src/Components/Actuators/Servo/aero_servo_dynamics.m` | 一阶舵机与限幅 |
| 电机动力学 | `src/Components/Actuators/Motor/aero_motor_dynamics.m` | 一阶电机与限幅/停机 |
| Position | `src/GNC/Control/Position/position_p.m` | 位置 P → 速度参考 |
| Velocity | `src/GNC/Control/Velocity/velocity_pid.m` | 速度 PID → 姿态/推力 |
| Attitude | `src/GNC/Control/Attitude/attitude_p.m` | 姿态 P → 角速度参考 |
| Rate | `src/GNC/Control/Rate/rate_pid.m` | 角速度 PID → 力矩需求 |
| Allocation | `src/GNC/Control/Allocation/pinv_allocator.m` | 六舵面 PINV 与电机独立推力 |
| Topic 契约 | `src/GNC/Control/Internal/topic_specs.m` | 字段、维度、单位、超时 |
| 控制默认参数 | `src/GNC/Control/Internal/control_defaults.m` | 四级控制器与分配默认值 |
| Guidance | `src/GNC/Guidance/` | 位置/角速度参考生成 |
| Navigation | `src/GNC/Navigation/ideal_state.m` | IdealState 估计适配器 |
| 场景装配 | `src/Scenarios/` | 组件、Topic、初值、频率组装 |
| WTR 导出 | `tools/wtr/wtr_export.m` | 生成 `windtrace.flight.v1` |
| WTR 校验 | `tools/wtr/wtr_validate.m` | 容器与数据契约检查 |
| 测试入口 | `tests/run_tests.m` | 完整回归与测试清单检查 |

## 4. 模块职责与边界

| 模块 | 负责 | 不负责 |
|---|---|---|
| Simulation | 时钟、调度、锁存、积分、日志 | 气动方程、控制律、机型参数 |
| Aircraft | 机型装配、物理参数、动力学、气动 | 控制器实现、场景配置 |
| Components | 通用执行器动态、设备参数 | 机型几何、GNC 算法 |
| Guidance | 参考轨迹/目标生成 | 状态估计、反馈控制 |
| Navigation | 状态估计适配 | 真值注入控制、执行器命令 |
| Control | P/PID 算法、限幅、分配 | 机型物理参数、仿真时钟 |
| Scenarios | 选择组件、频率、初值、参考 | 控制数学、物理方程 |
| WTR | 结果打包、契约校验、可视化交付 | 在线控制、原始数据替代 |

信息隔离原则：

- GNC 只读取 Topic Bus 上的估计/参考量，不读取 Aircraft 内部真实状态。
- 真值只用于结果记录、WTR 真值对照和测试检查，不进入控制反馈。
- `IdealState` 是测试适配器，不代表传感器噪声、EKF 或真实导航。
- 普通 Topic 不承载 GroundTruth。

## 5. 四级控制与分配

默认控制链：

```text
Position P          v_ref = v_ff + Kp·(p_ref − p)
Velocity PID        a_cmd = a_ff + Kp·e_v + I − Kd·a_est
Attitude P          ω_ref = Kp·e_att + ω_ff
Rate PID            τ = Kp·e_ω + I + Kff·ω_ref − Kd·α_est
Allocation          δ = δ_trim + pinv(B_tau)·τ_rel
```

- 四级控制器分别位于独立目录，拥有独立参数、状态与频率。
- Velocity 的 D 项使用测量加速度反馈，`Kd` 单位为 1（无量纲）。
- Attitude 默认使用 reduced-attitude 误差、yaw weight 与参考模型；`yaw_rate`
  由 Guidance 经 Position/Velocity 传入，不静默丢弃。
- Rate 不做控制器本地力矩裁剪；执行器约束由 Allocation/PINV 承担，饱和通过
  `control/allocation_status` 反馈。
- Allocation 使用 SHC09 物理效能矩阵 `B_tau [N·m/rad]`，输出舵角 `rad`；
  电机推力 `T [N]` 与六舵面分离，不做七维统一求解。
- 不使用 PX4 归一化虚拟控制接口；控制接口保持 SI。

## 6. 运行方法

### 6.1 初始化

```matlab
cd('path/to/AeroGNC-Sim');
aero_setup();
```

`aero_setup.m` 显式登记生产源码目录，不加载 tests，也不递归整个仓库。

### 6.2 使用合成 fixture 运行悬停

```matlab
addpath('tests', 'tests/fixtures', 'tests/helpers');
source = struct('type', 'fixture', 'tables', shc09_aero_fixture());

s = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 4], ...
    'position_offset', [0.1; 0; 0.05]));

result = aero_sim(s.model, s.cfg);
```

fixture 仅用于软件/数值测试，不代表真实气动。

### 6.3 使用外部气动数据运行 SHC09

```matlab
source = struct('type', 'external', 'file', 'path/to/shc09_aero.mat');
s = hover_scenario(struct('aero_source', source, 'tspan', [0, 10]));
result = aero_sim(s.model, s.cfg);
```

外部 MAT 需包含 `duct_raw` 与 `wing_raw` 系数表；真实数据因再分发许可未确认，
不随仓库发布。

### 6.4 运行角速度阶跃

```matlab
source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
s = rate_step_scenario(struct( ...
    'aero_source', source, ...
    'axis', 1, ...
    'amplitude', 0.5, ...
    'start_s', 0.5, ...
    'end_s', 1.5, ...
    'tspan', [0, 2.5]));
result = aero_sim(s.model, s.cfg);
```

### 6.5 导出 WTR

```matlab
out = wtr_export(result, s.model, struct( ...
    'scenario', s.name, ...
    'signature', s.signature, ...
    'gains', s.gains, ...
    'seed', s.cfg.seed));
% out.wtr_path 可由 WindTrace 打开
```

导出要求发布日志完整（`dropped==0`）；`.wtr` 是派生可视化数据，权威原始数据
是同一目录的 `simulation.mat`。

### 6.6 运行测试

```matlab
matlab -batch "run('tests/run_tests.m')"
```

或在 MATLAB 中：

```matlab
run('tests/run_tests.m')
```

`tests/run_tests.m` 会检查测试清单与磁盘文件一致，并按登记顺序执行。

## 7. 结果与可视化

```text
results/<scenario>/<run_id>/
├── simulation.mat
└── flight.wtr
```

- `simulation.mat`：完整仿真结果、状态、事件、配置与发布日志，是科研原始结果。
- `flight.wtr`：按 `windtrace.flight.v1` 打包的派生数据，供 WindTrace 读取与回放。
- WTR 不替代原始结果；分析/复现以 `.mat` 和配置签名为准。
- 结果目录受容量策略与路径安全约束；导出失败不删除已有权威 `.mat`。

## 8. 当前能力与限制

| 类别 | 状态 |
|---|---|
| Simulation 内核 | 已实现：分段积分、多速率调度、锁存、发布日志、结果容量 |
| SHC09 机型 | 已实现：六自由度、涵道/机翼/舵面、执行器、悬停配平 |
| 四级 GNC | 已实现：Position P、Velocity PID、Attitude P、Rate PID |
| 控制分配 | 已实现：六舵面 PINV、配平、限幅、饱和/残差、电机独立推力 |
| 闭环场景 | 已实现：HoverHold、BodyRate Step |
| 导航 | 已实现：IdealState 测试适配器 |
| WTR | 已实现：v1 导出、契约校验、锁存时间轴、参数追溯 |
| 真实气动 | 未完成：数据许可/物理有效性未闭合；fixture 不是物理验证 |
| 实机辨识 | 未完成：除质量外的惯量、执行器动态等为参考/名义值 |
| INDI / MPC / DCA / CA16 / WLS | 未实现 |
| EKF2/ECL、传感器噪声模型 | 未实现 |
| 归一化虚拟控制接口 | 未实施；当前保持 SI 接口 |

物理验证状态：**BLOCKED**。合成气动 fixture 只能证明软件链路、数值稳定性和
接口正确性，不能表述为真实气动、实机或型号认证结果。

## 9. 文档入口

- [`ARCHITECTURE.md`](ARCHITECTURE.md)：架构原则与冻结决策。
- [`DESIGN.md`](DESIGN.md)：接口、数学定义、参数归属与配置细节。
- [`AGENTS.md`](AGENTS.md)：开发、提交与交接约束。
- [AUDIT 仓库](https://github.com/tangqingxi/AeroGNC-Sim-Audit)：阶段验证证据、
  审计报告与交接记录。
