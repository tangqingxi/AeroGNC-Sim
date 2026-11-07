# AeroGNC-Sim

面向 GNC 科研的模块化 MATLAB 飞行仿真平台。架构基线见 [`ARCHITECTURE.md`](ARCHITECTURE.md)，协作与代码规范见 [`AGENTS.md`](AGENTS.md)。

**当前状态**：`Architecture-Baseline` 已验收；`Simulation-Kernel` 完成整改，状态 `ready-for-review`（等待 NEXUS 验收）。尚无 SHC09 机型、无真实 GNC 算法、无传感器硬件模型。

## 目录结构

```text
AeroGNC-Sim/
├── aero_setup.m                 路径初始化（显式登记已实现模块目录）
├── ARCHITECTURE.md              架构基线（冻结决策，不得静默修改）
├── AGENTS.md                    协作、目录与中文注释规范
├── README.md
├── .gitignore                   排除 results/ 与 MATLAB 临时产物
├── src/
│   ├── Simulation/              仿真内核：配置、调度、积分、锁存、结果组装
│   │   ├── aero_sim.m           主入口：分段积分 + 事件调度 + 日志
│   │   ├── aero_schedule.m      事件时刻与同时事件排序
│   │   ├── aero_integrate.m     单段 ode45（零阶保持输入）
│   │   ├── aero_default_config.m / aero_validate_config.m
│   │   ├── aero_empty_component.m / aero_fill_defaults.m
│   │   ├── aero_measurement.m   读取最新可用测量
│   │   ├── aero_project_root.m  定位项目根目录
│   │   └── aero_env_info.m      记录 MATLAB/Git 环境
│   └── Simulation/Results/      结果持久化与容量治理
│       ├── aero_save_result.m   安全写盘（唯一临时文件、拒绝覆盖）
│       ├── aero_results_path.m  路径规范化与越界/链接检查
│       ├── aero_results_policy.m / aero_results_usage.m / aero_results_check.m
├── tests/
│   ├── run_tests.m              回归入口
│   ├── test_*.m                 14 项测试
│   └── fixtures/                解析测试模型（测试替身，不是飞行器）
├── examples/
│   └── demo_first_order.m       最小端到端示例
└── results/                     仿真输出（gitignore，受容量策略约束）
```

尚未实现的 `src/Aircraft/`、`src/Components/`、`src/GNC/`、`src/Scenarios/` 暂不建空目录；开始实现时再创建并登记到 `aero_setup.m`。

## 快速开始

```bat
:: 全部回归测试（约 5 s，退出码 0 表示通过）
matlab -batch "cd('E:\AeroGNC-Sim'); run('tests/run_tests.m')"

:: 最小示例
matlab -batch "cd('E:\AeroGNC-Sim'); run('examples/demo_first_order.m')"
```

交互式使用：

```matlab
cd('E:\AeroGNC-Sim');
aero_setup();

model = struct('name', 'first-order', 'x0', 0, ...
               'dynamics', @(t, x, u, env) -2*x + 3*u);

cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.initial_command = 0;
cfg.components = struct('name', 'command', 'kind', 'periodic', 'period', 0.5, ...
                        'priority', 50, 'latches', true, 'step', @step_fn);

result = aero_sim(model, cfg);
```

## 内核契约

**模型**：`model.name`、可选 `model.x0`、`model.dynamics = @(t,x,u,env)`。状态维度由 `x0` 决定，内核不出现 6DOF 或任何机型下标；动力学必须无副作用。

**时间语义**

- 统一虚拟时间；组件事件时刻为 `t0 + offset + k*period`，由 `k` 直接计算，不经过固定网格，因而无累计漂移。
- 组件更新区间为 `[t0, tf)`；`tf` 只是积分终点边界，不触发组件更新。
- 相邻事件之间用**分段 `ode45`** 积分，段内锁存输入保持不变；`ode45` 内部步点仅用于积分，不是采样点。
- 事件按 `(时间, priority, 组件顺序, 动作)` 排序；相差小于 `cfg.time_tolerance`（默认 `1e-12*max(1,tf)`）的事件构成同一时刻的同步组。
- 默认理想零计算延迟；`result.stats.wall_time_s` 只是 MATLAB 实际耗时，不代表飞控执行延迟。

**组件类型**

| kind | 字段 | 语义 |
|---|---|---|
| `periodic` | `period, offset, priority, step, latches` | 周期运行 `bus = step(t, bus)`；`latches=true` 才允许修改 `bus.held_command` |
| `sensor` | `period, offset, priority, delay, sample` | `value = sample(t, x)` 读取真值；测量在 `t+delay` 才发布 |

**因果与隔离**

- 真值只通过 sensor 的 `sample(t, x)` 传入，`periodic` 组件永远拿不到真实状态（`bus.truth` 恒为空）。
- 测量在 `sample_time + delay` 之后才进入 `bus.measurements`，控制器不可能读到未来数据。
- `bus.command` 是控制计算值，`bus.held_command` 是实际进入动力学的锁存值；非 `latches` 组件修改锁存值会被内核报错。
- `bus.command` / `bus.held_command` / `bus.measurements` 由内核保证存在；其余字段可由组件自由读写（如 `bus.nav`）。

**错误处理**：非法配置、非法周期、重复组件名、非有限传感器输出、越权锁存等均以 `aero:*` 标识符报错，不静默修正。

## 结果结构

| 字段 | 含义 |
|---|---|
| `time`, `state` | 事件边界时间与对应连续状态（真值，供验证/记录） |
| `events` | 每个事件的时间、组件、动作（`sample`/`publish`/`step`）、优先级 |
| `measurements` | 已发布测量：`name, sample_time, available_time, value` |
| `held`, `computed` | 每个边界后的实际锁存值与最新计算值 |
| `segments` | 每段 `t0, t1, held, internal_points`（`ode45` 内部点数） |
| `solver`, `config`, `seed`, `environment` | 生效求解器设置、完整配置、随机种子、MATLAB 版本与 Git 提交 |
| `stats` | 事件数、段数、锁存次数、内部积分点总数、实际耗时 |

当前不伪造 Navigation/Guidance/Control 的正式数据通道；需要时由组件写入 `bus` 字段，结果中通过事件与 `computed` 体现。

## Results 容量与安全

- 结果默认**只驻留内存**；只有 `cfg.save.enabled = true` 且给出 `cfg.save.path` 才写盘。
- 路径必须落在 results 根目录（默认 `<repo>/results`，已 gitignore）：拒绝 `../` 穿越、根目录外的绝对路径、仅同前缀的相似路径，以及经过符号链接/Junction 的路径；非法路径不会先创建目录。
- **不覆盖已有文件**：目标已存在时直接报 `aero:results:exists`，原文件内容与哈希保持不变。
- 写入先落到同目录的唯一临时文件，容量按“现有用量 + 临时文件大小”计算（临时文件不计入现有用量，新文件只计一次）；超限时删除本次临时文件并报错，**不删除、不覆盖已有证据**，也不误删其他任务的临时文件。
- 上限 **10 GB**（`10e9` 字节，比 10 GiB 更保守），**8 GB** 预警；文件数预算 **5,000**（临时建议值，达到 80% 提示复核，不阻断）。
- 不自动清理原始科研证据，不向 AUDIT 复制大型原始数据。

```matlab
p = aero_results_policy();                 % 10e9 / 8e9 / 5000 / 4000
aero_results_usage('results');             % 当前字节数与文件数
aero_results_check('results', p, 2e9);     % ok|warning|file-warning|file-limit|limit
```

## 测试与容差

`tests/run_tests.m` 覆盖 14 项：解析模型一致性、一阶动态、状态维度通用性、分段边界、多速率（800/200/125 Hz 与 73/127 Hz）、同时事件因果链、测量延迟、指令锁存、求解器容差独立性、可复现性、Results 容量、Results 安全（覆盖/穿越/链接）、非法输入、`ode45` 自适应。

默认数值容差：`RelTol = 1e-9`、`AbsTol = 1e-11`；测试阈值在实现前设定，未因失败而放宽。实测最大误差：单状态 `2.2e-16`、一阶 `1.4e-11`、三状态 `1.5e-11`、自适应示例 `7.4e-6`（阈值 `1e-5`）。

系统不允许创建 Junction 时，链接拒绝测试会标记为 `SKIP` 而非通过；报告需如实说明。

## 已实现与未实现

- **已实现**：事件驱动分段 `ode45`、多速率调度、零阶保持锁存、延迟测量发布、真值隔离、结果结构与容量安全策略、14 项回归。
- **测试替身**：`tests/fixtures/` 的解析模型、测试中的虚拟传感器与控制器，不冒充正式模块。
- **未实现**：SHC09 动力学、真实执行器/传感器、PID/INDI/MPC/分配、EKF/GNSS/IMU、GUI/3D 可视化。

## 代码规范

新增与修改的 MATLAB 代码使用简体中文注释，目录职责、可读性要求与移动后复验规则见 [`AGENTS.md`](AGENTS.md) 第 5、6 节。
