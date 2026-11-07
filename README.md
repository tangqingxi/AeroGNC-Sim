# AeroGNC-Sim

面向 GNC 科研的模块化 MATLAB 飞行仿真平台。架构基线见 [`ARCHITECTURE.md`](ARCHITECTURE.md)，协作规则见 [`AGENTS.md`](AGENTS.md)。

**当前状态**：`Architecture-Baseline` 已验收；`Simulation-Kernel` 已实现并通过全部回归测试，状态 `ready-for-review`（等待 NEXUS 验收）。尚无 SHC09 机型、无真实 GNC 算法、无传感器硬件模型。

## 目录

```text
ARCHITECTURE.md   架构基线（冻结决策，不得静默修改）
AGENTS.md         双仓库协作与交接规则
src/              Simulation 内核（通用，不含任何机型物理）
models/           解析测试模型（kernel 测试替身，不是飞行器模型）
tests/            自动化回归测试
examples/         最小可运行示例
results/          仿真输出（已 gitignore，见容量治理）
```

## 快速开始

```bat
matlab -batch "cd('E:\AeroGNC-Sim'); run('tests/run_tests.m')"
matlab -batch "cd('E:\AeroGNC-Sim'); run('examples/demo_first_order.m')"
```

测试失败时 `matlab -batch` 返回非零退出码。全部测试约 2 s，无额外工具箱依赖（仅 MATLAB 基础 `ode45`/`odeset`/`save`）。

## 内核契约

```matlab
model.name     = 'first-order';
model.x0       = 0;                                  % 任意维度，内核不假设 6DOF
model.dynamics = @(t, x, u, env) -2*x + 3*u;         % 无副作用

cfg = aero_default_config();
cfg.tspan = [0 2];
cfg.initial_command = 0;
cfg.components = struct('name','command','kind','periodic','period',0.5, ...
                        'priority',50,'latches',true,'step',@step_fn);

result = aero_sim(model, cfg);
```

**时间语义**

- 统一虚拟时间；组件事件时刻为 `t0 + offset + k*period`，由 `k` 直接计算，不经过固定网格，因而无累计漂移。
- 组件更新区间为 `[t0, tf)`；`tf` 只是积分终点边界，不触发组件更新。
- 相邻事件之间用**分段 `ode45`** 积分，段内锁存输入保持不变；`ode45` 内部步点仅用于积分，不是采样点。
- 事件按 `(时间, priority, 组件顺序, 动作)` 排序；同一时刻（相差小于 `cfg.time_tolerance`，默认 `1e-12*max(1,tf)`）的事件构成一个同步组，按声明优先级顺序执行。
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

`result` 至少包含：

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

## Results 容量治理

- 结果默认**只驻留内存**；只有 `cfg.save.enabled = true` 且给出 `cfg.save.path` 才写盘，路径必须位于 results 根目录（默认 `<repo>/results`，已 gitignore）。
- 活跃 Results 上限 **10 GB**（按 `10e9` 字节，较 10 GiB 保守），**8 GB** 预警。
- 写盘先写临时文件；若投影用量超限，临时文件被删除并报错，已有证据不被删除、不被覆盖。
- 文件数预算 **5,000**（临时建议值，非冻结标准）：达到 80%（4,000）提示复核，达到上限仍只告警、不阻断。
- 不自动清理原始科研证据，不向 AUDIT 复制大型原始数据。

```matlab
p = aero_results_policy();                 % 10e9 / 8e9 / 5000 / 4000
aero_results_usage('results');             % 当前字节数与文件数
aero_results_check('results', p, 2e9);     % 状态: ok|warning|file-warning|file-limit|limit
```

## 测试与容差

`tests/run_tests.m` 覆盖 13 项：解析模型一致性、一阶动态、状态维度通用性、分段边界、多速率（800/200/125 Hz 与 73/127 Hz）、同时事件因果链、测量延迟、指令锁存、求解器容差独立性、可复现性、Results 限额、非法输入、`ode45` 自适应。

默认数值容差：`RelTol = 1e-9`、`AbsTol = 1e-11`；测试阈值（1e-9 等）在实现前设定，未因失败而放宽。实测最大误差：单状态 `2.2e-16`、一阶 `1.4e-11`、三状态 `1.5e-11`、自适应示例 `7.4e-6`（阈值 `1e-5`）。

## 已知边界

- 模型均为解析测试替身，`Aircraft` 真实动力学、执行器、传感器、GNC 算法均属后续阶段。
- 测量值支持任意有限实数（标量或向量），但内核不解释其物理含义。
- 无 GUI、无 3D 可视化、无 Simulink/ROS/Gazebo、无数据库或后台服务。
