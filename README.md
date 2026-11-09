# AeroGNC-Sim

模块化 MATLAB GNC 科研仿真平台。用户通过选择机型（Aircraft）、设备型号（Components）、
制导/导航/控制算法与场景来搭建仿真，统一由 Simulation 内核运行。SHC09 涵道式尾座无人机
是首个完整机型实例，但平台不绑定 SHC09。

- 架构基线：[`ARCHITECTURE.md`](ARCHITECTURE.md)
- 详细设计：[`DESIGN.md`](DESIGN.md)（模块边界、设备参数、接口约定、SHC09 冻结定义）
- 协作与代码规范：[`AGENTS.md`](AGENTS.md)

## 当前支持

- Simulation 内核：统一虚拟时间、多速率事件、分段 `ode45`、指令锁存、测量延迟、结果记录与容量策略。
- Aircraft：SHC09 非线性六自由度模型（20 状态：位置、速度、四元数姿态、角速度、
  六舵面实际偏角、旋翼实际转速），含涵道/机翼/六舵面/陀螺/反扭矩/风扰。
- Components：通用 Servo/Motor 一阶动力学（含 `model_type` 校验与停机/启动语义），以及
  KST X08H、MAD M6C06 320KV V2 型号参数。±40° 行程是 SHC09 装配限制，不是 KST 型号规格。
- 指令接口：`command.servos`（6×1）与 `command.motor` 两条独立信号，可不同频率锁存；
  内核同时兼容原有数值命令，并在运行期校验非法命令。
- SHC09 悬停配平（不依赖气动表）；执行器与实际状态均为连续状态；停止指令允许真实零转速。
- 结果记录模型元数据（版本、质量、惯量、设备型号、生效参数、覆盖值与气动数据 SHA256）。

## 环境要求

- MATLAB R2025b（仅使用基础 MATLAB，无额外工具箱依赖）。
- Windows / Linux 均可；测试在 Windows + R2025b 上验证。

## 初始化与运行

```matlab
cd('E:\AeroGNC-Sim');
aero_setup();                       % 显式登记已实现模块路径

cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.initial_command = 0;            % 数值命令仍可用

model = struct('name', 'first-order', 'x0', 0, ...
               'dynamics', @(t, x, u, env) -2*x + 3*u);
result = aero_sim(model, cfg);
```

SHC09（需要外部气动数据文件，见下）：

```matlab
model = shc09_model(struct('aero_source', ...
    struct('type', 'external', 'file', 'path/to/shc09_aero.mat')));
cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.x0 = model.x0;
cfg.initial_command = model.initial_command;
result = aero_sim(model, cfg);
```

也可运行示例：

```bat
matlab -batch "run('examples/demo_first_order.m')"
matlab -batch "run('examples/demo_shc09_open_loop.m')"
```

SHC09 示例读取环境变量 `AEROGNC_SHC09_AERO_FILE`（或传入数据文件路径）。数据文件需含
`duct_raw`（N×4）与 `wing_raw`（M×4）系数表。**气动原始表再分发许可尚未确认，因此不随
仓库发布**；缺失时示例会打印说明而不伪造数据。

## 测试

```bat
matlab -batch "run('tests/run_tests.m')"
```

覆盖内核 14 项 + SHC09/Components/接口 8 项，共 22 项。SHC09 气动的
**物理验证依赖外部数据**：结构测试使用 `tests/fixtures/shc09_aero_fixture.m` 中
明确标注的合成系数（全零气动表），只验证代码通路、符号与积分，不代表气动性能。

## Results

仿真结果默认只驻留内存；写盘需显式配置并落在 `results/` 下（已 gitignore）。
活跃结果上限 10 GB、8 GB 预警，已有文件默认拒绝覆盖、路径越界会被拒绝。
详见 `DESIGN.md` 与 `src/Simulation/Results/`。

## 尚未实现

Guidance / Navigation / Control / 控制分配、Scenarios、传感器模型、二阶或辨识型执行器动态、
真实气动数据入库、实机参数辨识。当前参数中，惯量与执行器动态参数为参考值/名义值，
不得表述为实机精度。
