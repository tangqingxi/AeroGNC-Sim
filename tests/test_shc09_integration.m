function info = test_shc09_integration()
%TEST_SHC09_INTEGRATION 在 Simulation-Kernel 中运行 SHC09 开环仿真

check_that(~isempty(which('shc09_model')), 'aero_setup 未登记 SHC09 路径');
check_that(~isempty(which('aero_servo_dynamics')), 'aero_setup 未登记 Components 路径');

src = struct('type', 'fixture', 'file', '', 'tables', shc09_aero_fixture());
model = shc09_model(struct('aero_source', src));
p = model.parameters;
s = model.state_layout;
trim = model.trim;

cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = model.x0;
cfg.initial_command = model.initial_command;
cfg.solver = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
cfg.components = struct( ...
    'name', {'servo_cmd', 'motor_cmd', 'servo_latch', 'motor_latch', 'truth_guard'}, ...
    'kind', 'periodic', ...
    'period', {0.01, 0.02, 0.005, 0.02, 0.05}, ...
    'priority', {40, 41, 50, 55, 60}, ...
    'latches', {false, false, true, true, false}, ...
    'step', { ...
        @(t, bus) command_servos(bus, trim.delta_rad * ones(6, 1), t, 0.5), ...
        @(t, bus) command_motor(bus, trim.omega_rad_s, t, 0.3), ...
        @(t, bus) copy_servos(bus), ...
        @(t, bus) copy_motor(bus), ...
        @(t, bus) guard_truth(bus)});

res = aero_sim(model, cfg);

check_that(size(res.state, 2) == s.nx, '状态维度 %d 不是 20', size(res.state, 2));
qn = sqrt(sum(res.state(:, s.quaternion).^2, 2));
check_that(max(abs(qn - 1)) < 1e-10, '四元数范数偏离 1：最大 %.3e', max(abs(qn - 1)));
check_that(max(abs(res.state(:, s.servo)), [], 'all') <= p.servo_deflection_limit_rad + 1e-9, ...
    '舵面偏角超出机械行程');

before = find(res.time <= 0.25, 1, 'last');
check_that(norm(res.state(before, s.position)) < 1e-6, ...
    '配平段位置漂移 %.3e m 超过 1e-6', norm(res.state(before, s.position)));

% 0.3 s 后电机指令已更新，0.5 s 前舵面指令未变
g35 = find(abs(res.time - 0.35) < 1e-12, 1);
check_that(abs(res.held{g35}.motor - trim.omega_rad_s * 1.02) < 1e-9, ...
    '0.35 s 时电机锁存值未更新');
check_that(abs(res.held{g35}.servos(1) - trim.delta_rad) < 1e-12, ...
    '电机更新时覆盖了舵面指令');

% 0.5 s 后舵面指令更新，电机值保持 1.02 倍
g55 = find(abs(res.time - 0.55) < 1e-12, 1);
check_that(abs(res.held{g55}.servos(1) - (trim.delta_rad + deg2rad(2))) < 1e-12, ...
    '0.55 s 时舵面锁存值未更新');
check_that(abs(res.held{g55}.motor - trim.omega_rad_s * 1.02) < 1e-9, ...
    '舵面更新时覆盖了电机指令');

% 实际执行器状态连续跟随指令
actualDelta = res.state(end, s.servo);
check_that(mean(actualDelta) > trim.delta_rad + deg2rad(0.5), ...
    '舵面实际状态未跟随阶跃指令');
check_that(res.state(end, s.motor) > trim.omega_rad_s * 1.01, ...
    '电机实际转速未跟随阶跃指令');

% 结果结构可读
check_that(isfield(res, 'events') && ~isempty(res.events), '事件日志缺失');
check_that(numel(res.segments) == numel(res.time) - 1, '分段数与时间点数不一致');

info = struct('passed', true, 'summary', sprintf( ...
    'SHC09 20 状态积分 %d 段；配平漂移<1e-6 m；舵/电机独立锁存；末态 δ=%.2f°, Ω=%.1f', ...
    numel(res.segments), rad2deg(mean(actualDelta)), res.state(end, s.motor)));

end

% ===========================================================================
function bus = command_servos(bus, base, t, stepTime)
if t >= stepTime - 1e-12
    bus.command.servos = base + deg2rad(2);
else
    bus.command.servos = base;
end
end

function bus = command_motor(bus, base, t, stepTime)
if t >= stepTime - 1e-12
    bus.command.motor = base * 1.02;
else
    bus.command.motor = base;
end
end

function bus = copy_servos(bus)
bus.held_command.servos = bus.command.servos;
end

function bus = copy_motor(bus)
bus.held_command.motor = bus.command.motor;
end

function bus = guard_truth(bus)
check_that(isempty(bus.truth), '周期组件读取到了真实状态（bus.truth 非空）');
end
