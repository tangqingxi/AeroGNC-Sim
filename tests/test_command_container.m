function info = test_command_container()
%TEST_COMMAND_CONTAINER 结构体命令的通用锁存：两路信号互不覆盖

% ---- 内核不解释字段名：使用任意字段 a / b ---------------------------
model = struct();
model.name = 'command-probe';
model.x0 = 0;
model.dynamics = @(t, x, cmd, env) cmd.a; %#ok<INUSD>

cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.initial_command = struct('a', 0, 'b', 2);
cfg.solver = struct('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', Inf, 'InitialStep', []);
cfg.components = struct( ...
    'name', {'a_cmd', 'b_cmd', 'a_latch', 'b_latch'}, ...
    'kind', 'periodic', ...
    'period', {0.1, 0.25, 0.2, 0.5}, ...
    'priority', {40, 41, 50, 55}, ...
    'latches', {false, false, true, true}, ...
    'step', {@(t, bus) set_field(bus, 'command', 'a', t), ...
             @(t, bus) set_field(bus, 'command', 'b', 10 + t), ...
             @(t, bus) copy_field(bus, 'servos', 'a'), ...
             @(t, bus) copy_field(bus, 'motor', 'b')});

res = aero_sim(model, cfg);

heldA = cellfun(@(h) h.a, res.held);
heldB = cellfun(@(h) h.b, res.held);
times = res.time;

% a 路只在 0.2 s 网格上变化；b 路只在 0.5 s 网格上变化
for g = 2:numel(times)
    t = times(g);
    aChanged = abs(heldA(g) - heldA(g - 1)) > 1e-12;
    bChanged = abs(heldB(g) - heldB(g - 1)) > 1e-12;
    if aChanged
        check_that(abs(t / 0.2 - round(t / 0.2)) < 1e-9, ...
            'a 路在非 0.2 s 锁存时刻 %.6f 发生变化', t);
    end
    if bChanged
        check_that(abs(t / 0.5 - round(t / 0.5)) < 1e-9, ...
            'b 路在非 0.5 s 锁存时刻 %.6f 发生变化', t);
    end
end

% 0.2 s：a 路锁存新值，b 保持不变（t=0 时由 b_cmd 计算并锁存为 10）
idx = find(abs(times - 0.2) < 1e-12, 1);
check_that(abs(heldA(idx) - 0.2) < 1e-12, 'a 路锁存值 %.6f 与计算值 0.2 不符', heldA(idx));
check_that(abs(heldB(idx) - heldB(1)) < 1e-12, ...
    'a 路更新时覆盖了 b 路（%.6f vs %.6f）', heldB(idx), heldB(1));

% 0.5 s：b 路锁存新值，a 保持 0.4（0.4 s 处锁存的最近值）
idx = find(abs(times - 0.5) < 1e-12, 1);
check_that(abs(heldB(idx) - 10.5) < 1e-12, 'b 路锁存值 %.6f 与计算值 10.5 不符', heldB(idx));
check_that(abs(heldA(idx) - 0.4) < 1e-12, 'b 路更新时覆盖了 a 路（%.6f）', heldA(idx));

% 积分只使用a路锁存值：按分段锁存值重建积分并核对
rebuilt = 0;
for k = 1:numel(res.segments)
    s = res.segments{k};
    rebuilt = rebuilt + s.held.a * (s.t1 - s.t0);
end
check_that(abs(res.state(end, 1) - rebuilt) < 1e-9, ...
    '状态积分与锁存值不一致（%.9f vs %.9f）', res.state(end, 1), rebuilt);

% ---- 原有数值命令接口不受影响 --------------------------------------
legacy = aero_default_config();
legacy.tspan = [0, 0.5];
legacy.initial_command = 1;
legacy.components = struct('name', 'latch', 'kind', 'periodic', 'period', 0.25, ...
    'priority', 50, 'latches', true, 'step', @(t, bus) set_numeric(bus, -2));
resLegacy = aero_sim(scalar_integrator_model(), legacy);
check_that(isfinite(resLegacy.state(end)), '数值命令接口回归失败');

% ---- 非法结构体命令必须明确失败 ------------------------------------
bad = aero_default_config();
bad.tspan = [0, 0.1];
bad.initial_command = struct('a', [1, NaN]);
check_error(@() aero_sim(model, bad), 'aero:config:command', '非有限结构体字段');

% ---- SHC09 指令容器形状检查 ----------------------------------------
p = shc09_parameters();
check_error(@() shc09_unpack_command(struct('servos', zeros(6, 1))), ...
    'shc09:command:format', '缺少 motor 字段');
check_error(@() shc09_unpack_command(struct('servos', zeros(5, 1), 'motor', 1)), ...
    'shc09:command:servos', '舵面维度不是 6');

info = struct('passed', true, 'summary', sprintf( ...
    '两路结构体命令独立锁存、互不覆盖（a: %.2f, b: %.2f），数值命令仍可用', heldA(idx), heldB(idx)));

end

% ===========================================================================
function bus = set_field(bus, group, name, value)
bus.(group).(name) = value;
end

function bus = copy_field(bus, target, name)
bus.held_command.(name) = bus.command.(name);
end

function bus = set_numeric(bus, value)
bus.held_command = value;
end
