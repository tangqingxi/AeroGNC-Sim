function info = test_shc09_operating_states()
%TEST_SHC09_OPERATING_STATES 停机保持、启动、停止与持续自由落体验证
%
% 关注点：物理停机（0 rad/s）与最低运行转速（200 rad/s）必须区分，
% 状态投影不得把停机状态抬到运行下限。

src = struct('type', 'fixture', 'file', '', 'tables', shc09_aero_fixture());
% 垂直下落时涵道迎角趋于 180°，超出气动表有效域；本测试只验证停机与运动学，
% 因此显式选择 clip 策略，并在此说明这一物理限制（不是静默外推）。
model = shc09_model(struct('aero_source', src, 'initial_mode', 'rest', ...
    'aero_domain_policy', 'clip'));
p = model.parameters;
s = model.state_layout;

cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = model.x0;
cfg.solver = struct('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', Inf, 'InitialStep', []);

% ---- 1. 停机保持：电机指令 0，允许真实零转速 ------------------------
cfg.initial_command = struct('servos', zeros(6, 1), 'motor', 0);
res = aero_sim(model, cfg);
check_that(max(abs(res.state(:, s.motor))) < 1e-12, ...
    '停机指令下电机转速被抬高（max %.3e）', max(abs(res.state(:, s.motor))));

% 无推力时自由落体：z(t) = 0.5 g t²
expectedZ = 0.5 * p.gravity_m_s2 * 1^2;
check_that(abs(res.state(end, 3) - expectedZ) < 1e-6, ...
    '自由落体高度 %.6f 与解析 %.6f 不符', res.state(end, 3), expectedZ);
check_that(abs(res.state(end, 6) - p.gravity_m_s2) < 1e-6, ...
    '自由落体垂向速度 %.6f 与解析 %.6f 不符', res.state(end, 6), p.gravity_m_s2);

% ---- 2. 从停机启动：指令 200 rad/s --------------------------------
cfg.initial_command = struct('servos', zeros(6, 1), 'motor', 200);
res = aero_sim(model, cfg);
check_that(res.state(1, s.motor) == 0, '启动瞬间转速不应跳变到运行下限');
check_that(abs(res.state(end, s.motor) - 200) < 1, ...
    '1 s 后转速 %.3f 未收敛到 200 rad/s', res.state(end, s.motor));

% ---- 3. 停止指令：从 800 rad/s 减速到 0 ---------------------------
x0 = model.x0;
x0(s.motor) = 800;
cfg.x0 = x0;
cfg.initial_command = struct('servos', zeros(6, 1), 'motor', 0);
res = aero_sim(model, cfg);
check_that(abs(res.state(1, s.motor) - 800) < 1e-12, '停机测试初值未生效');
check_that(res.state(end, s.motor) < 1, ...
    '停机指令 1 s 后转速仍为 %.3f', res.state(end, s.motor));

% ---- 4. 持续自由落体（2 s） ---------------------------------------
cfg.x0 = model.x0;
cfg.tspan = [0, 2];
res = aero_sim(model, cfg);
check_that(abs(res.state(end, 6) - 2 * p.gravity_m_s2) < 1e-6, ...
    '2 s 自由落体垂向速度 %.6f 与解析 %.6f 不符', ...
    res.state(end, 6), 2 * p.gravity_m_s2);

info = struct('passed', true, 'summary', sprintf( ...
    ['停机保持 0 rad/s，启动到 200 rad/s，停止收敛到 %.2f，自由落体 2 s 与解析一致；' ...
     '垂直下落超出气动表域，使用显式 clip 策略'], res.state(end, s.motor)));

end
