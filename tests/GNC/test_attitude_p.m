function info = test_attitude_p()
%TEST_ATTITUDE_P 姿态 P：四元数误差、机体系角速度与单位

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

% 四元数 ↔ 旋转矩阵往返
q = [cos(0.15); sin(0.15) * [0.3; -0.4; 0.5] / norm([0.3; -0.4; 0.5])];
q = q / norm(q);
R = quat_to_rotation(q);
q2 = rotation_to_quat(R);
check_that(norm(q - q2) < 1e-12 || norm(q + q2) < 1e-12, '四元数往返误差过大');

% 偏航 0.2 rad 的期望姿态，Kp=2 → ω_z≈0.4 rad/s
nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', zeros(3, 1), 'alpha', zeros(3, 1), ...
    'sample_time_s', 0, 'ideal', 1);
qDesired = [cos(0.1); 0; 0; sin(0.1)];
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', qDesired, 'thrust_n', p.mass_kg * p.gravity_m_s2), ...
    0, 0, 0, 'test');

params = struct('nav_topic', 'navigation/state', ...
    'attitude_topic', 'control/attitude_setpoint', ...
    'output_topic', 'control/rate_setpoint', ...
    'attitude', struct('Kp', [2; 2; 2], 'rate_limit', [10; 10; 10], ...
        'omega_ff', zeros(3, 1)));

bus = attitude_p(0, bus, params);
msg = bus.topics.read('control/rate_setpoint', 0);
expected = 2 * 2 * sin(0.1);
check_that(abs(msg.data.omega(3) - expected) < 1e-12, ...
    '偏航角速度参考错误：%.6f', msg.data.omega(3));
check_that(norm(msg.data.omega(1:2)) < 1e-12, '出现交叉轴角速度');
check_that(norm(msg.data.alpha) < 1e-12, '角加速度前馈应为零');
check_that(~isfield(bus, 'control_state'), '姿态 P 不应创建控制状态');

% omega_ff 单位为 rad/s，直接相加；角加速度不得混入
params.attitude.omega_ff = [0.1; 0; 0];
bus = attitude_p(0, bus, params);
msg = bus.topics.read('control/rate_setpoint', 0);
check_that(abs(msg.data.omega(1) - 0.1) < 1e-12, 'omega_ff 未按 rad/s 叠加');

% 限幅独立生效
params.attitude.Kp = [100; 100; 100];
params.attitude.rate_limit = [0.5; 0.5; 0.5];
bus = attitude_p(0, bus, params);
msg = bus.topics.read('control/rate_setpoint', 0);
check_that(all(abs(msg.data.omega) <= 0.5 + 1e-12), '角速度限幅失效');

info = struct('passed', true, 'summary', '四元数误差、omega_ff(rad/s)、alpha=0 与限幅通过');

end

% ===========================================================================
function bus = make_bus(trim)
bus = struct();
bus.command = trim.command;
bus.topics = aero_topic_bus();
specs = topic_specs();
for k = 1:numel(specs)
    bus.topics.register(specs{k}.name, specs{k}.spec);
end
end
