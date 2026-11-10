function info = test_rate_pid()
%TEST_RATE_PID PX4 Rate PID：P/I/D、i_factor、条件积分与无本地力矩限幅

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

gains = struct('Kp', [0.35; 0.35; 0.06], 'Ki', [0.08; 0.08; 0.02], ...
    'Kd', [0.5; 0.5; 0.5], 'Kff', [0; 0; 0], ...
    'integral_limit', [0.15; 0.15; 0.05], 'saturation_tolerance', 1e-6);
params = struct('nav_topic', 'navigation/state', ...
    'rate_topic', 'control/rate_setpoint', ...
    'allocation_topic', 'control/allocation_status', ...
    'output_topic', 'control/torque_setpoint', ...
    'rate', gains, 'rates', struct('rate_hz', 400));

nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', zeros(3, 1), 'alpha', [1; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
ref = struct('omega', [5; 0; 0], 'alpha', zeros(3, 1));
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0, 0, 0, 'test');

% 无饱和：i_factor 应衰减大误差的积分；力矩 > 旧 0.5 N·m 安全限幅，
% 证明 Rate PID 不再做本地力矩裁剪。
bus = rate_pid(0, bus, params);
tau = bus.topics.read('control/torque_setpoint', 0).data.torque;
iFactor = max(0, 1 - (5 / deg2rad(400))^2);
expectedIntegral = gains.Ki(1) * iFactor * 5 * 0.0025;
check_that(abs(tau(1) - (0.35 * 5 - 0.5 * 1)) < 1e-12, ...
    'P/D 合成力矩错误：%.6f', tau(1));
check_that(tau(1) > 0.5, 'Rate PID 仍存在本地力矩限幅');
check_that(abs(bus.control_state.rate.integral(1) - expectedIntegral) < 1e-12, ...
    'i_factor 积分更新错误');

% 正饱和 + 正误差：条件积分冻结
bus.topics.publish('control/allocation_status', status_msg([1; 0; 0]), ...
    0.0025, 0.0025, 0.0025, 'test');
bus.topics.publish('navigation/state', nav, 0.0025, 0.0025, 0.0025, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0.0025, 0.0025, 0.0025, 'test');
before = bus.control_state.rate.integral(1);
bus = rate_pid(0.0025, bus, params);
check_that(abs(bus.control_state.rate.integral(1) - before) < 1e-15, ...
    '正饱和时正误差积分未冻结');

% 负饱和 + 负误差：条件积分冻结
ref.omega = [-5; 0; 0];
bus.topics.publish('control/allocation_status', status_msg([-1; 0; 0]), ...
    0.005, 0.005, 0.005, 'test');
bus.topics.publish('navigation/state', nav, 0.005, 0.005, 0.005, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0.005, 0.005, 0.005, 'test');
before = bus.control_state.rate.integral(1);
bus = rate_pid(0.005, bus, params);
check_that(abs(bus.control_state.rate.integral(1) - before) < 1e-15, ...
    '负饱和时负误差积分未冻结');

% 饱和方向与误差相反时仍可积分（卸载）
ref.omega = [5; 0; 0];
bus.topics.publish('control/allocation_status', status_msg([-1; 0; 0]), ...
    0.0075, 0.0075, 0.0075, 'test');
bus.topics.publish('navigation/state', nav, 0.0075, 0.0075, 0.0075, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0.0075, 0.0075, 0.0075, 'test');
before = bus.control_state.rate.integral(1);
bus = rate_pid(0.0075, bus, params);
check_that(bus.control_state.rate.integral(1) > before, ...
    '饱和方向与误差相反时积分未更新');

% i_factor 在 400°/s 以上为 0
bus2 = make_bus(trim);
ref2 = struct('omega', [7; 0; 0], 'alpha', zeros(3, 1));
bus2.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus2.topics.publish('control/rate_setpoint', ref2, 0, 0, 0, 'test');
bus2 = rate_pid(0, bus2, params);
check_that(abs(bus2.control_state.rate.integral(1)) < 1e-15, ...
    'i_factor 未在大误差时归零');

info = struct('passed', true, 'summary', ...
    'P/I/D、i_factor、正负饱和条件积分与无本地力矩裁剪通过');

end

% ===========================================================================
function msg = status_msg(saturated)
msg = struct('delta', zeros(6, 1), 'achieved', zeros(3, 1), ...
    'residual', zeros(3, 1), 'saturated', saturated, ...
    'motor_rad_s', 800, 've_nominal_m_s', 20);
end

function bus = make_bus(trim)
bus = struct();
bus.command = trim.command;
bus.topics = aero_topic_bus();
specs = topic_specs();
for k = 1:numel(specs)
    bus.topics.register(specs{k}.name, specs{k}.spec);
end
end
