function info = test_rate_pid()
%TEST_RATE_PID 角速度 PID：P/I/D、角加速度反馈与 Topic 抗饱和

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

gains = struct('Kp', [0.35; 0.35; 0.06], 'Ki', [0.08; 0.08; 0.02], ...
    'Kd', [0.5; 0.5; 0.5], 'Kff', [0; 0; 0], ...
    'integral_limit', [0.15; 0.15; 0.05], ...
    'torque_limit', [1; 1; 1], 'saturation_tolerance', 1e-6);
params = struct('nav_topic', 'navigation/state', ...
    'rate_topic', 'control/rate_setpoint', ...
    'allocation_topic', 'control/allocation_status', ...
    'output_topic', 'control/torque_setpoint', ...
    'rate', gains, 'rates', struct('rate_hz', 400));

nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', [0; 0; 0], 'alpha', [1; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
ref = struct('omega', [0.5; 0; 0], 'alpha', zeros(3, 1));
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0, 0, 0, 'test');

% 无分配反馈：τ = Kp*0.5 - Kd*1 = 0.175 - 0.5
bus = rate_pid(0, bus, params);
tau = bus.topics.read('control/torque_setpoint', 0).data.torque;
check_that(abs(tau(1) - (0.35 * 0.5 - 0.5 * 1)) < 1e-12, ...
    'P/D 合成力矩错误：%.6f', tau(1));
check_that(isfield(bus.control_state, 'rate'), '速率 PID 未持有独立状态');

% 分配器反馈正饱和：正误差不得继续积分
bus.topics.publish('control/allocation_status', ...
    struct('delta', zeros(6, 1), 'achieved', zeros(3, 1), ...
           'residual', zeros(3, 1), 'saturated', [1; 0; 0], ...
           'motor_rad_s', 800, 've_nominal_m_s', 20), 0.01, 0.01, 0.01, 'test');
bus.topics.publish('navigation/state', nav, 0.01, 0.01, 0.01, 'test');
bus.topics.publish('control/rate_setpoint', ref, 0.01, 0.01, 0.01, 'test');
integralBefore = bus.control_state.rate.integral(1);
bus = rate_pid(0.01, bus, params);
check_that(abs(bus.control_state.rate.integral(1) - integralBefore) < 1e-15, ...
    '正饱和时积分未被冻结');

% 无饱和：积分正常增长
bus2 = make_bus(trim);
bus2.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus2.topics.publish('control/rate_setpoint', ref, 0, 0, 0, 'test');
bus2 = rate_pid(0, bus2, params);
bus2.topics.publish('navigation/state', nav, 0.01, 0.01, 0.01, 'test');
bus2.topics.publish('control/rate_setpoint', ref, 0.01, 0.01, 0.01, 'test');
bus2 = rate_pid(0.01, bus2, params);
check_that(bus2.control_state.rate.integral(1) > 0, '未饱和时积分未增长');

% 力矩限幅
gains.torque_limit = [0.01; 0.01; 0.01];
params.rate = gains;
bus2.topics.publish('navigation/state', nav, 0.02, 0.02, 0.02, 'test');
bus2.topics.publish('control/rate_setpoint', ref, 0.02, 0.02, 0.02, 'test');
bus2 = rate_pid(0.02, bus2, params);
tau = bus2.topics.read('control/torque_setpoint', 0.02).data.torque;
check_that(all(abs(tau) <= 0.01 + 1e-12), '力矩限幅失效');

info = struct('passed', true, 'summary', sprintf( ...
    'P/D 力矩 %.4f N·m，Topic 饱和冻结与限幅通过', tau(1)));

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
