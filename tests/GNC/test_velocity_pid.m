function info = test_velocity_pid()
%TEST_VELOCITY_PID 速度 PID：P/I/D、垂直积分、Tracking Anti-Windup 与推力限幅

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);

gains = struct();
gains.Kp = [1.2; 1.2; 2.0];
gains.Ki = [0.1; 0.1; 0.3];
gains.Kd = [0.5; 0.5; 0.5];
gains.integral_limit = [5; 5; 5];

params = make_params(gains, 1000, 5, 300, deg2rad(45), trim.thrust_n);
bus = make_bus(trim);

nav = struct('position', [0; 0; 0], 'velocity', [0; 0; 0], ...
    'acceleration', [2; 0; 0], 'quaternion', [1; 0; 0; 0], ...
    'omega', [0; 0; 0], 'alpha', [0; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
setpoint = struct('velocity', [1; 0; 0], 'acceleration_ff', [0; 0; 0], ...
    'yaw', 0, 'yaw_rate', 0);
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/velocity_setpoint', setpoint, 0, 0, 0, 'test');

% P + D：a = Kp*1 - Kd*2 = 1.2 - 1.0 = 0.2
bus = velocity_pid(0, bus, params);
attitude = bus.topics.read('control/attitude_setpoint', 0);
expectedAcceleration = 1.2 * 1 - 0.5 * 2;
tilt = 2 * acos(min(1, abs(attitude.data.quaternion(1))));
check_that(abs(tilt - atan2(expectedAcceleration, p.gravity_m_s2)) < 1e-9, ...
    'P+D 合成加速度与期望姿态不一致');
check_that(isfield(bus, 'control_state') && isfield(bus.control_state, 'velocity'), ...
    '速度 PID 未持有独立状态');
check_that(abs(attitude.data.yaw - 0) < 1e-12 && abs(attitude.data.yaw_rate) < 1e-12, ...
    'yaw/yaw_rate 未进入姿态设定');

% 纯 P 对照：Kd=0 时倾角更大
cleanBus = make_bus(trim);
cleanGains = gains;
cleanGains.Kd = [0; 0; 0];
cleanParams = make_params(cleanGains, 1000, 5, 300, deg2rad(45), trim.thrust_n);
cleanBus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
cleanBus.topics.publish('control/velocity_setpoint', setpoint, 0, 0, 0, 'test');
cleanBus = velocity_pid(0, cleanBus, cleanParams);
qNoD = cleanBus.topics.read('control/attitude_setpoint', 0).data.quaternion;
tiltNoD = 2 * acos(min(1, abs(qNoD(1))));
check_that(tilt < tiltNoD, '非零 Kd 未减小期望倾角（D 项无效）');

% 垂直积分钳位 ±g
bus.control_state.velocity.integral(3) = 100;
bus.topics.publish('navigation/state', nav, 0.01, 0.01, 0.01, 'test');
bus.topics.publish('control/velocity_setpoint', setpoint, 0.01, 0.01, 0.01, 'test');
bus = velocity_pid(0.01, bus, params);
check_that(abs(bus.control_state.velocity.integral(3)) <= p.gravity_m_s2 + 1e-12, ...
    '垂直积分未钳位到 ±g');

% Tracking Anti-Windup：总推力饱和时应修正水平误差，积分不应继续正增长
satBus = make_bus(trim);
satParams = make_params(gains, 10, 1, 0, deg2rad(45), trim.thrust_n);
satSetpoint = struct('velocity', [5; 0; 0], 'acceleration_ff', [0; 0; 0], ...
    'yaw', 0, 'yaw_rate', 0);
satBus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
satBus.topics.publish('control/velocity_setpoint', satSetpoint, 0, 0, 0, 'test');
satBus = velocity_pid(0, satBus, satParams);
satAttitude = satBus.topics.read('control/attitude_setpoint', 0);
check_that(satAttitude.data.thrust_n <= 10 + 1e-9, '总推力超过上限');
check_that(satBus.control_state.velocity.integral(1) < ...
    gains.Ki(1) * 5 * 0.01 - 1e-12, ...
    'Tracking Anti-Windup 未衰减饱和误差积分');

info = struct('passed', true, 'summary', sprintf( ...
    'P/I/D 与测量加速度反馈通过；D 使倾角 %.3f°→%.3f°；垂直钳位/ARW 通过', ...
    rad2deg(tiltNoD), rad2deg(tilt)));

end

% ===========================================================================
function params = make_params(gains, maxThrust, minThrust, margin, maxTilt, hoverThrust)
params = struct('nav_topic', 'navigation/state', ...
    'setpoint_topic', 'control/velocity_setpoint', ...
    'output_topic', 'control/attitude_setpoint', ...
    'velocity', gains, ...
    'max_thrust_n', maxThrust, 'min_thrust_n', minThrust, ...
    'horizontal_thrust_margin_n', margin, 'max_tilt_rad', maxTilt, ...
    'hover_thrust_n', hoverThrust, ...
    'mass_kg', 2.1, 'gravity_m_s2', 9.788, ...
    'rates', struct('velocity_hz', 100));
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
