function info = test_velocity_pid()
%TEST_VELOCITY_PID 速度 PID：P/I/D、测量加速度反馈与积分限幅

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

gains = struct();
gains.Kp = [1.2; 1.2; 2.0];
gains.Ki = [0.1; 0.1; 0.3];
gains.Kd = [0.5; 0.5; 0.5];
gains.integral_limit = [5; 5; 5];
gains.limit_xy = 10;
gains.limit_up = 10;
gains.limit_down = 10;

params = struct('nav_topic', 'navigation/state', ...
    'setpoint_topic', 'control/velocity_setpoint', ...
    'output_topic', 'control/attitude_setpoint', ...
    'velocity', gains, 'acceleration_limit', [100; 100; 100], ...
    'max_tilt_rad', deg2rad(45), 'mass_kg', p.mass_kg, ...
    'gravity_m_s2', p.gravity_m_s2, 'yaw', 0, ...
    'rates', struct('velocity_hz', 100));

nav = struct('position', [0; 0; 0], 'velocity', [0; 0; 0], ...
    'acceleration', [2; 0; 0], 'quaternion', [1; 0; 0; 0], ...
    'omega', [0; 0; 0], 'alpha', [0; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
setpoint = struct('velocity', [1; 0; 0], 'acceleration_ff', [0; 0; 0]);
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

% 去除测量加速度后，输出应变大（证明 D 项参与计算）
nav.acceleration = [0; 0; 0];
bus.topics.publish('navigation/state', nav, 0.01, 0.01, 0.01, 'test');
bus.topics.publish('control/velocity_setpoint', setpoint, 0.01, 0.01, 0.01, 'test');
bus = velocity_pid(0.01, bus, params);
check_that(abs(bus.control_state.velocity.integral(1)) > 0, '积分项未更新');

% 纯 P 对照：a = 1.2（用新总线，Kd=0）
cleanBus = make_bus(trim);
cleanGains = gains;
cleanGains.Kd = [0; 0; 0];
cleanParams = params;
cleanParams.velocity = cleanGains;
nav.acceleration = [2; 0; 0];
cleanBus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
cleanBus.topics.publish('control/velocity_setpoint', setpoint, 0, 0, 0, 'test');
cleanBus = velocity_pid(0, cleanBus, cleanParams);
qNoD = cleanBus.topics.read('control/attitude_setpoint', 0).data.quaternion;
tiltNoD = 2 * acos(min(1, abs(qNoD(1))));
check_that(tilt < tiltNoD, '非零 Kd 未减小期望倾角（D 项无效）');

% 积分限幅
gains.integral_limit = [1e-4; 1e-4; 1e-4];
params.velocity = gains;
for k = 1:20
    t = 0.02 + k * 0.01;
    bus.topics.publish('navigation/state', nav, t, t, t, 'test');
    bus.topics.publish('control/velocity_setpoint', setpoint, t, t, t, 'test');
    bus = velocity_pid(t, bus, params);
end
check_that(all(abs(bus.control_state.velocity.integral) <= 1e-4 + 1e-12), ...
    '速度积分未限幅');

info = struct('passed', true, 'summary', sprintf( ...
    'P/I/D 与测量加速度反馈通过；D 使倾角 %.3f°→%.3f°', ...
    rad2deg(tiltNoD), rad2deg(tilt)));

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
