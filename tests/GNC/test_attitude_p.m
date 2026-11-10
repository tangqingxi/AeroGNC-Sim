function info = test_attitude_p()
%TEST_ATTITUDE_P PX4 默认 reduced-attitude 误差、yaw weight、yaw_rate 前馈与限幅

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

attitude = struct('Kp', [2; 2; 2], 'rate_limit', [10; 10; 10], ...
    'yaw_weight', 0.4, 'ref_model', struct('enabled', false, ...
    'omega_n', 50, 'ff_gain', 0, 'ff_limit', deg2rad(100)));
params = struct('nav_topic', 'navigation/state', ...
    'attitude_topic', 'control/attitude_setpoint', ...
    'output_topic', 'control/rate_setpoint', ...
    'attitude', attitude, 'rates', struct('attitude_hz', 200));

nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', zeros(3, 1), 'alpha', zeros(3, 1), ...
    'sample_time_s', 0, 'ideal', 1);
qd = [cos(0.1); 0; 0; sin(0.1)];   % yaw 0.2 rad
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', qd, 'thrust_n', p.mass_kg * p.gravity_m_s2, ...
    'yaw', 0.2, 'yaw_rate', 0), 0, 0, 0, 'test');

bus = attitude_p(0, bus, params);
msg = bus.topics.read('control/rate_setpoint', 0);
expected = (2 / 0.4) * 2 * sin(0.4 * 0.1);
check_that(abs(msg.data.omega(3) - expected) < 1e-9, ...
    'yaw weight 默认误差输出错误：%.6f', msg.data.omega(3));
check_that(norm(msg.data.omega(1:2)) < 1e-12, '纯偏航误差产生交叉轴输出');
check_that(isfield(bus.control_state, 'attitude'), '姿态环未持有参考模型状态');

% yaw weight=1 时仍应得到相同的等效偏航增益（误差缩放与增益补偿相消）
attitude.yaw_weight = 1.0;
bus2 = make_bus(trim);
bus2.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus2.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', qd, 'thrust_n', p.mass_kg * p.gravity_m_s2, ...
    'yaw', 0.2, 'yaw_rate', 0), 0, 0, 0, 'test');
params2 = params;
params2.attitude = attitude;
bus2 = attitude_p(0, bus2, params2);
omega1 = bus2.topics.read('control/rate_setpoint', 0).data.omega(3);
check_that(abs(omega1 - msg.data.omega(3)) < 1e-3, ...
    'yaw weight 补偿改变了等效偏航增益');

% yaw_rate 前馈：姿态无误差时，速率参考应包含该角速度
bus3 = make_bus(trim);
bus3.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus3.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', [1; 0; 0; 0], 'thrust_n', p.mass_kg * p.gravity_m_s2, ...
    'yaw', 0, 'yaw_rate', 0.3), 0, 0, 0, 'test');
bus3 = attitude_p(0, bus3, params);
omegaYaw = bus3.topics.read('control/rate_setpoint', 0).data.omega(3);
check_that(abs(omegaYaw - 0.3) < 1e-9, 'yaw_rate 未作为角速度前馈传递');

% 角速度限幅
attitude.rate_limit = [0.1; 0.1; 0.1];
attitude.yaw_weight = 0.4;
attitude.ref_model.enabled = false;
params3 = params;
params3.attitude = attitude;
bus4 = make_bus(trim);
bus4.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus4.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', [cos(0.25); 0; 0; sin(0.25)], ...
    'thrust_n', p.mass_kg * p.gravity_m_s2, 'yaw', 0.5, 'yaw_rate', 0), ...
    0, 0, 0, 'test');
bus4 = attitude_p(0, bus4, params3);
omegaLimited = bus4.topics.read('control/rate_setpoint', 0).data.omega;
check_that(all(abs(omegaLimited) <= 0.1 + 1e-12), '角速度参考未限幅');

% 参考模型启用：首拍锁定，第二拍平滑跟踪，状态保持单位四元数
attitude.rate_limit = [10; 10; 10];
attitude.ref_model.enabled = true;
params4 = params;
params4.attitude = attitude;
bus5 = make_bus(trim);
bus5.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus5.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', [1; 0; 0; 0], 'thrust_n', p.mass_kg * p.gravity_m_s2, ...
    'yaw', 0, 'yaw_rate', 0), 0, 0, 0, 'test');
bus5 = attitude_p(0, bus5, params4);
bus5.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', [cos(0.2); 0; 0; sin(0.2)], ...
    'thrust_n', p.mass_kg * p.gravity_m_s2, 'yaw', 0.4, 'yaw_rate', 0), ...
    0.005, 0.005, 0.005, 'test');
bus5.topics.publish('navigation/state', nav, 0.005, 0.005, 0.005, 'test');
bus5 = attitude_p(0.005, bus5, params4);
qRef = bus5.control_state.attitude.q_ref;
check_that(abs(norm(qRef) - 1) < 1e-12, '参考模型四元数未归一化');
check_that(abs(qRef(1) - 1) > 1e-9, '参考模型未对阶跃进行平滑');

info = struct('passed', true, 'summary', sprintf( ...
    '默认误差 %.4f rad/s、yaw_weight 补偿、yaw_rate=0.3 前馈、限幅与参考模型通过', ...
    expected));

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
