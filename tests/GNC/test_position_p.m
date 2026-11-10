function info = test_position_p()
%TEST_POSITION_P 位置 P：位置误差到速度参考、限幅、无内部状态

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

nav = struct('position', [1; 2; 3], 'velocity', [0; 0; 0], ...
    'acceleration', [0; 0; 0], 'quaternion', [1; 0; 0; 0], ...
    'omega', [0; 0; 0], 'alpha', [0; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
ref = struct('position', [1.5; 1.0; 3.0], 'velocity', [0; 0; 0], ...
    'acceleration', [0.1; 0; 0], 'yaw', 0, 'yaw_rate', 0);
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('guidance/reference', ref, 0, 0, 0, 'test');

params = struct('nav_topic', 'navigation/state', ...
    'reference_topic', 'guidance/reference', ...
    'output_topic', 'control/velocity_setpoint', ...
    'position', struct('Kp', [1; 2; 1]), ...
    'velocity', struct('limit_xy', 10, 'limit_up', 10, 'limit_down', 10));

bus = position_p(0, bus, params);
msg = bus.topics.read('control/velocity_setpoint', 0);
expected = [0.5; -2.0; 0];
check_that(norm(msg.data.velocity - expected) < 1e-12, ...
    '速度参考错误：%s', mat2str(msg.data.velocity.', 4));
check_that(norm(msg.data.acceleration_ff - ref.acceleration) < 1e-12, ...
    '前馈加速度未透传');
check_that(~isfield(bus, 'control_state'), '位置 P 不应创建控制状态');

% 限幅：水平与上下分开
ref.position = [100; 0; 0];
nav.sample_time_s = 1;
bus.topics.publish('navigation/state', nav, 1, 1, 1, 'test');
bus.topics.publish('guidance/reference', ref, 1, 1, 1, 'test');
bus = position_p(1, bus, params);
msg = bus.topics.read('control/velocity_setpoint', 1);
check_that(norm(msg.data.velocity(1:2)) <= 10 + 1e-12, '水平速度未限幅');
check_that(msg.data.velocity(3) <= 10 + 1e-12, '垂直速度未限幅');

info = struct('passed', true, 'summary', '位置误差→速度参考、前馈透传与限幅通过');

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
