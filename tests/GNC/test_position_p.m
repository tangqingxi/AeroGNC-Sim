function info = test_position_p()
%TEST_POSITION_P 位置 P：误差、前馈、PX4 水平限幅优先级与垂直限幅

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
bus = make_bus(trim);

nav = struct('position', [1; 2; 3], 'velocity', [0; 0; 0], ...
    'acceleration', [0; 0; 0], 'quaternion', [1; 0; 0; 0], ...
    'omega', [0; 0; 0], 'alpha', [0; 0; 0], ...
    'sample_time_s', 0, 'ideal', 1);
ref = struct('position', [1.5; 1.0; 3.0], 'velocity', [0; 0; 0], ...
    'acceleration', [0.1; 0; 0], 'yaw', 0.2, 'yaw_rate', 0.1);
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('guidance/reference', ref, 0, 0, 0, 'test');

params = struct('nav_topic', 'navigation/state', ...
    'reference_topic', 'guidance/reference', ...
    'output_topic', 'control/velocity_setpoint', ...
    'position', struct('Kp', [1; 2; 1], ...
        'velocity_limit_xy', 10, 'velocity_limit_up', 10, ...
        'velocity_limit_down', 10));

bus = position_p(0, bus, params);
msg = bus.topics.read('control/velocity_setpoint', 0);
expected = [0.5; -2.0; 0];
check_that(norm(msg.data.velocity - expected) < 1e-12, ...
    '速度参考错误：%s', mat2str(msg.data.velocity.', 4));
check_that(norm(msg.data.acceleration_ff - ref.acceleration) < 1e-12, ...
    '前馈加速度未透传');
check_that(abs(msg.data.yaw - 0.2) < 1e-12 && abs(msg.data.yaw_rate - 0.1) < 1e-12, ...
    'yaw/yaw_rate 未透传');
check_that(~isfield(bus, 'control_state'), '位置 P 不应创建控制状态');

% PX4 优先级：位置反馈分量优先，前馈不能把合成方向拉偏
ref.position = [3; 4; 3];
ref.velocity = [1; 0; 0];
ref.yaw_rate = 0.2;
nav.sample_time_s = 1;
bus.topics.publish('navigation/state', nav, 1, 1, 1, 'test');
bus.topics.publish('guidance/reference', ref, 1, 1, 1, 'test');
params.position.velocity_limit_xy = 1;
bus = position_p(1, bus, params);
msg = bus.topics.read('control/velocity_setpoint', 1);
expectedDir = ([1; 2] .* ([3; 4] - [1; 2]));
expectedDir = expectedDir / norm(expectedDir);
check_that(norm(msg.data.velocity(1:2) / max(norm(msg.data.velocity(1:2)), 1e-12) ...
    - expectedDir) < 1e-12, '水平限幅未保持位置反馈方向优先级');
check_that(norm(msg.data.velocity(1:2)) <= 1 + 1e-12, '水平速度未限幅');

% 垂直上下限独立
ref.position(3) = 100;
bus.topics.publish('guidance/reference', ref, 2, 2, 2, 'test');
nav.sample_time_s = 2;
bus.topics.publish('navigation/state', nav, 2, 2, 2, 'test');
params.position.velocity_limit_up = 0.5;
params.position.velocity_limit_down = 0.2;
bus = position_p(2, bus, params);
msg = bus.topics.read('control/velocity_setpoint', 2);
check_that(abs(msg.data.velocity(3) - 0.2) < 1e-12, '下降速度限幅错误');

info = struct('passed', true, 'summary', ...
    '位置误差→速度参考、前馈/yaw 透传、PX4 水平优先与垂直限幅通过');

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
