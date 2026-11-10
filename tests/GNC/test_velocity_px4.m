function info = test_velocity_px4()
%TEST_VELOCITY_PX4 Velocity 的 PX4 解耦、hover-thrust、余量与力一致性

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
hover = trim.thrust_n;
maxThrust = p.duct.k_T0 * p.motor_device.omega_range_rad_s(2)^2;
minThrust = 0.12 * maxThrust;

base = struct('Kp', zeros(3, 1), 'Ki', zeros(3, 1), 'Kd', zeros(3, 1), ...
    'integral_limit', [1; 1; 1]);

% 1) 悬停映射：az=0，水平=0 → q=I，T=hover
[target, q, thrust] = run_case(base, zeros(3, 1), maxThrust, minThrust, 0, ...
    p, hover, deg2rad(45));
check_that(norm(q - [1; 0; 0; 0]) < 1e-12, '悬停姿态不是单位四元数');
check_that(abs(thrust - hover) < 1e-12, '悬停推力不等于 trim.thrust_n');
check_that(norm(-thrust * z_from_q(q) - [0; 0; -hover]) < 1e-12, ...
    '悬停力重建不一致');

% 2) 垂直解耦：改变 az 不改变水平加速度决定的姿态方向
a0 = [0.5; 0; 0];
[~, q0] = run_case(base, a0, maxThrust, minThrust, 0, p, hover, deg2rad(45));
[~, qz] = run_case(base, [0.5; 0; 2.0], maxThrust, minThrust, 0, p, hover, deg2rad(45));
check_that(dot(z_from_q(q0), z_from_q(qz)) > 1 - 1e-12, ...
    '垂直加速度改变了 decouple 模式的姿态方向');

% 3) PX4 hover-thrust SI 映射
[~, qMap, tMap] = run_case(base, [0; 0; 2.0], maxThrust, minThrust, 0, ...
    p, hover, deg2rad(45));
expectedT = hover * (1 - 2.0 / p.gravity_m_s2);
check_that(abs(tMap - expectedT) < 1e-12, ...
    'hover-thrust SI 映射错误：%.6f != %.6f', tMap, expectedT);
check_that(norm(qMap - [1; 0; 0; 0]) < 1e-12, '纯垂直加速度产生姿态偏转');

% 4) 无饱和时，从 q 与 T 重建的力等于独立公式
aTest = [1.2; -0.4; 0.3];
[~, qTest, tTest] = run_case(base, aTest, 1000, 1, 0, p, hover, deg2rad(45));
z = [-aTest(1); -aTest(2); p.gravity_m_s2];
z = z / norm(z);
expectedF = -z * (hover * (1 - aTest(3) / p.gravity_m_s2) / z(3));
rebuiltF = -tTest * z_from_q(qTest);
check_that(norm(rebuiltF - expectedF) < 1e-9, ...
    '姿态/推力重建力与独立期望不一致：%.3e', norm(rebuiltF - expectedF));

% 5) 水平余量：小水平需求不能无条件预留完整 margin
[~, qMargin, tMargin] = run_case(base, [0.1; 0; 0], 21, 1, 5, p, hover, deg2rad(45));
zMargin = z_from_q(qMargin);
verticalForce = tMargin * zMargin(3);
check_that(verticalForce > 20.5, ...
    '小水平需求仍预留了完整水平余量：%.3f N', verticalForce);
check_that(tMargin <= 21 + 1e-9, '小水平需求总推力超过 max');

% 6) 最大/最小推力
[~, ~, tMax] = run_case(base, [0; 0; -100], maxThrust, minThrust, 0, ...
    p, hover, deg2rad(45));
[~, ~, tMin] = run_case(base, [0; 0; 100], maxThrust, minThrust, 0, ...
    p, hover, deg2rad(45));
check_that(tMax <= maxThrust + 1e-9, '总推力超过物理最大推力');
check_that(tMin >= minThrust - 1e-9, '总推力低于最小推力');

% 7) 倾角限制
[~, qTilt, ~] = run_case(base, [50; 0; 0], 1000, 1, 0, p, hover, deg2rad(10));
zTilt = z_from_q(qTilt);
tilt = acos(min(1, max(-1, zTilt(3))));
check_that(tilt <= deg2rad(10) + 1e-9, '最终姿态超过最大倾角：%.3f°', rad2deg(tilt));

% 8) 垂直积分抗饱和：最小/最大推力方向
awGains = struct('Kp', [1; 1; 1], 'Ki', [0; 0; 1], 'Kd', zeros(3, 1), ...
    'integral_limit', [1; 1; 1]);
params = make_params(awGains, 100, 10, 0, p, hover, deg2rad(45));
params.min_thrust_n = 10;
% 最小推力：az=100 → T=10，e_z>0
[integralMin] = run_integral_case(params, [0; 0; 100], [0; 0; 1]);
check_that(abs(integralMin) < 1e-12, '最小推力正误差未冻结');
% 最大推力：az=-100 → T=10，e_z<0
params.max_thrust_n = 10;
params.min_thrust_n = 1;
[integralMax] = run_integral_case(params, [0; 0; -100], [0; 0; -1]);
check_that(abs(integralMax) < 1e-12, '最大推力负误差未冻结');

info = struct('passed', true, 'summary', sprintf( ...
    'PX4 解耦/映射/余量/限幅/力一致性通过：hover=%.3f N, Tmax=%.3f N', ...
    hover, maxThrust));

end

% ===========================================================================
function [target, q, thrust] = run_case(gains, aFF, maxT, minT, margin, p, hover, tilt)
params = make_params(gains, maxT, minT, margin, p, hover, tilt);
bus = make_bus();
nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', zeros(3, 1), 'alpha', zeros(3, 1), ...
    'sample_time_s', 0, 'ideal', 1);
setpoint = struct('velocity', zeros(3, 1), 'acceleration_ff', aFF, ...
    'yaw', 0, 'yaw_rate', 0);
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/velocity_setpoint', setpoint, 0, 0, 0, 'test');
bus = velocity_pid(0, bus, params);
msg = bus.topics.read('control/attitude_setpoint', 0);
q = msg.data.quaternion;
thrust = msg.data.thrust_n;
target = struct('bus', bus);
end

function integral = run_integral_case(params, aFF, vSpZ)
bus = make_bus();
nav = struct('position', zeros(3, 1), 'velocity', zeros(3, 1), ...
    'acceleration', zeros(3, 1), 'quaternion', [1; 0; 0; 0], ...
    'omega', zeros(3, 1), 'alpha', zeros(3, 1), ...
    'sample_time_s', 0, 'ideal', 1);
setpoint = struct('velocity', vSpZ, 'acceleration_ff', aFF, ...
    'yaw', 0, 'yaw_rate', 0);
bus.topics.publish('navigation/state', nav, 0, 0, 0, 'test');
bus.topics.publish('control/velocity_setpoint', setpoint, 0, 0, 0, 'test');
bus = velocity_pid(0, bus, params);
integral = bus.control_state.velocity.integral(3);
end

function params = make_params(gains, maxT, minT, margin, p, hover, tilt)
params = struct('nav_topic', 'navigation/state', ...
    'setpoint_topic', 'control/velocity_setpoint', ...
    'output_topic', 'control/attitude_setpoint', ...
    'velocity', gains, 'max_thrust_n', maxT, 'min_thrust_n', minT, ...
    'horizontal_thrust_margin_n', margin, 'max_tilt_rad', tilt, ...
    'hover_thrust_n', hover, 'mass_kg', p.mass_kg, ...
    'gravity_m_s2', p.gravity_m_s2, ...
    'rates', struct('velocity_hz', 100));
end

function bus = make_bus()
bus = struct();
bus.command = struct();
bus.topics = aero_topic_bus();
specs = topic_specs();
for k = 1:numel(specs)
    bus.topics.register(specs{k}.name, specs{k}.spec);
end
end

function z = z_from_q(q)
R = quat_to_rotation(q);
z = R * [0; 0; 1];
z = z / norm(z);
end
