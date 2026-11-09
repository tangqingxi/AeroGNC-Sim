function info = test_gnc_math()
%TEST_GNC_MATH GNC 数学与分配器的最小解析检查

p = shc09_parameters();
trim = shc09_hover_equilibrium(p);
geometry = shc09_control_surface_geometry(p);

% ---- 四元数与旋转往返 ------------------------------------------------
q = [cos(0.15); sin(0.15) * [0.3; -0.4; 0.5] / norm([0.3; -0.4; 0.5])];
q = q / norm(q);
R = gnc_quat_to_rotation(q);
q2 = gnc_rotation_to_quat(R);
check_that(norm(q - q2) < 1e-12 || norm(q + q2) < 1e-12, '四元数往返误差过大');
check_that(norm(R * R.' - eye(3)) < 1e-12, '旋转矩阵不正交');

% ---- 姿态误差：偏航 0.2 rad 的期望向量 ------------------------------
qDesired = [cos(0.1); 0; 0; sin(0.1)];
e = gnc_quat_error([1; 0; 0; 0], qDesired);
check_that(abs(e(3) - 2 * sin(0.1)) < 1e-12, '姿态误差 z 分量错误');
check_that(norm(e(1:2)) < 1e-12, '姿态误差出现交叉轴分量');

% ---- 推力矢量与倾角限制 ---------------------------------------------
[qHover, thrust, zBody] = gnc_thrust_to_attitude(zeros(3, 1), 0, ...
    p.mass_kg, p.gravity_m_s2, deg2rad(25));
check_that(abs(thrust - p.mass_kg * p.gravity_m_s2) < 1e-9, '悬停推力不等于 mg');
check_that(norm(qHover - [1; 0; 0; 0]) < 1e-9, '悬停姿态不是单位四元数');
check_that(norm(zBody - [0; 0; 1]) < 1e-9, '悬停机体 z 轴错误');

[qTilt, ~, zTilt] = gnc_thrust_to_attitude([5; 0; 0], 0, ...
    p.mass_kg, p.gravity_m_s2, deg2rad(25));
tilt = acos(min(1, max(-1, zTilt(3))));
check_that(tilt <= deg2rad(25) + 1e-9, '倾角限制失效：%.3f rad', tilt);
check_that(~isempty(qTilt), '倾角限制后四元数为空');

% ---- PINV 分配：零力矩回到配平，已知力矩可复现 ----------------------
bus = struct();
bus.command = trim.command;
bus.gnc = struct();
bus.topics = aero_topic_bus();
specs = gnc_topic_specs();
for k = 1:numel(specs)
    bus.topics.register(specs{k}.name, specs{k}.spec);
end
bus.topics.publish('control/attitude_setpoint', ...
    struct('quaternion', [1; 0; 0; 0], 'thrust_n', trim.thrust_n), 0, 0, 0, 'test');
bus.topics.publish('control/torque_setpoint', struct('torque', zeros(3, 1)), 0, 0, 0, 'test');

params = allocation_params(p, trim, geometry);
bus = gnc_allocate_pinv_step(0, bus, params);
check_that(max(abs(bus.command.servos - trim.delta_rad)) < 1e-9, ...
    '零力矩时未回到配平舵角');
check_that(abs(bus.command.motor - trim.omega_rad_s) < 1e-6, ...
    '悬停推力对应的电机转速错误');

Vec = trim.exit_speed_m_s;
Btau = p.servo_torque_coefficient * Vec^2 * ...
    diag([p.l1_m, p.l1_m, p.l2_m]) * geometry.B_M;
tau = [0.05; -0.03; 0.01];
bus.topics.publish('control/torque_setpoint', struct('torque', tau), 1, 1, 1, 'test');
bus = gnc_allocate_pinv_step(1, bus, params);
achieved = Btau * (bus.command.servos - trim.delta_rad * ones(6, 1));
check_that(norm(achieved - tau) < 1e-6, ...
    'PINV 分配未复现期望力矩：误差 %.3e', norm(achieved - tau));

info = struct('passed', true, 'summary', sprintf( ...
    '四元数往返/姿态误差/推力矢量/倾角限制/PINV 分配通过（力矩误差 %.1e）', ...
    norm(achieved - tau)));

end

% ===========================================================================
function params = allocation_params(p, trim, geometry)
params = struct( ...
    'torque_topic', 'control/torque_setpoint', ...
    'attitude_topic', 'control/attitude_setpoint', ...
    'status_topic', 'control/allocation_status', ...
    'B_M', geometry.B_M, ...
    'l1_m', p.l1_m, 'l2_m', p.l2_m, ...
    'servo_torque_coefficient', p.servo_torque_coefficient, ...
    'trim_delta_rad', trim.delta_rad, ...
    'delta_limit', p.servo_effective_range_rad, ...
    'diffuser_ratio', p.diffuser_ratio, ...
    'air_density', p.air_density_kg_m3, ...
    'disc_area', p.disc_area_m2, ...
    'k_T0', p.duct.k_T0, ...
    'hover_thrust_n', trim.thrust_n, ...
    'max_thrust_n', p.duct.k_T0 * p.motor_device.omega_range_rad_s(2)^2, ...
    'motor_min_rad_s', p.motor_device.omega_range_rad_s(1), ...
    'motor_max_rad_s', p.motor_device.omega_range_rad_s(2), ...
    've_min_m_s', 1e-3, ...
    'saturation_tolerance', 1e-6);
end
