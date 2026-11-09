function info = test_shc09_dynamics()
%TEST_SHC09_DYNAMICS 刚体动力学、气动符号、配平与四元数运动学

src = struct('type', 'fixture', 'file', '', 'tables', shc09_aero_fixture());
model = shc09_model(struct('aero_source', src));
p = model.parameters;
s = model.state_layout;

% ---- 自由落体：无推力时加速度为 +g（NED 向下） ----------------------
x = zeros(s.nx, 1);
x(s.quaternion) = [1; 0; 0; 0];
cmd0 = struct('servos', zeros(6, 1), 'motor', 0);
dx = model.dynamics(0, x, cmd0, struct());
check_that(max(abs(dx(s.velocity) - [0; 0; p.gravity_m_s2])) < 1e-12, ...
    '自由落体加速度方向错误：%s', mat2str(dx(s.velocity).'));
check_that(max(abs(dx(s.quaternion))) < 1e-12, '无角速度时四元数导数非零');

% ---- 旋翼正转：推力沿 -Z，反扭矩沿 -Z ------------------------------
x(s.motor) = 700;
cmd = struct('servos', zeros(6, 1), 'motor', 700);
dx = model.dynamics(0, x, cmd, struct());
check_that(dx(6) < p.gravity_m_s2, '旋翼推力未抵消重力（应使 az < g）');

[F_b, M_b, ~, detail] = shc09_aero_force([0; 0; 0], [0; 0; 0], zeros(6, 1), 700, p, model.aero);
check_that(F_b(3) < 0, '旋翼推力不是沿 -Z');
check_that(detail.M_fan < 0, '旋翼空气动力反扭矩不是沿 -Z');
check_that(detail.M_sta > 0, '固定片反扭矩方向应与旋翼反扭矩相反');

% ---- 陀螺力矩符号 ---------------------------------------------------
omega = [0.1; 0.2; 0];
[~, ~, ~, detail] = shc09_aero_force([0; 0; 0], omega, zeros(6, 1), 800, p, model.aero);
expectedGyro = -p.rotor_direction * p.fan_inertia_kg_m2 * 800 * [omega(2); -omega(1); 0];
check_that(max(abs(detail.M_gyro - expectedGyro)) < 1e-12, '陀螺力矩符号错误');

% ---- 舵面力矩：数值 Jacobian 与解析 B_tau 一致 ---------------------
trim = model.trim;
Ve = trim.exit_speed_m_s;
[~, B_tau] = shc09_control_surfaces(zeros(6, 1), Ve, p);
eps = 1e-7;
[~, M0] = shc09_aero_force([0; 0; 0], [0; 0; 0], zeros(6, 1), trim.omega_rad_s, p, model.aero);
for i = 1:6
    delta = zeros(6, 1);
    delta(i) = eps;
    [~, Mi] = shc09_aero_force([0; 0; 0], [0; 0; 0], delta, trim.omega_rad_s, p, model.aero);
    numeric = (Mi - M0) / eps;
    check_that(max(abs(numeric - B_tau(:, i))) < 1e-6, ...
        '第 %d 个舵面 Jacobian 与 B_tau 不一致', i);
end

% ---- 六舵同向偏转只产生偏航力矩 ------------------------------------
[~, ~, B_M] = shc09_control_surfaces(ones(6, 1), Ve, p);
check_that(max(abs(B_M(1:2, :))) >= 0, 'B_M 读取异常');
check_that(abs(sum(B_M(1, :))) < 1e-12 && abs(sum(B_M(2, :))) < 1e-12, ...
    '六舵同向偏转不应产生滚转/俯仰力矩');
check_that(abs(sum(B_M(3, :)) - 6) < 1e-12, '偏航通道求和不为 6');

% ---- 悬停配平残差 ---------------------------------------------------
dx = model.dynamics(0, model.x0, model.initial_command, struct());
check_that(norm(dx(s.velocity)) < 1e-9, '悬停配平平动加速度 %.3e 非零', norm(dx(s.velocity)));
check_that(norm(dx(s.omega)) < 1e-9, '悬停配平角加速度 %.3e 非零', norm(dx(s.omega)));
check_that(norm(dx(s.servo)) < 1e-12 && abs(dx(s.motor)) < 1e-12, ...
    '配平状态下执行器导数不为零');
check_that(abs(trim.delta_deg - 7.31) < 0.01, '配平舵角 %.4f° 与参考约 7.31° 不符', trim.delta_deg);

% ---- 四元数运动学与范数投影 ----------------------------------------
qdot = shc09_quat_kinematics([1; 0; 0; 0], [0; 0; 1]);
check_that(max(abs(qdot - [0; 0; 0; 0.5])) < 1e-15, '四元数运动学符号错误');

xq = zeros(s.nx, 1);
xq(s.quaternion) = [2; 0; 0; 0];
xq(s.servo) = 10;
xq(s.motor) = 5000;
xp = shc09_project(xq, p);
check_that(abs(norm(xp(s.quaternion)) - 1) < 1e-12, '状态投影未归一化四元数');
check_that(all(abs(xp(s.servo)) <= p.servo_deflection_limit_rad + 1e-15), '状态投影未限制舵角');
check_that(xp(s.motor) <= p.motor_device.omega_range_rad_s(2), '状态投影未限制转速');

% ---- 气动表越界必须显式处理 ----------------------------------------
vDescend = [0; 0; 5];
check_error(@() shc09_aero_force(vDescend, [0; 0; 0], zeros(6, 1), 700, p, model.aero), ...
    'shc09:aero:outOfDomain', '气动表越界');
pClip = p;
pClip.aero_domain_policy = 'clip';
[Mclip] = shc09_aero_force(vDescend, [0; 0; 0], zeros(6, 1), 700, pClip, model.aero);
check_that(all(isfinite(Mclip)), 'clip 策略下结果应为有限值');

info = struct('passed', true, 'summary', sprintf( ...
    '自由落体/推力与反扭矩方向/陀螺/Jacobian/配平(δ=%.3f°)/四元数/越界均通过', trim.delta_deg));

end
