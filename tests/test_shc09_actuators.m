function info = test_shc09_actuators()
%TEST_SHC09_ACTUATORS 设备型号参数、通用执行器动力学与限幅/速率约束

% ---- 设备型号参数可由 Aircraft 装配加载 ------------------------------
servo = ServoDevices.KST_X08H.parameters();
motor = MotorDevices.MAD_M6C06_320KV_V2.parameters();
check_that(strcmp(servo.device_class, 'servo'), 'KST 参数类别错误');
check_that(strcmp(motor.device_class, 'motor'), 'MAD 参数类别错误');
check_that(max(abs(servo.range_rad - deg2rad([-40 40]))) < 1e-12, 'KST 行程参数错误');
check_that(servo.tau_s > 0 && servo.rate_limit_rad_s > 0, 'KST 动态参数非正');
check_that(numel(motor.omega_range_rad_s) == 2, 'MAD 转速范围格式错误');

% ---- 通用舵机动力学：一阶解析响应 --------------------------------
params = struct('tau_s', 0.1, 'rate_limit_rad_s', Inf, 'range_rad', [-1, 1]);
opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
[t, d] = ode45(@(tt, dd) aero_servo_dynamics(dd, 0.5, params), [0, 0.3], 0, opts);
expected = 0.5 * (1 - exp(-3));
err = abs(d(end) - expected);
check_that(err < 1e-9, '舵机一阶响应误差 %.3e 超过 1e-9', err);

% ---- 速率限制 -------------------------------------------------------
rateParams = struct('tau_s', 0.001, 'rate_limit_rad_s', 1, 'range_rad', [-1, 1]);
[t, d] = ode45(@(tt, dd) aero_servo_dynamics(dd, 1, rateParams), [0, 0.1], 0);
check_that(abs(d(end) - 0.1) < 1e-4, '速率限制下 0.1 s 应近似 0.1 rad，实际 %.6f', d(end));

% ---- 机械行程 -------------------------------------------------------
limitParams = struct('tau_s', 0.01, 'rate_limit_rad_s', Inf, 'range_rad', [-0.5, 0.5]);
dd = aero_servo_dynamics(0, 10, limitParams);
check_that(dd > 0, '目标超出上限时导数方向错误');
ddAtLimit = aero_servo_dynamics(0.5, 10, limitParams);
check_that(ddAtLimit == 0, '到达机械上限后导数未置零');

% ---- 参数变更不要求改动力学：不同 tau 得到不同响应 ----------------
slow = struct('tau_s', 0.5, 'rate_limit_rad_s', Inf, 'range_rad', [-1, 1]);
fast = struct('tau_s', 0.05, 'rate_limit_rad_s', Inf, 'range_rad', [-1, 1]);
dSlow = ode45(@(tt, dd) aero_servo_dynamics(dd, 1, slow), [0, 0.2], 0);
dFast = ode45(@(tt, dd) aero_servo_dynamics(dd, 1, fast), [0, 0.2], 0);
check_that(dFast.y(end) > dSlow.y(end), '较小时间常数应响应更快');

% ---- 通用电机动力学 ------------------------------------------------
mParams = struct('tau_s', 0.2, 'omega_range_rad_s', [0, 1000], 'accel_limit_rad_s2', Inf);
[t, w] = ode45(@(tt, ww) aero_motor_dynamics(ww, 500, mParams), [0, 0.4], 0, opts);
expectedW = 500 * (1 - exp(-2));
errW = abs(w(end) - expectedW);
check_that(errW < 1e-8, '电机一阶响应误差 %.3e 超过 1e-8', errW);

% ---- 电机转速限幅 ---------------------------------------------------
dOver = aero_motor_dynamics(0, 5000, mParams);
dAtMax = aero_motor_dynamics(0, 1000, mParams);
check_that(dOver > 0 && abs(dOver - dAtMax) < 1e-12, ...
    '超范围目标未按 [0,1000] 限幅（%.6f vs %.6f）', dOver, dAtMax);
dAtTop = aero_motor_dynamics(1000, 5000, mParams);
check_that(dAtTop == 0, '电机到达转速上限后导数未置零');

% ---- 通用动力学不依赖品牌：任意参数结构均可运行 ------------------
custom = struct('tau_s', 0.2, 'rate_limit_rad_s', 2, 'range_rad', [-0.3, 0.3]);
check_that(isfinite(aero_servo_dynamics(0, 1, custom)), '通用舵机动力学拒绝自定义参数');

info = struct('passed', true, 'summary', sprintf( ...
    'KST/MAD 参数加载、一阶解析误差 %.1e/%.1e、速率/行程/转速限幅均通过', err, errW));

end
