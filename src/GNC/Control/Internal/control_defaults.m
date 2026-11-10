function d = control_defaults()
%CONTROL_DEFAULTS 四级控制器与 PINV 分配默认参数（SI 单位）
%
% 字段按控制器分组，模块只读取自己的参数结构：
%   position.Kp [1/s]；velocity.Kp [1/s]、Ki [1/s²]、Kd [s]（测量加速度反馈）；
%   attitude.Kp [1/s]、omega_ff [rad/s]（角速度前馈，不是角加速度）；
%   rate.Kp [N·m/(rad/s)]、Ki [N·m/rad]、Kd [N·m/(rad/s²)]、Kff [N·m/(rad/s)]；
%   allocation：配平、行程、电机与名义模型。
%
% 增益为仿真名义值，未经实机辨识；修改必须同步实验记录与 WTR 参数。

d = struct();

% ---- 位置 P ----------------------------------------------------------
d.position.Kp = [0.8; 0.8; 1.0];

% ---- 速度 PID --------------------------------------------------------
d.velocity.Kp = [1.2; 1.2; 2.0];
d.velocity.Ki = [0.10; 0.10; 0.30];
d.velocity.Kd = [0.00; 0.00; 0.00];   % 测量加速度反馈；非零值有独立测试
d.velocity.integral_limit = [0.5; 0.5; 1.0];
d.velocity.limit_xy = 1.5;            % m/s
d.velocity.limit_up = 0.8;            % m/s（NED 负 z 向上）
d.velocity.limit_down = 0.8;          % m/s
d.acceleration_limit = [3.0; 3.0; 3.0];
d.max_tilt_rad = deg2rad(25);

% ---- 姿态 P ----------------------------------------------------------
d.attitude.Kp = [4.0; 4.0; 2.0];
d.attitude.rate_limit = [2.0; 2.0; 1.0];
d.attitude.omega_ff = zeros(3, 1);    % rad/s，可选角速度前馈

% ---- 角速度 PID ------------------------------------------------------
d.rate.Kp = [0.35; 0.35; 0.06];
d.rate.Ki = [0.08; 0.08; 0.02];
d.rate.Kd = [0.004; 0.004; 0.001];
d.rate.Kff = [0; 0; 0];
d.rate.integral_limit = [0.15; 0.15; 0.05];
d.rate.torque_limit = [0.5; 0.5; 0.15];
d.rate.saturation_tolerance = 1e-6;

% ---- 执行器 / 分配 ---------------------------------------------------
d.allocation.trim_delta_rad = [];     % 空表示由 SHC09 悬停配平填充
d.allocation.motor_min_rad_s = 200;
d.allocation.motor_max_rad_s = 1200;
d.allocation.ve_min_m_s = 1e-3;

% ---- 多速率（Position 与 Velocity 独立配置） -------------------------
d.rates.sensor_hz = 400;
d.rates.navigation_hz = 400;
d.rates.guidance_hz = 50;
d.rates.position_hz = 100;
d.rates.velocity_hz = 100;
d.rates.attitude_hz = 200;
d.rates.rate_hz = 400;
d.rates.servo_latch_hz = 200;
d.rates.motor_latch_hz = 50;

end
