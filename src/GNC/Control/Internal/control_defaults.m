function d = control_defaults()
%CONTROL_DEFAULTS 四级控制器与 PINV 分配默认参数（SI 单位）
%
% 字段按控制器分组，模块只读取自己的参数结构：
%   position.Kp [1/s]、velocity_limit_* [m/s]；
%   velocity.Kp [1/s]、Ki [1/s²]、Kd [1]（测量加速度反馈，无量纲）；
%   attitude.Kp [1/s]、yaw_weight [1]、ref_model（PX4 默认参考模型）；
%   rate.Kp [N·m/(rad/s)]、Ki [N·m/rad]、Kd [N·m/(rad/s²)]、Kff [N·m/(rad/s)]；
%   allocation：配平、行程、电机与名义模型。
%
% 增益为仿真名义值，未经实机辨识；修改必须同步实验记录与 WTR 参数。

d = struct();

% ---- 位置 P ----------------------------------------------------------
d.position.Kp = [0.8; 0.8; 1.0];
d.position.velocity_limit_xy = 1.5;   % m/s，位置环水平速度上限
d.position.velocity_limit_up = 0.8;   % m/s（NED 负 z 向上）
d.position.velocity_limit_down = 0.8; % m/s

% ---- 速度 PID --------------------------------------------------------
d.velocity.Kp = [1.2; 1.2; 2.0];
d.velocity.Ki = [0.10; 0.10; 0.30];
d.velocity.Kd = [0.00; 0.00; 0.00];   % 测量加速度反馈；输出/输入均 m/s²，故无量纲
d.velocity.integral_limit = [0.5; 0.5; 1.0];
d.max_tilt_rad = deg2rad(25);

% ---- 姿态 P ----------------------------------------------------------
d.attitude.Kp = [4.0; 4.0; 2.0];
d.attitude.rate_limit = [2.0; 2.0; 1.0];
d.attitude.yaw_weight = 0.4;          % PX4 MC_YAW_WEIGHT 默认
d.attitude.ref_model = struct( ...
    'enabled', true, ...
    'omega_n', 50.0, ...              % rad/s，PX4 MC_REF_W_N 默认
    'ff_gain', 0.0, ...               % PX4 MC_REF_FF 默认
    'ff_limit', deg2rad(100));        % rad/s，PX4 MC_REF_FF_MAX 默认

% ---- 角速度 PID ------------------------------------------------------
d.rate.Kp = [0.35; 0.35; 0.06];
d.rate.Ki = [0.08; 0.08; 0.02];
d.rate.Kd = [0.004; 0.004; 0.001];
d.rate.Kff = [0; 0; 0];
d.rate.integral_limit = [0.15; 0.15; 0.05];
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
