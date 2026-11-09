function d = gnc_control_defaults()
%GNC_CONTROL_DEFAULTS P–PID–P–PID + PINV 基线的默认参数
%
% 所有增益均为 SI 单位下的显式数值，不直接照搬 PX4 归一化增益：
%   位置环      Kp [1/s]
%   速度环      Kp [1/s]、Ki [1/s²]、Kd [s]
%   姿态环      Kp [1/s]（误差为旋转向量，输出 rad/s）
%   角速度环    Kp [N·m/(rad/s)]、Ki [N·m/rad]、Kd [N·m/(rad/s²)]
%   PINV 分配   使用名义 B_tau（N·m/rad）与配平舵角（rad）
%
% 这些参数是仿真名义值，未经实机辨识；修改时必须同步更新实验记录。

d = struct();

% ---- 位置 / 速度 ------------------------------------------------------
d.position.Kp = [0.8; 0.8; 1.0];
d.velocity.Kp = [1.2; 1.2; 2.0];
d.velocity.Ki = [0.10; 0.10; 0.30];
d.velocity.Kd = [0.00; 0.00; 0.00];
d.velocity.integral_limit = [0.5; 0.5; 1.0];
d.velocity.limit_xy = 1.5;          % m/s
d.velocity.limit_up = 0.8;          % m/s（NED 负 z 向上）
d.velocity.limit_down = 0.8;        % m/s
d.acceleration_limit = [3.0; 3.0; 3.0];
d.max_tilt_rad = deg2rad(25);
d.velocity_feedforward = true;

% ---- 姿态 -------------------------------------------------------------
d.attitude.Kp = [4.0; 4.0; 2.0];
d.attitude.rate_limit = [2.0; 2.0; 1.0];
d.attitude.alpha_feedforward = [0; 0; 0];

% ---- 角速度 -----------------------------------------------------------
d.rate.Kp = [0.35; 0.35; 0.06];
d.rate.Ki = [0.08; 0.08; 0.02];
d.rate.Kd = [0.004; 0.004; 0.001];
d.rate.Kff = [0; 0; 0];
d.rate.integral_limit = [0.15; 0.15; 0.05];
d.rate.torque_limit = [0.5; 0.5; 0.15];
d.rate.saturation_tolerance = 1e-6;

% ---- 执行器 / 分配 ----------------------------------------------------
d.allocation.trim_delta_rad = [];    % 空表示由 SHC09 悬停配平填充
d.allocation.motor_min_rad_s = 200;
d.allocation.motor_max_rad_s = 1200;
d.allocation.ve_min_m_s = 1e-3;

% ---- 多速率 -----------------------------------------------------------
d.rates.sensor_hz = 400;
d.rates.navigation_hz = 400;
d.rates.guidance_hz = 50;
d.rates.position_hz = 100;
d.rates.attitude_hz = 200;
d.rates.rate_hz = 400;
d.rates.servo_latch_hz = 200;
d.rates.motor_latch_hz = 50;

end
