function p = shc09_parameters(overrides)
%SHC09_PARAMETERS SHC09 装配与物理参数（唯一权威来源）
%
% 输入：
%   overrides - 可选结构体，显式覆盖下列字段；覆盖值记录在 p.overrides，
%               便于实验追溯（不得在脚本里散落物理常数）。
%
% 输出 p 的关键字段：
%   质量/惯量        mass_kg, inertia_frd_kg_m2
%   环境             gravity_m_s2, air_density_kg_m3
%   涵道             disc_area_m2, diffuser_ratio, duct.k_*
%   舵面几何         control_surface_angle_rad, l1_m, l2_m, servo_torque_coefficient
%   执行器           servo_device, motor_device, servo_deflection_limit_rad
%   装配             servo_pwm_rate_hz, servo_latch_rate_hz, motor_latch_rate_hz
%   气动数据来源     aero_source（external / fixture）
%
% 可信等级：惯性矩为 Gazebo/PX4 FRD 参考值，未实机辨识；舵机与电机动态
% 参数为名义仿真参数；几何与行程为已确认的实机定义。详见 DESIGN.md。

if nargin < 1
    overrides = struct();
end

p = struct();
p.name = 'SHC09';
p.version = '0.1.0';
p.frame = 'FRD';

% ---- 质量、惯量、环境 ------------------------------------------------
p.mass_kg = 2.1;
p.inertia_frd_kg_m2 = diag([0.0438, 0.0436, 0.005006]);
p.inertia_provenance = 'Gazebo/PX4 FRD 参考惯量；未实机辨识';
p.gravity_m_s2 = 9.788;
p.air_density_kg_m3 = 1.225;

% ---- 涵道与旋翼 ------------------------------------------------------
p.rotor_radius_m = 0.114;
p.disc_area_m2 = pi * 0.114^2;
p.diffuser_ratio = 0.7;
p.rotor_direction = 1;           % +1：绕 +Z 右手正向；推力沿 -Z
p.fan_inertia_kg_m2 = 3.7e-5;    % 旋翼转动惯量（陀螺力矩用）

p.duct = struct( ...
    'k_T0', 2.8964e-5, ...
    'k_Th', -3.7614e-4, ...
    'k_Ts', -2.6733e-4, ...
    'k_Ns', 5.0699e-4, ...
    'k_Q0', 5.7792e-7, ...
    'l_cpz', -0.15, ...
    'k_cpx', 2.6619);

% ---- 六舵面几何与效能 ------------------------------------------------
p.control_surface_angle_rad = deg2rad(0:60:300);
p.l1_m = 0.267;                  % 轴向有效力臂
p.l2_m = 0.066;                  % 偏航有效力臂
p.servo_torque_coefficient = 0.0032;   % d_cs
p.fixed_vane_coefficient = 5.3688e-4;  % k_sta
p.servo_deflection_limit_rad = deg2rad(40);

% ---- 非轴流阻尼 ------------------------------------------------------
p.damping_k_my0 = 0.0198;
p.damping_k_myv = 0.0015;

% ---- 执行器设备与装配频率 --------------------------------------------
p.servo_device = ServoDevices.KST_X08H.parameters();
p.motor_device = MotorDevices.MAD_M6C06_320KV_V2.parameters();
p.servo_pwm_rate_hz = 200;       % 实机舵机输出更新率（装配配置）
p.servo_latch_rate_hz = 200;     % 舵面指令锁存频率（装配配置）
p.motor_latch_rate_hz = 50;      % 电机指令锁存频率（可与舵面不同）

% ---- 气动数据来源 ----------------------------------------------------
p.aero_source = struct('type', 'external', 'file', '', 'tables', []);
p.aero_domain_policy = 'error';   % 'error'：越界报错；'clip'：显式钳位到边界

% ---- 显式覆盖 --------------------------------------------------------
if ~isempty(overrides)
    if ~isstruct(overrides) || ~isscalar(overrides)
        error('shc09:parameters:overrides', 'overrides 必须是标量结构体');
    end
    names = fieldnames(overrides);
    applied = struct();
    for k = 1:numel(names)
        if ~isfield(p, names{k})
            error('shc09:parameters:unknown', '未知的 SHC09 参数覆盖字段：%s', names{k});
        end
        p.(names{k}) = overrides.(names{k});
        applied.(names{k}) = overrides.(names{k});
    end
    p.overrides = applied;
else
    p.overrides = struct();
end

end
