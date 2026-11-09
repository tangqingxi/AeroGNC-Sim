function dx = shc09_dynamics(~, x, command, env, p, aero)
%SHC09_DYNAMICS SHC09 完整非线性六自由度动力学
%
% 状态（20 维，索引由 shc09_state_layout 给出）：
%   p_n(3) 位置 NED [m] | v_n(3) 速度 NED [m/s] | q_bn(4) 四元数
%   ω_b(3) 机体角速度 FRD [rad/s] | δ(6) 舵面实际偏角 [rad] | Ω 旋翼转速 [rad/s]
%
% 输入：
%   x       - 20×1 连续状态
%   command - struct('servos', 6×1 rad, 'motor', rad/s)
%   env     - 可选环境结构：wind_ned_m_s、force_ned_n、torque_body_nm
%   p       - SHC09 参数；aero - 气动样条
%
% 方程：
%   ṗ_n = v_n
%   v̇_n = R_nb F_b / m + F_ext,n / m + [0;0;g]
%   q̇   = 0.5 q ⊗ [0; ω_b]
%   ω̇_b = J⁻¹ (M_b − ω_b × J ω_b)
%   δ̇   = aero_servo_dynamics(δ, servos, 设备参数)
%   Ω̇   = aero_motor_dynamics(Ω, motor, 设备参数)
%
% 本函数不修改输入状态、不消耗随机数、不读写文件；同一输入必得同一导数。

s = shc09_state_layout();
[servoCommand, motorCommand] = shc09_unpack_command(command);

R_nb = shc09_quat_to_dcm(x(s.quaternion));
v_n = x(s.velocity);
omega = x(s.omega);
delta = x(s.servo);
Omega = x(s.motor);

[wind, forceExt, torqueExt] = environment_terms(env);
v_air_b = R_nb' * (v_n - wind);

[F_b, M_b] = shc09_aero_force(v_air_b, omega, delta, Omega, p, aero);

J = p.inertia_frd_kg_m2;
a_n = R_nb * F_b / p.mass_kg + forceExt / p.mass_kg + [0; 0; p.gravity_m_s2];
omegaDot = J \ (M_b + torqueExt - cross(omega, J * omega));

dx = zeros(s.nx, 1);
dx(s.position) = v_n;
dx(s.velocity) = a_n;
dx(s.quaternion) = shc09_quat_kinematics(x(s.quaternion), omega);
dx(s.omega) = omegaDot;
dx(s.servo) = aero_servo_dynamics(delta, servoCommand, p.servo_device);
dx(s.motor) = aero_motor_dynamics(Omega, motorCommand, p.motor_device);

end

% ===========================================================================
function [wind, forceExt, torqueExt] = environment_terms(env)
wind = zeros(3, 1);
forceExt = zeros(3, 1);
torqueExt = zeros(3, 1);
if isempty(env) || ~isstruct(env)
    return;
end
if isfield(env, 'wind_ned_m_s') && ~isempty(env.wind_ned_m_s)
    wind = env.wind_ned_m_s(:);
end
if isfield(env, 'force_ned_n') && ~isempty(env.force_ned_n)
    forceExt = env.force_ned_n(:);
end
if isfield(env, 'torque_body_nm') && ~isempty(env.torque_body_nm)
    torqueExt = env.torque_body_nm(:);
end
end
