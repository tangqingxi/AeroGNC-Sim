function truth = shc09_outputs(t, x, p, aero, env)
%SHC09_OUTPUTS SHC09 标准化真实状态输出（NED/FRD）
%
% 供传感器、日志与验证使用；Navigation/Control 不得读取 SHC09 内部下标。
%
% 力与加速度语义（全部在机体系 FRD）：
%   aero_force_body_n        纯气动力（涵道+机翼+舵面）
%   external_force_body_n    外部扰动力的机体系表示（env.force_ned_n 旋转后）
%   total_force_body_n       气动 + 外部扰动（不含重力）
%   specific_force_body_m_s2 total_force / m，即加速度计比力（不含重力）
%   gravity_ned_m_s2         重力在 NED 中的分量 [0;0;g]
%   acceleration_ned_m_s2    R_nb·total_force/m + 重力
%   aero_torque_body_nm      纯气动力矩（不含外部扰动力矩）
%   external_torque_body_nm  外部扰动力矩
%   total_torque_body_nm     气动 + 外部扰动
%
% 该函数为纯计算：不修改状态、不写文件。

if nargin < 5
    env = struct();
end
s = shc09_state_layout();

wind = zeros(3, 1);
externalForceNed = zeros(3, 1);
externalTorqueBody = zeros(3, 1);
if isstruct(env)
    if isfield(env, 'wind_ned_m_s') && ~isempty(env.wind_ned_m_s)
        wind = env.wind_ned_m_s(:);
    end
    if isfield(env, 'force_ned_n') && ~isempty(env.force_ned_n)
        externalForceNed = env.force_ned_n(:);
    end
    if isfield(env, 'torque_body_nm') && ~isempty(env.torque_body_nm)
        externalTorqueBody = env.torque_body_nm(:);
    end
end

q = x(s.quaternion);
R_nb = shc09_quat_to_dcm(q);
v_air = R_nb' * (x(s.velocity) - wind);

[aeroForce, aeroTorque, Ve] = shc09_forces_and_moments( ...
    v_air, x(s.omega), x(s.servo), x(s.motor), p, aero);

externalForceBody = R_nb' * externalForceNed;
totalForceBody = aeroForce + externalForceBody;
totalTorqueBody = aeroTorque + externalTorqueBody;

truth = struct();
truth.time_s = t;
truth.position_ned_m = x(s.position);
truth.velocity_ned_m_s = x(s.velocity);
truth.quaternion_bn = q;
truth.rotation_nb = R_nb;
truth.omega_body_rad_s = x(s.omega);
truth.airspeed_body_m_s = v_air;
truth.servo_rad = x(s.servo);
truth.motor_rad_s = x(s.motor);
truth.exit_speed_m_s = Ve;

truth.aero_force_body_n = aeroForce;
truth.external_force_ned_n = externalForceNed;
truth.external_force_body_n = externalForceBody;
truth.total_force_body_n = totalForceBody;
truth.specific_force_body_m_s2 = totalForceBody / p.mass_kg;
truth.gravity_ned_m_s2 = [0; 0; p.gravity_m_s2];
truth.acceleration_ned_m_s2 = R_nb * totalForceBody / p.mass_kg + truth.gravity_ned_m_s2;

truth.aero_torque_body_nm = aeroTorque;
truth.external_torque_body_nm = externalTorqueBody;
truth.total_torque_body_nm = totalTorqueBody;

end
