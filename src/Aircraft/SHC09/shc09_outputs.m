function truth = shc09_outputs(t, x, p, aero, env)
%SHC09_OUTPUTS SHC09 标准化真实状态输出（NED/FRD）
%
% 供传感器、日志与验证使用；Navigation/Control 不得读取 SHC09 内部下标。
%
% 输出字段（全部为显式单位）：
%   time_s, position_ned_m, velocity_ned_m_s, quaternion_bn, rotation_nb
%   omega_body_rad_s, airspeed_body_m_s, servo_rad, motor_rad_s
%   force_body_n, torque_body_nm, exit_speed_m_s
%
% 该函数为纯计算：不修改状态、不写文件。

if nargin < 5
    env = struct();
end
s = shc09_state_layout();

wind = zeros(3, 1);
torqueExt = zeros(3, 1);
if isstruct(env)
    if isfield(env, 'wind_ned_m_s') && ~isempty(env.wind_ned_m_s)
        wind = env.wind_ned_m_s(:);
    end
    if isfield(env, 'torque_body_nm') && ~isempty(env.torque_body_nm)
        torqueExt = env.torque_body_nm(:);
    end
end

q = x(s.quaternion);
R_nb = shc09_quat_to_dcm(q);
v_air = R_nb' * (x(s.velocity) - wind);

[F_b, M_b, Ve] = shc09_aero_force(v_air, x(s.omega), x(s.servo), x(s.motor), p, aero);

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
truth.force_body_n = F_b;
truth.torque_body_nm = M_b + torqueExt;
truth.exit_speed_m_s = Ve;

end
