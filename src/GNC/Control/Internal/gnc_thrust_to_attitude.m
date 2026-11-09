function [q_desired, thrust_n, z_body] = gnc_thrust_to_attitude( ...
    acceleration_ned, yaw_rad, mass_kg, gravity_m_s2, max_tilt_rad)
%GNC_THRUST_TO_ATTITUDE 由期望加速度构造推力矢量、姿态与总推力
%
% 输入（NED / FRD，SI）：
%   acceleration_ned - 期望平动加速度（m/s²，含重力补偿语义）
%   yaw_rad          - 期望偏航角（rad，NED 航向）
%   mass_kg          - 质量（kg）
%   gravity_m_s2     - 重力加速度（m/s²，NED 向下为正）
%   max_tilt_rad     - 相对悬停姿态的最大倾角（rad）
%
% 输出：
%   q_desired - 期望 q_bn（4×1，标量在前）
%   thrust_n  - 总推力需求（N，沿 -z_b）
%   z_body    - 期望机体 z 轴（FRD，3×1）
%
% 推导：
%   m·a = F_thrust + m·g_ned  →  F_thrust = m·(a − g_ned)
%   推力沿 −z_b，因此 z_b = −F/|F|；悬停时 F=[0;0;−mg]，z_b=[0;0;1]。
%   偏航由投影到垂直于 z_b 的航向向量确定，再正交化得到完整旋转矩阵。

if nargin < 5
    max_tilt_rad = deg2rad(30);
end
acceleration_ned = acceleration_ned(:);
if numel(acceleration_ned) ~= 3 || any(~isfinite(acceleration_ned))
    error('gnc:control:acceleration', '期望加速度必须是 3 个有限实数');
end

gravity_ned = [0; 0; gravity_m_s2];
force_ned = mass_kg * (acceleration_ned - gravity_ned);
force_norm = norm(force_ned);
if force_norm < 1e-9
    force_ned = [0; 0; -mass_kg * gravity_m_s2];
    force_norm = norm(force_ned);
end
z_body = -force_ned / force_norm;

% 倾角限制：把 z 轴朝悬停轴旋转回允许锥体内
vertical = [0; 0; 1];
cos_tilt = min(1, max(-1, z_body.' * vertical));
tilt = acos(cos_tilt);
if tilt > max_tilt_rad
    axis = cross(z_body, vertical);
    axis_norm = norm(axis);
    if axis_norm < 1e-9
        axis = [1; 0; 0];   % 与悬停轴反向时的任意水平轴
    else
        axis = axis / axis_norm;
    end
    angle = tilt - max_tilt_rad;
    z_body = rotate_axis(z_body, axis, angle);
end
z_body = z_body / norm(z_body);

% 由偏航航向构造 x/y 轴：x 轴在水平航向在 z 垂面上的投影
heading = [cos(yaw_rad); sin(yaw_rad); 0];
y_body = cross(z_body, heading);
if norm(y_body) < 1e-9
    y_body = cross(z_body, [0; 1; 0]);
    if norm(y_body) < 1e-9
        y_body = cross(z_body, [1; 0; 0]);
    end
end
y_body = y_body / norm(y_body);
x_body = cross(y_body, z_body);
x_body = x_body / norm(x_body);
y_body = cross(z_body, x_body);

R_desired = [x_body, y_body, z_body];
q_desired = gnc_rotation_to_quat(R_desired);
thrust_n = force_norm;

end

% ===========================================================================
function v = rotate_axis(v, axis, angle)
% Rodrigues 旋转
axis = axis / norm(axis);
v = v * cos(angle) + cross(axis, v) * sin(angle) + axis * (axis.' * v) * (1 - cos(angle));
end
