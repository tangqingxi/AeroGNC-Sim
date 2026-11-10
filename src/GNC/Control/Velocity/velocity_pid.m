function bus = velocity_pid(t, bus, params)
%VELOCITY_PID 速度 PID 控制器（独立一级，默认 100 Hz）
%
% 控制律（PX4 纯 PID 路径，SI 版本）：
%   a_cmd = a_ff + Kp_vel·e_v + I_vel − Kd_vel·a_est
%   F_d = m·(a_cmd − g_ned) → 倾角/推力限幅 → q_d、T
%
% 输入：
%   control/velocity_setpoint  velocity、acceleration_ff、yaw、yaw_rate
%   navigation/state           velocity、acceleration（IdealState 差分估计）
% 输出：
%   control/attitude_setpoint  quaternion、thrust_n、yaw、yaw_rate
%
% PX4 对齐项：
%   - 垂直积分在 PID 前钳位到 ±g；
%   - 垂直积分饱和：推力到下限且仍要求上升、或到上限且仍要求下降时，
%     该拍垂直误差置零；
%   - 水平 Tracking Anti-Windup：推力矢量饱和时，用已产生的水平加速度
%     修正速度误差，增益 2/Kp_x；
%   - 推力上限、下限、水平推力余量与最大倾角共同约束控制需求；
%   - 积分更新为一阶欧拉，误差与输出均为 SI（m/s²）。
%
% Kd 使用**测量加速度反馈**（navigation/state.acceleration），单位 1（无量纲）；
% 因为加速度误差与加速度输出均为 m/s²，Kd 不是秒。不使用 Aircraft 真实加速度。
% 本模块不读取分配器/真值；积分状态归本控制器所有。

if ~isfield(bus, 'control_state')
    bus.control_state = struct();
end
if ~isfield(bus.control_state, 'velocity')
    bus.control_state.velocity = struct( ...
        'integral', zeros(3, 1), 'time', t, 'initialized', false);
end
state = bus.control_state.velocity;

[setpoint, setpointStatus] = bus.topics.tryRead(params.setpoint_topic, t);
[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
if ~strcmp(setpointStatus, 'ok') || ~strcmp(navStatus, 'ok')
    return;
end

if state.initialized
    dt = max(t - state.time, 1e-6);
else
    dt = 1 / params.rates.velocity_hz;
end

gravity = params.gravity_m_s2;
% PX4：垂直积分在速度控制前限制到 ±g
state.integral(3) = min(max(state.integral(3), -gravity), gravity);

velocityError = setpoint.data.velocity(:) - nav.data.velocity(:);
acceleration = setpoint.data.acceleration_ff(:) ...
    + params.velocity.Kp .* velocityError ...
    + state.integral ...
    - params.velocity.Kd .* nav.data.acceleration(:);

% 期望合力（NED）；推力沿 −z_b，因此 z_b = −F/|F|
forceDesired = params.mass_kg * (acceleration - [0; 0; gravity]);
forceNorm = norm(forceDesired);
if forceNorm < 1e-9
    forceDesired = [0; 0; -params.mass_kg * gravity];
    forceNorm = norm(forceDesired);
end
zBody = -forceDesired / forceNorm;
zBody = limit_tilt(zBody, params.max_tilt_rad);

% 垂直方向优先：保持期望垂直力，限制总推力与水平余量
verticalForce = forceDesired(3);
cosTilt = max(zBody(3), 1e-6);
verticalThrust = -verticalForce / cosTilt;
verticalThrust = max(verticalThrust, params.min_thrust_n);
verticalThrustMax = sqrt(max(params.max_thrust_n^2 ...
    - params.horizontal_thrust_margin_n^2, 0));
verticalThrust = min(verticalThrust, verticalThrustMax);
verticalForce = -cosTilt * verticalThrust;

% 水平方向：只有在垂直推力分配后剩余的推力能力才可用
horizontalForce = forceDesired(1:2);
horizontalMax = sqrt(max(params.max_thrust_n^2 - verticalForce^2, 0));
horizontalNorm = norm(horizontalForce);
if horizontalNorm > horizontalMax
    horizontalForce = horizontalForce / horizontalNorm * horizontalMax;
    horizontalNorm = horizontalMax;
end

forceLimited = [horizontalForce; verticalForce];
thrust = norm(forceLimited);
if thrust < 1e-9
    forceLimited = [0; 0; -params.mass_kg * gravity];
    thrust = norm(forceLimited);
end
zBody = -forceLimited / thrust;
zBody = limit_tilt(zBody, params.max_tilt_rad);

% 垂直积分抗饱和：与 PX4 _thr_sp(2) 的上下限判断等价（NED 向下为负）
if (verticalForce >= -params.min_thrust_n && velocityError(3) >= 0) || ...
        (verticalForce <= -params.max_thrust_n && velocityError(3) <= 0)
    velocityError(3) = 0;
end

% 水平 Tracking Anti-Windup：只在实际推力矢量受限时修正误差
producedAccelerationXY = forceLimited(1:2) / params.mass_kg;
accelerationXY = acceleration(1:2);
if norm(accelerationXY) > norm(producedAccelerationXY)
    arwGain = 2 / max(params.velocity.Kp(1), 1e-6);
    velocityError(1:2) = velocityError(1:2) ...
        - arwGain * (accelerationXY - producedAccelerationXY);
end

velocityError(~isfinite(velocityError)) = 0;
state.integral = state.integral + params.velocity.Ki .* velocityError * dt;
state.integral = min(max(state.integral, -params.velocity.integral_limit), ...
    params.velocity.integral_limit);
% 仍保证垂直积分不超过 ±g（PX4 每拍都会重新钳位）
state.integral(3) = min(max(state.integral(3), -gravity), gravity);

[qDesired, ~] = attitude_from_thrust_direction(zBody, setpoint.data.yaw);

payload = struct('quaternion', qDesired, 'thrust_n', thrust, ...
    'yaw', setpoint.data.yaw, 'yaw_rate', setpoint.data.yaw_rate);
bus.topics.publish(params.output_topic, payload, t, t, t, 'velocity_pid');

state.time = t;
state.initialized = true;
bus.control_state.velocity = state;

end

% ===========================================================================
function z = limit_tilt(z, maxTilt)
% 把机体 z 轴限制在相对 [0;0;1] 的最大倾角内（PX4 ControlMath::limitTilt）
vertical = [0; 0; 1];
cosTilt = min(1, max(-1, z.' * vertical));
angle = acos(cosTilt);
if angle > maxTilt
    axis = cross(z, vertical);
    if norm(axis) < 1e-9
        axis = [1; 0; 0];
    else
        axis = axis / norm(axis);
    end
    angle = angle - maxTilt;
    z = z * cos(angle) + cross(axis, z) * sin(angle) ...
        + axis * (axis.' * z) * (1 - cos(angle));
end
z = z / norm(z);
end
