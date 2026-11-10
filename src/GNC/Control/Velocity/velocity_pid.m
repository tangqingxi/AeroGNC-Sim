function bus = velocity_pid(t, bus, params)
%VELOCITY_PID 速度 PID 控制器（独立一级，默认 100 Hz）
%
% 控制律（PX4 SHC09 默认纯 PID 路径，SI 版本）：
%   a_cmd = a_ff + Kp_vel·e_v + I_vel − Kd_vel·a_est
%   1) 由水平加速度与重力构造初始 body-z（默认 decouple=1，不含 a_z）
%   2) 最大倾角限制
%   3) 由垂直加速度与 hover thrust 计算垂直推力
%   4) 最小/最大推力、水平余量与水平优先级
%   5) 最终受限推力 → 姿态、总推力
%   6) 用最终受限推力做 Tracking Anti-Windup，再更新积分
%
% 输入：
%   control/velocity_setpoint  velocity、acceleration_ff、yaw、yaw_rate
%   navigation/state           velocity、acceleration（IdealState 差分估计）
% 输出：
%   control/attitude_setpoint  quaternion、thrust_n、yaw、yaw_rate
%
% Kd 使用测量加速度反馈，单位 1（无量纲，输出/输入均为 m/s²）。
% 不读取 Aircraft 真值；积分状态归本控制器所有。

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

% 1) 默认 decouple 路径：body-z 只由水平加速度与重力决定。
%    NED z 向下；z_b = [-a_x, -a_y, g] / ||...||。
horizontal = [acceleration(1); acceleration(2); 0];
zBody = [-horizontal(1); -horizontal(2); gravity];
if norm(zBody) < 1e-9
    zBody = [0; 0; 1];
else
    zBody = zBody / norm(zBody);
end
zBody = limit_tilt(zBody, params.max_tilt_rad);
cosTilt = max(zBody(3), 1e-6);

% 2) PX4 hover-thrust 映射（归一化形式），再回到 SI 总推力：
%    T_z,norm = a_z*(hover/g) − hover
%    collective_norm = T_z,norm / cos
%    T = −collective_norm * T_max = hover_phys*(1 − a_z/g)/cos
hoverThrust = params.hover_thrust_n;
verticalThrust = hoverThrust * (1 - acceleration(3) / gravity) / cosTilt;
verticalThrust = max(verticalThrust, params.min_thrust_n);

% 3) 初始推力矢量（沿 −z_b）；先做垂直积分抗饱和。
thrustVector = -zBody * verticalThrust;
if (thrustVector(3) >= -params.min_thrust_n && velocityError(3) >= 0) || ...
        (thrustVector(3) <= -params.max_thrust_n && velocityError(3) <= 0)
    velocityError(3) = 0;
end

% 4) 水平余量：只预留当前水平需求，最多为 margin；垂直优先。
horizontalThrust = norm(thrustVector(1:2));
reserve = min(horizontalThrust, params.horizontal_thrust_margin_n);
verticalMax = sqrt(max(params.max_thrust_n^2 - reserve^2, 0));
thrustVector(3) = max(thrustVector(3), -verticalMax);

% 5) 用剩余能力限制水平推力。
horizontalMax = sqrt(max(params.max_thrust_n^2 - thrustVector(3)^2, 0));
horizontalThrust = norm(thrustVector(1:2));
if horizontalThrust > horizontalMax
    thrustVector(1:2) = thrustVector(1:2) / horizontalThrust * horizontalMax;
end

% 6) 最终受限推力、姿态与总推力必须由同一向量导出。
thrust = norm(thrustVector);
if thrust < 1e-9
    thrustVector = [0; 0; -params.max_thrust_n];
    thrust = norm(thrustVector);
end
zBody = -thrustVector / thrust;
zLimited = limit_tilt(zBody, params.max_tilt_rad);
if norm(zLimited - zBody) > 1e-12
    % 水平/垂直优先级处理后仍保证最终姿态在倾角锥内；同步重算力矢量，
    % 避免“姿态已限、推力仍对应旧方向”。
    zBody = zLimited;
    thrustVector = -zBody * thrust;
end

% 7) 水平 Tracking Anti-Windup：使用最终受限推力产生的水平加速度。
producedAccelerationXY = thrustVector(1:2) / params.mass_kg;
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
