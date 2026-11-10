function bus = velocity_pid(t, bus, params)
%VELOCITY_PID 速度 PID 控制器（独立一级，默认 100 Hz）
%
% 控制律：
%   a_cmd = a_ff + Kp_vel·e_v + I_vel − Kd_vel·a_est
%   F = m(a_cmd − g_ned)，T=|F|，并转换为期望姿态与总推力
%
% 输入：
%   control/velocity_setpoint  velocity、acceleration_ff
%   navigation/state           velocity、acceleration（IdealState 差分估计）
% 输出：
%   control/attitude_setpoint  quaternion、thrust_n
%
% Kd 使用**测量加速度反馈**（navigation/state.acceleration），单位 s；
% 不使用 Aircraft 真实加速度。积分状态归本控制器所有，限幅并按加速度
% 输出饱和方向冻结。

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

velocityError = setpoint.data.velocity(:) - nav.data.velocity(:);
acceleration = setpoint.data.acceleration_ff(:) ...
    + params.velocity.Kp .* velocityError ...
    + state.integral ...
    - params.velocity.Kd .* nav.data.acceleration(:);

accelerationRaw = acceleration;
acceleration = min(max(acceleration, -params.acceleration_limit), ...
    params.acceleration_limit);

% 抗饱和：输出已饱和且误差继续增大时，冻结该轴的积分
for axis = 1:3
    saturated = abs(accelerationRaw(axis)) >= params.acceleration_limit(axis) - 1e-12;
    if ~(saturated && sign(velocityError(axis)) == sign(accelerationRaw(axis)))
        state.integral(axis) = state.integral(axis) + ...
            params.velocity.Ki(axis) * velocityError(axis) * dt;
    end
end
state.integral = min(max(state.integral, -params.velocity.integral_limit), ...
    params.velocity.integral_limit);

[qDesired, thrust] = thrust_to_attitude(acceleration, params.yaw, ...
    params.mass_kg, params.gravity_m_s2, params.max_tilt_rad);

payload = struct('quaternion', qDesired, 'thrust_n', thrust);
bus.topics.publish(params.output_topic, payload, t, t, t, 'velocity_pid');

state.time = t;
state.initialized = true;
bus.control_state.velocity = state;

end
