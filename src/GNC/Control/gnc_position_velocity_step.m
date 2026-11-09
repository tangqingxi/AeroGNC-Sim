function bus = gnc_position_velocity_step(t, bus, params)
%GNC_POSITION_VELOCITY_STEP 位置 P + 速度 PID（100 Hz）
%
% 数据流：
%   navigation/state + guidance/reference
%     → 速度指令（位置 P + 速度前馈）
%     → 加速度指令（速度 PID + 加速度前馈）
%     → control/attitude_setpoint（姿态与总推力）
%
% 单位：位置 m、速度 m/s、加速度 m/s²、推力 N、偏航 rad。
% 积分状态保存在 bus.gnc.position 中，仅本组件读写。

if ~isfield(bus, 'gnc') || ~isfield(bus.gnc, 'position')
    if ~isfield(bus, 'gnc')
        bus.gnc = struct();
    end
    bus.gnc.position = struct('integral', zeros(3, 1), 'time', t, 'initialized', false);
end
state = bus.gnc.position;

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[ref, refStatus] = bus.topics.tryRead(params.reference_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(refStatus, 'ok')
    return;   % 数据不完整时保持上一输出，不用零值伪造
end

if state.initialized
    dt = max(t - state.time, 1e-6);
else
    dt = 1 / params.rates.position_hz;
end

% ---- 位置 P：速度指令 ------------------------------------------------
positionError = ref.data.position(:) - nav.data.position(:);
velocityCommand = ref.data.velocity(:) + params.position.Kp .* positionError;

% 水平速度按优先级限幅；垂直方向分别限制上升/下降
xy = velocityCommand(1:2);
xyNorm = norm(xy);
if xyNorm > params.velocity.limit_xy
    velocityCommand(1:2) = xy / xyNorm * params.velocity.limit_xy;
end
velocityCommand(3) = min(max(velocityCommand(3), -params.velocity.limit_up), ...
    params.velocity.limit_down);

% ---- 速度 PID --------------------------------------------------------
velocityError = velocityCommand - nav.data.velocity(:);
acceleration = params.velocity.Kp .* velocityError + state.integral;
if params.velocity_feedforward
    acceleration = acceleration + ref.data.acceleration(:);
end

% 积分更新与限幅（简化的条件抗饱和：输出已饱和且误差继续增大时冻结）
accelerationRaw = acceleration;
acceleration = min(max(acceleration, -params.acceleration_limit), ...
    params.acceleration_limit);
for axis = 1:3
    saturated = abs(accelerationRaw(axis)) >= params.acceleration_limit(axis) - 1e-12;
    if ~(saturated && sign(velocityError(axis)) == sign(accelerationRaw(axis)))
        state.integral(axis) = state.integral(axis) + ...
            params.velocity.Ki(axis) * velocityError(axis) * dt;
    end
end
state.integral = min(max(state.integral, -params.velocity.integral_limit), ...
    params.velocity.integral_limit);

% ---- 推力矢量与姿态 --------------------------------------------------
[qDesired, thrust, zBody] = gnc_thrust_to_attitude(acceleration, ref.data.yaw, ...
    params.mass_kg, params.gravity_m_s2, params.max_tilt_rad);

payload = struct('quaternion', qDesired, 'thrust_n', thrust);
bus.topics.publish(params.attitude_topic, payload, t, t, t, 'position_control');

state.time = t;
state.initialized = true;
bus.gnc.position = state;
bus.gnc.position.last_z_body = zBody; %#ok<NASGU>  % 供诊断日志读取

end
