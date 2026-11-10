function bus = position_p(t, bus, params)
%POSITION_P 位置 P 控制器（独立一级，默认 100 Hz）
%
% 控制律：
%   v_ref = v_ff + Kp_pos .* (p_ref − p_est)
%   再按水平/垂直限幅得到速度参考
%
% 输入：
%   guidance/reference   p_ref、v_ff、a_ff、yaw、yaw_rate
%   navigation/state     position
% 输出：
%   control/velocity_setpoint  velocity（NED，m/s）、acceleration_ff（m/s²）、
%                              yaw（rad）、yaw_rate（rad/s）
%
% 本模块无积分状态，不执行速度 PID，不生成姿态，也不读取真值或速率环状态。
% acceleration_ff 只是把 Guidance 的前馈原样传递，不是本模块计算的控制量。
% 速度限幅属于 Position 级参数，不在 Velocity 控制器中重复定义。

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[ref, refStatus] = bus.topics.tryRead(params.reference_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(refStatus, 'ok')
    return;
end

positionError = ref.data.position(:) - nav.data.position(:);
positionVelocity = params.position.Kp .* positionError;
feedforward = ref.data.velocity(:);
total = feedforward + positionVelocity;

% 水平速度按 PX4 语义优先保证位置反馈分量，再用剩余能力容纳前馈。
velocity = zeros(3, 1);
velocity(1:2) = constrain_xy(positionVelocity(1:2), feedforward(1:2), ...
    params.position.velocity_limit_xy);
% 垂直方向分别限制上升/下降（NED，向上为负）
velocity(3) = min(max(total(3), -params.position.velocity_limit_up), ...
    params.position.velocity_limit_down);

payload = struct( ...
    'velocity', velocity, ...
    'acceleration_ff', ref.data.acceleration(:), ...
    'yaw', ref.data.yaw, ...
    'yaw_rate', ref.data.yaw_rate);

bus.topics.publish(params.output_topic, payload, t, t, t, 'position_p');

end
