function bus = position_p(t, bus, params)
%POSITION_P 位置 P 控制器（独立一级，默认 100 Hz）
%
% 控制律：
%   v_ref = v_ff + Kp_pos .* (p_ref − p_est)
%   再按水平/垂直限幅得到速度参考
%
% 输入：
%   guidance/reference   p_ref、v_ff、a_ff、yaw
%   navigation/state     position
% 输出：
%   control/velocity_setpoint  velocity（NED，m/s）与 acceleration_ff（m/s²）
%
% 本模块无积分状态，不执行速度 PID，不生成姿态，也不读取真值或速率环状态。
% acceleration_ff 只是把 Guidance 的前馈原样传递，不是本模块计算的控制量。

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[ref, refStatus] = bus.topics.tryRead(params.reference_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(refStatus, 'ok')
    return;
end

positionError = ref.data.position(:) - nav.data.position(:);
velocity = ref.data.velocity(:) + params.position.Kp .* positionError;

% 水平速度按优先级限幅；垂直方向分别限制上升/下降
xy = velocity(1:2);
xyNorm = norm(xy);
if xyNorm > params.velocity.limit_xy
    velocity(1:2) = xy / xyNorm * params.velocity.limit_xy;
end
velocity(3) = min(max(velocity(3), -params.velocity.limit_up), ...
    params.velocity.limit_down);

payload = struct( ...
    'velocity', velocity, ...
    'acceleration_ff', ref.data.acceleration(:));

bus.topics.publish(params.output_topic, payload, t, t, t, 'position_p');

end
