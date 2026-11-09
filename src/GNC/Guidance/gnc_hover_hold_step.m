function bus = gnc_hover_hold_step(t, bus, params)
%GNC_HOVER_HOLD_STEP 悬停位置参考（Guidance，只给目标不做追踪）
%
% 输入 params：
%   output_topic - 参考 Topic 名称（默认 guidance/reference）
%   position     - 目标位置 NED（3×1，m）
%   velocity     - 前馈速度（3×1，m/s，默认零）
%   acceleration - 前馈加速度（3×1，m/s²，默认零）
%   yaw          - 目标偏航（rad，默认 0）
%   yaw_rate     - 偏航角速率（rad/s，默认 0）

payload = struct( ...
    'position', params.position(:), ...
    'velocity', params.velocity(:), ...
    'acceleration', params.acceleration(:), ...
    'yaw', params.yaw, ...
    'yaw_rate', params.yaw_rate);

bus.topics.publish(params.output_topic, payload, t, t, t, 'guidance');

end
