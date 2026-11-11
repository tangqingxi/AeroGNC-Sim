function bus = velocity_step(t, bus, params)
%VELOCITY_STEP 分段常值速度参考（Guidance 层，不做速度跟踪）
%
% 输入 params：
%   output_topic     - 速度参考 Topic（guidance/velocity_reference）
%   velocity          - 阶跃期间的速度参考（NED，3×1，m/s）
%   acceleration_ff   - 前馈加速度（NED，3×1，m/s²）
%   yaw / yaw_rate    - 航向参考（rad / rad/s）
%   start_s / end_s   - 阶跃起止时刻（s）；t>=end_s 后恢复零速度
%
% 输出载荷：velocity、acceleration_ff、yaw、yaw_rate。该模块只生成参考，
% 不读取导航状态，也不计算姿态、推力或舵面。

if t < params.start_s || t >= params.end_s
    velocity = zeros(3, 1);
else
    velocity = params.velocity(:);
end

payload = struct( ...
    'velocity', velocity, ...
    'acceleration_ff', params.acceleration_ff(:), ...
    'yaw', params.yaw, ...
    'yaw_rate', params.yaw_rate);

bus.topics.publish(params.output_topic, payload, t, t, t, 'guidance');

end
