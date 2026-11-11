function bus = body_rate_step(t, bus, params)
%BODY_RATE_STEP 角速度阶跃参考（Guidance）
%
% 输入 params：
%   output_topic - 角速度参考 Topic（guidance/rate_reference）
%   axis         - 阶跃轴（1=滚转，2=俯仰，3=偏航）
%   amplitude    - 阶跃幅值（rad/s）
%   start_s      - 开始时刻（s）
%   end_s        - 结束时刻（s，之后回到零）
%   thrust_n     - 场景固定总推力（N，FRD -Z）
%   alpha_ff     - 角加速度前馈（rad/s²，默认零）
%
% 参考是分段常值，不在 Guidance 内做追踪或滤波。

omega = zeros(3, 1);
if t >= params.start_s && t < params.end_s
    omega(params.axis) = params.amplitude;
end

payload = struct('omega', omega, 'alpha', params.alpha_ff(:), ...
    'thrust_n', params.thrust_n);
bus.topics.publish(params.output_topic, payload, t, t, t, 'guidance');

end
