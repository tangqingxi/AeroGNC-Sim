function bus = attitude_step(t, bus, params)
%ATTITUDE_STEP 分段常值姿态参考（Guidance 层，不做姿态跟踪）
%
% 输入 params：
%   output_topic       - guidance/attitude_reference
%   quaternion_level   - 水平/恢复姿态 q_bn（4×1，标量在前）
%   quaternion_step    - 阶跃期间目标姿态 q_bn（4×1）
%   thrust_n           - 固定总推力参考（N）
%   yaw / yaw_rate     - 参考航向（rad）与世界系偏航角速率（rad/s）
%   start_s / end_s    - 阶跃起止（s）；t>=end_s 恢复水平姿态
%
% 该模块只生成姿态/推力参考，不读取导航状态，也不计算角速度或执行器指令。

if t >= params.start_s && t < params.end_s
    quaternion = params.quaternion_step(:);
else
    quaternion = params.quaternion_level(:);
end
quaternion = quaternion / norm(quaternion);

payload = struct( ...
    'quaternion', quaternion, ...
    'thrust_n', params.thrust_n, ...
    'yaw', params.yaw, ...
    'yaw_rate', params.yaw_rate);

bus.topics.publish(params.output_topic, payload, t, t, t, 'guidance');

end
