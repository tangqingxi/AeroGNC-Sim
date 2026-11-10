function bus = ideal_state(t, bus, params)
%IDEAL_STATE IdealState 测试导航适配器
%
% 输入：
%   t      - 当前仿真时间（s）
%   bus    - 内核总线（含 bus.topics）
%   params - sensor_topic、output_topic、acceleration_limit
%
% 输出 navigation/state：
%   position、velocity（原样）、acceleration（相邻速度差分）、quaternion、
%   omega、alpha（相邻角速度差分）、sample_time_s、ideal=1。
%
% 平动/角加速度都由**本适配器**根据测量时间差估计，并显式标记为理想
% 估计；控制器不读取 Aircraft 真实加速度。该适配器不是 EKF，也不是
% 真实传感器融合结果。

if ~isfield(bus, 'topics')
    error('gnc:nav:topics', 'bus.topics 缺失，GNC 必须通过 Topic Bus 通信');
end

[msg, status] = bus.topics.tryRead(params.sensor_topic, t);
if ~strcmp(status, 'ok')
    return;   % 尚无可用测量时保持上一估计，不发布伪造数据
end

value = msg.data.value(:);
if numel(value) ~= 13 || any(~isfinite(value))
    error('gnc:nav:sensor', 'sensor/state 载荷必须是 13 个有限实数');
end
position = value(1:3);
velocity = value(4:6);
quaternion = value(7:10);
omega = value(11:13);
quaternion = quaternion / norm(quaternion);
if quaternion(1) < 0
    quaternion = -quaternion;
end

acceleration = zeros(3, 1);
alpha = zeros(3, 1);
if isfield(bus, 'navigation_state')
    previous = bus.navigation_state;
    dt = msg.data.sample_time_s - previous.sample_time;
    if dt > 1e-9
        acceleration = (velocity - previous.velocity) / dt;
        alpha = (omega - previous.omega) / dt;
        limit = params.acceleration_limit;
        acceleration = min(max(acceleration, -limit), limit);
        alpha = min(max(alpha, -limit), limit);
    end
end

bus.navigation_state = struct( ...
    'sample_time', msg.data.sample_time_s, ...
    'velocity', velocity, ...
    'omega', omega);

payload = struct( ...
    'position', position, ...
    'velocity', velocity, ...
    'acceleration', acceleration, ...
    'quaternion', quaternion, ...
    'omega', omega, ...
    'alpha', alpha, ...
    'sample_time_s', msg.data.sample_time_s, ...
    'ideal', 1);

bus.topics.publish(params.output_topic, payload, msg.data.sample_time_s, t, t, 'navigation');

end
