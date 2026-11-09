function bus = gnc_ideal_state_step(t, bus, params)
%GNC_IDEAL_STATE_STEP IdealState 测试导航适配器
%
% 输入：
%   t      - 当前仿真时间（s）
%   bus    - 内核总线（含 bus.topics）
%   params - struct：sensor_topic、output_topic、alpha_limit
%
% 行为：
%   从 sensor/state 读取最新可用测量，按 PX4 风格输出 StateEstimate 到
%   navigation/state。位置/速度/四元数原样传递（这是 IdealState 假设，
%   不是 EKF）；角加速度由相邻测量差分得到并限幅。
%
% 真值隔离：本函数只读 Topic，不接触任何真实状态或 Aircraft 内部字段。

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

if ~isfield(bus, 'gnc')
    bus.gnc = struct();
end
% 角加速度：相邻测量差分；首个样本置零
alpha = zeros(3, 1);
if isfield(bus, 'gnc') && isfield(bus.gnc, 'navigation')
    previous = bus.gnc.navigation;
    dt = msg.data.sample_time_s - previous.sample_time;
    if dt > 1e-9
        alpha = (omega - previous.omega) / dt;
        limit = params.alpha_limit;
        alpha = min(max(alpha, -limit), limit);
    end
end

bus.gnc.navigation = struct('sample_time', msg.data.sample_time_s, 'omega', omega);

payload = struct( ...
    'position', position, ...
    'velocity', velocity, ...
    'quaternion', quaternion, ...
    'omega', omega, ...
    'alpha', alpha, ...
    'sample_time_s', msg.data.sample_time_s, ...
    'ideal', 1);

bus.topics.publish(params.output_topic, payload, msg.data.sample_time_s, t, t, 'navigation');

end
