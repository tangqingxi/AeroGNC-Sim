function bus = gnc_rate_step(t, bus, params)
%GNC_RATE_STEP 角速度 PID（400 Hz）
%
% 读取 navigation/state 与角速度参考（control/rate_setpoint 或
% guidance/rate_reference），计算机体力矩需求：
%   τ = Kp·e + I + Kff·ω_ref − Kd·α_meas
% 其中 e = ω_ref − ω。积分限幅，并按分配器上一拍的饱和方向冻结积分，
% 避免 PID 与执行器限幅互相拉扯。
%
% 输出 control/torque_setpoint（N·m），不直接写执行器。

if ~isfield(bus, 'gnc') || ~isfield(bus.gnc, 'rate')
    if ~isfield(bus, 'gnc')
        bus.gnc = struct();
    end
    bus.gnc.rate = struct('integral', zeros(3, 1), 'time', t, 'initialized', false);
end
state = bus.gnc.rate;

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[ref, refStatus] = bus.topics.tryRead(params.rate_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(refStatus, 'ok')
    return;
end

if state.initialized
    dt = max(t - state.time, 1e-6);
else
    dt = 1 / params.rates.rate_hz;
end

rateError = ref.data.omega(:) - nav.data.omega(:);
torque = params.rate.Kp .* rateError + state.integral ...
    + params.rate.Kff .* ref.data.omega(:) - params.rate.Kd .* nav.data.alpha(:);
torque = min(max(torque, -params.rate.torque_limit), params.rate.torque_limit);

% 抗饱和：上一拍分配器已饱和时，禁止继续同方向积分
saturated = zeros(3, 1);
if isfield(bus.gnc, 'allocation') && isfield(bus.gnc.allocation, 'saturated')
    saturated = bus.gnc.allocation.saturated;
end
for axis = 1:3
    blocked = (saturated(axis) > 0 && rateError(axis) > 0) || ...
              (saturated(axis) < 0 && rateError(axis) < 0);
    if ~blocked
        state.integral(axis) = state.integral(axis) + ...
            params.rate.Ki(axis) * rateError(axis) * dt;
    end
end
state.integral = min(max(state.integral, -params.rate.integral_limit), ...
    params.rate.integral_limit);

bus.topics.publish(params.torque_topic, struct('torque', torque), t, t, t, 'rate_control');

state.time = t;
state.initialized = true;
bus.gnc.rate = state;

end
