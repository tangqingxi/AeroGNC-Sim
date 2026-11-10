function bus = rate_pid(t, bus, params)
%RATE_PID 角速度 PID 控制器（独立一级，默认 400 Hz）
%
% 控制律：
%   τ = Kp·e + I + Kff·ω_ref − Kd·α_est
%   e = ω_ref − ω
%
% 输入：
%   control/rate_setpoint        omega、alpha（前馈，仅记录）
%   navigation/state             omega、alpha（IdealState 差分估计）
%   control/allocation_status    saturated（+1/-1，Topic 反馈）
% 输出：
%   control/torque_setpoint      torque（N·m，FRD）
%
% 积分状态归本控制器所有；积分限幅，并按分配器上一拍的饱和方向冻结。
% 抗饱和只通过 Topic 读取，不访问分配器内部状态或其它控制器状态。

if ~isfield(bus, 'control_state')
    bus.control_state = struct();
end
if ~isfield(bus.control_state, 'rate')
    bus.control_state.rate = struct( ...
        'integral', zeros(3, 1), 'time', t, 'initialized', false);
end
state = bus.control_state.rate;

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
    + params.rate.Kff .* ref.data.omega(:) ...
    - params.rate.Kd .* nav.data.alpha(:);
torque = min(max(torque, -params.rate.torque_limit), params.rate.torque_limit);

% 饱和反馈必须来自 Topic；缺失时按未饱和处理
saturated = zeros(3, 1);
[status, statusOk] = bus.topics.tryRead(params.allocation_topic, t);
if strcmp(statusOk, 'ok')
    saturated = status.data.saturated(:);
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

bus.topics.publish(params.output_topic, struct('torque', torque), t, t, t, 'rate_pid');

state.time = t;
state.initialized = true;
bus.control_state.rate = state;

end
