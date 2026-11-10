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
%
% PX4 RateControl 对齐项：
%   - PID 求和后**不做本地力矩限幅**；实际约束由 PINV/执行器限幅与
%     分配器饱和反馈承担。
%   - 条件积分先按饱和方向钳位误差符号，再乘 i_factor：
%       i_factor = max(0, 1 − (e_ω / (400°/s))²)
%   - 一阶欧拉积分，积分限幅。

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

% 饱和反馈必须来自 Topic；缺失时按未饱和处理
saturated = zeros(3, 1);
[status, statusOk] = bus.topics.tryRead(params.allocation_topic, t);
if strcmp(statusOk, 'ok')
    saturated = status.data.saturated(:);
end

iFactorAngle = deg2rad(400);
for axis = 1:3
    errorAxis = rateError(axis);
    % 条件积分：正饱和时不再允许正误差，负饱和时不再允许负误差
    if saturated(axis) > 0
        errorAxis = min(errorAxis, 0);
    elseif saturated(axis) < 0
        errorAxis = max(errorAxis, 0);
    end
    iFactor = max(0, 1 - (errorAxis / iFactorAngle)^2);
    state.integral(axis) = state.integral(axis) ...
        + params.rate.Ki(axis) * iFactor * errorAxis * dt;
end
state.integral = min(max(state.integral, -params.rate.integral_limit), ...
    params.rate.integral_limit);

bus.topics.publish(params.output_topic, struct('torque', torque), t, t, t, 'rate_pid');

state.time = t;
state.initialized = true;
bus.control_state.rate = state;

end
