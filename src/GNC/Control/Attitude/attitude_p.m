function bus = attitude_p(t, bus, params)
%ATTITUDE_P 姿态 P 控制器（独立一级，默认 200 Hz）
%
% 控制律（PX4 默认 reduced-attitude + yaw-weight 路径）：
%   e   = quat_error_default(q, q_ref, yaw_weight)
%   ω_ref = Kp_att·e + ω_yaw_rate + ω_ref_model_ff
%   最后按 rate_limit 逐轴限幅
%
% 输入：
%   navigation/state           quaternion、omega
%   control/attitude_setpoint  quaternion、thrust_n、yaw、yaw_rate
% 输出：
%   control/rate_setpoint      omega（rad/s）、alpha=0（rad/s²）、原样 thrust_n（N）
%
% 参考模型：PX4 默认启用二阶临界阻尼参考模型（MC_REF_W_N=50 rad/s，
% MC_REF_FF=0），本模块实现其 ZOH 精确离散形式。yaw_rate 是制导给出的
% 世界系偏航角速率，按 PX4 语义旋转到参考机体系后作为_rate feedforward，
% 不把世界系 yaw_rate 直接加到机体 z 轴。
%
% 不实现 PX4 的可选 DCM-log/tilt-torsion 等误差模式；V1.0 只保留默认路径。

if ~isfield(bus, 'control_state')
    bus.control_state = struct();
end
if ~isfield(bus.control_state, 'attitude')
    bus.control_state.attitude = struct( ...
        'q_ref', [1; 0; 0; 0], ...
        'omega_correction', zeros(3, 1), ...
        'omega_command', zeros(3, 1), ...
        'time', t, 'initialized', false);
end
state = bus.control_state.attitude;

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[setpoint, setpointStatus] = bus.topics.tryRead(params.attitude_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(setpointStatus, 'ok')
    return;
end

q = nav.data.quaternion(:) / norm(nav.data.quaternion);
qd = setpoint.data.quaternion(:) / norm(setpoint.data.quaternion);
yawRate = setpoint.data.yaw_rate;

if state.initialized
    dt = max(t - state.time, 1e-6);
else
    dt = 1 / params.rates.attitude_hz;
end

ref = params.attitude.ref_model;
if ref.enabled && ref.omega_n > 0
    if state.initialized && dt > 0
        qRefInv = quat_conj(state.q_ref);
        yawAxisBody = quat_rotate(qRefInv, [0; 0; 1]);
        omegaCommand = yawAxisBody * yawRate;

        qError = quat_canonical(quat_mul(qRefInv, qd));
        e = 2 * qError(2:4);
        wdt = ref.omega_n * dt;
        emt = exp(-wdt);
        a = (1 + wdt) * emt;
        b = dt * emt;
        gamma = ref.omega_n^2 * dt * emt;
        delta = (1 - wdt) * emt;

        deltaPhi = (1 - a) * e + b * state.omega_correction + omegaCommand * dt;
        state.omega_correction = gamma * e + delta * state.omega_correction;
        if abs(yawRate) > 1e-12
            state.omega_correction = state.omega_correction ...
                - (state.omega_correction.' * yawAxisBody) * yawAxisBody;
        end
        state.omega_command = omegaCommand;
        state.q_ref = quat_mul(state.q_ref, quat_from_rotvec(deltaPhi));
        state.q_ref = state.q_ref / norm(state.q_ref);
    else
        state.q_ref = qd;
        state.omega_correction = zeros(3, 1);
        state.omega_command = zeros(3, 1);
        state.initialized = true;
    end

    yawWeight = params.attitude.yaw_weight;
    errorVector = quat_error_default(q, state.q_ref, yawWeight);
    gain = params.attitude.Kp(:);
    gain(3) = gain(3) / max(yawWeight, 1e-4);
    omega = gain .* errorVector;

    qRel = quat_mul(quat_conj(q), state.q_ref);
    omega = omega + quat_rotate(qRel, state.omega_command);
    omegaFf = ref.ff_gain * quat_rotate(qRel, state.omega_correction);
    if ref.ff_limit > 0
        omegaFf = min(max(omegaFf, -ref.ff_limit), ref.ff_limit);
    end
    omega = omega + omegaFf;
else
    % 参考模型关闭时仍保留 PX4 的 yaw_rate 语义：世界系速率投影到机体
    state.q_ref = qd;
    state.initialized = true;
    yawWeight = params.attitude.yaw_weight;
    errorVector = quat_error_default(q, qd, yawWeight);
    gain = params.attitude.Kp(:);
    gain(3) = gain(3) / max(yawWeight, 1e-4);
    omega = gain .* errorVector;
    qRel = quat_mul(quat_conj(q), qd);
    yawAxisBody = quat_rotate(quat_conj(qd), [0; 0; 1]);
    omega = omega + quat_rotate(qRel, yawAxisBody * yawRate);
end

omega = min(max(omega, -params.attitude.rate_limit), params.attitude.rate_limit);

% 推力仅沿控制链传递，不参与姿态控制律，也不在此重新计算或限幅。
payload = struct('omega', omega, 'alpha', zeros(3, 1), ...
    'thrust_n', setpoint.data.thrust_n);
bus.topics.publish(params.output_topic, payload, t, t, t, 'attitude_p');

state.time = t;
bus.control_state.attitude = state;

end

% ===========================================================================
function q = quat_mul(a, b)
w = a(1) * b(1) - a(2:4).' * b(2:4);
v = a(1) * b(2:4) + b(1) * a(2:4) + cross(a(2:4), b(2:4));
q = [w; v];
end

function q = quat_conj(q)
q = [q(1); -q(2:4)];
end

function q = quat_canonical(q)
q = q / norm(q);
if q(1) < 0
    q = -q;
elseif abs(q(1)) < 1e-12
    idx = find(abs(q(2:4)) > 1e-12, 1, 'first');
    if ~isempty(idx) && q(idx + 1) < 0
        q = -q;
    end
end
end

function v = quat_rotate(q, v)
v = quat_to_rotation(q) * v;
end

function q = quat_from_rotvec(rotvec)
angle = norm(rotvec);
if angle < 1e-12
    q = [1; rotvec / 2];
else
    q = [cos(angle / 2); rotvec / angle * sin(angle / 2)];
end
q = q / norm(q);
end
