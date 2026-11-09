function bus = gnc_attitude_step(t, bus, params)
%GNC_ATTITUDE_STEP 姿态 P（200 Hz）
%
% 读取 navigation/state 与 control/attitude_setpoint，按四元数误差
% e = 2·vec(q⁻¹⊗q_d) 计算角速度指令，输出到 control/rate_setpoint。
% 不读取真实姿态或真实角速度，只使用导航 Topic。

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[setpoint, setpointStatus] = bus.topics.tryRead(params.attitude_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(setpointStatus, 'ok')
    return;
end

errorVector = gnc_quat_error(nav.data.quaternion, setpoint.data.quaternion);
omega = params.attitude.Kp .* errorVector + params.attitude.alpha_feedforward;
omega = min(max(omega, -params.attitude.rate_limit), params.attitude.rate_limit);

payload = struct('omega', omega, 'alpha', zeros(3, 1));
bus.topics.publish(params.rate_topic, payload, t, t, t, 'attitude_control');

end
