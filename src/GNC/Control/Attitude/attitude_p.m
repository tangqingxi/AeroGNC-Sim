function bus = attitude_p(t, bus, params)
%ATTITUDE_P 姿态 P 控制器（独立一级，默认 200 Hz）
%
% 控制律：
%   e = 2·vec(q⁻¹ ⊗ q_d)（机体系旋转向量，标量在前 Hamilton）
%   omega_ref = Kp_att·e + omega_ff
%
% 输入：
%   navigation/state           quaternion
%   control/attitude_setpoint  quaternion
% 输出：
%   control/rate_setpoint      omega（rad/s）、alpha=0（rad/s²）
%
% omega_ff 的单位是 rad/s（角速度前馈）；不得把角加速度（rad/s²）加到
% 角速度上。若上游没有角加速度前馈，alpha 显式输出零，而不是复用其它量。

[nav, navStatus] = bus.topics.tryRead(params.nav_topic, t);
[setpoint, setpointStatus] = bus.topics.tryRead(params.attitude_topic, t);
if ~strcmp(navStatus, 'ok') || ~strcmp(setpointStatus, 'ok')
    return;
end

errorVector = quat_error(nav.data.quaternion, setpoint.data.quaternion);
omega = params.attitude.Kp .* errorVector + params.attitude.omega_ff(:);
omega = min(max(omega, -params.attitude.rate_limit), params.attitude.rate_limit);

payload = struct('omega', omega, 'alpha', zeros(3, 1));
bus.topics.publish(params.output_topic, payload, t, t, t, 'attitude_p');

end
