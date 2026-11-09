function bus = gnc_allocate_pinv_step(t, bus, params)
%GNC_ALLOCATE_PINV_STEP 六舵面伪逆控制分配（400 Hz）
%
% 输入 Topic：
%   control/torque_setpoint      期望力矩 τ_d（N·m，FRD）
%   control/attitude_setpoint    可选；提供总推力 T（N）
% 输出：
%   control/allocation_status    实际舵角、可达力矩、残差、饱和方向
%   bus.command                  servos（6×1 rad）+ motor（rad/s）
%
% 分配律（PX4 PseudoInverse 的 SI 版本）：
%   δ = δ_trim + pinv(B_tau)·(τ_d − B_tau·δ_trim)
%   B_tau = d_cs·Ve²·diag([l1,l1,l2])·B_M
% 其中 Ve 由**指令推力**按名义涵道模型计算，不读取真实瞬时出口流速；
% δ 限幅到生效行程，残差 = τ_d − B_tau·δ_clipped 供角速度环抗饱和。
%
% 电机推力与舵面力矩分开：PINV 只解六舵面力矩，电机转速由总推力需求
% 经 k_T0 名义模型给出，不把七执行器混成一个分配问题。

[torqueMsg, torqueStatus] = bus.topics.tryRead(params.torque_topic, t);
if ~strcmp(torqueStatus, 'ok')
    return;
end
torqueDesired = torqueMsg.data.torque(:);

thrust = params.hover_thrust_n;
[attitudeMsg, attitudeStatus] = bus.topics.tryRead(params.attitude_topic, t);
if strcmp(attitudeStatus, 'ok')
    thrust = attitudeMsg.data.thrust_n;
end
thrust = min(max(thrust, 0), params.max_thrust_n);

% 名义出口流速与力矩效能
veNominal = sqrt(thrust / (params.diffuser_ratio * params.air_density * params.disc_area));
veNominal = max(veNominal, params.ve_min_m_s);
Btau = params.servo_torque_coefficient * veNominal^2 * ...
    diag([params.l1_m, params.l1_m, params.l2_m]) * params.B_M;

trim = params.trim_delta_rad * ones(6, 1);
torqueTrim = Btau * trim;                 % 配平舵角对应的总力矩
% 配平是基线，控制器的力矩需求是相对于配平的增量：
%   δ = δ_trim + pinv(B_tau)·τ_d
% 因此 τ_d = 0 时输出配平舵角，而不是零舵角。
delta = trim + pinv(Btau) * torqueDesired;

% 生效行程限幅（非对称）
deltaClipped = min(max(delta, params.delta_limit(1)), params.delta_limit(2));

achieved = Btau * deltaClipped;
achievedControl = achieved - torqueTrim;  % 相对配平的可控力矩
residual = torqueDesired - achievedControl;
saturated = zeros(3, 1);
for axis = 1:3
    if abs(residual(axis)) > params.saturation_tolerance
        saturated(axis) = sign(residual(axis));
    end
end

% 总推力 → 名义旋翼转速（与舵面分配独立）
motor = sqrt(thrust / params.k_T0);
motor = min(max(motor, params.motor_min_rad_s), params.motor_max_rad_s);

payload = struct( ...
    'delta', deltaClipped, ...
    'achieved', achieved, ...
    'residual', residual, ...
    'saturated', saturated, ...
    'motor_rad_s', motor, ...
    've_nominal_m_s', veNominal);
bus.topics.publish(params.status_topic, payload, t, t, t, 'allocation');

bus.command = struct('servos', deltaClipped, 'motor', motor);

if ~isfield(bus, 'gnc')
    bus.gnc = struct();
end
if ~isfield(bus.gnc, 'allocation')
    bus.gnc.allocation = struct();
end
bus.gnc.allocation.saturated = saturated;
bus.gnc.allocation.residual = residual;

end
