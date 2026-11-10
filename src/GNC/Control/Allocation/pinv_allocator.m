function bus = pinv_allocator(t, bus, params)
%PINV_ALLOCATOR 六舵面伪逆控制分配（独立一级，默认 400 Hz）
%
% 输入 Topic：
%   control/torque_setpoint      期望力矩 τ_d（N·m，FRD，**相对配平的增量**）
%   control/attitude_setpoint    总推力 T（N，可选）
% 输出：
%   control/allocation_status    δ、achieved、residual、saturated、motor、Ve_nom
%   bus.command                  servos（6×1 rad）+ motor（rad/s）
%
% 分配律（PX4 PseudoInverse 的 SI 版本）：
%   B_tau = d_cs·Ve²·diag([l1,l1,l2])·B_M        [N·m/rad]
%   δ = δ_trim + pinv(B_tau)·τ_d
%   achieved_control = B_tau·δ − B_tau·δ_trim
%   residual = τ_d − achieved_control
%
% 定义：
%   - τ_d 是**相对配平舵角**的力矩增量，不是绝对总力矩；τ_d=0 时 δ=δ_trim，
%     车辆保持悬停配平（涵道反扭矩与固定片扭矩已由 trim 抵消）。
%   - δ_trim 是共同配平舵角，六片相同；它不参与反馈，只作为分配基线。
%   - residual 为“未实现的力矩增量”；saturated(axis)=+1 表示正方向不足，
%     −1 表示负方向不足，0 表示未饱和。角速度环据此冻结积分。
%
% Ve 由**指令推力**按名义涵道模型计算，不读取真实瞬时出口流速。电机转速
% 由 Ω=sqrt(T/k_T0) 独立求解；PINV 只解六舵面力矩，不把七执行器混在一起。

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

veNominal = sqrt(thrust / (params.diffuser_ratio * params.air_density * params.disc_area));
veNominal = max(veNominal, params.ve_min_m_s);
Btau = params.servo_torque_coefficient * veNominal^2 * ...
    diag([params.l1_m, params.l1_m, params.l2_m]) * params.B_M;

trim = params.trim_delta_rad * ones(6, 1);
torqueTrim = Btau * trim;
delta = trim + pinv(Btau) * torqueDesired;

% 生效行程限幅（非对称）
deltaClipped = min(max(delta, params.delta_limit(1)), params.delta_limit(2));

achieved = Btau * deltaClipped;
achievedControl = achieved - torqueTrim;
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
bus.topics.publish(params.status_topic, payload, t, t, t, 'pinv_allocator');

bus.command = struct('servos', deltaClipped, 'motor', motor);

end
