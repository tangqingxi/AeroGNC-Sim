function trim = shc09_trim(p)
%SHC09_TRIM SHC09 静态悬停配平（零空速、水平姿态、零角速度）
%
% 输出 trim：
%   omega_rad_s     悬停旋翼转速
%   exit_speed_m_s  悬停出口流速
%   delta_rad       六舵面共同配平偏角（抵消旋翼反扭矩与固定片扭矩）
%   x0              对应的 20 维初始状态
%   command         对应的初始指令容器
%   residual        配平条件说明
%
% 推导（悬停时 Va=0，机翼项为 0，涵道表项因乘 Va² 也为 0）：
%   T = m g = k_T0 Ω²                     → Ω = sqrt(m g / k_T0)
%   Ve = sqrt(T / (sd ρ S))
%   M_fan = −Dir k_Q0 Ω²，M_sta = Dir k_sta Ve²
%   六舵共同偏角产生的偏航力矩 = d_cs Ve² l2 · 6δ
%   δ = −(M_fan + M_sta) / (6 d_cs Ve² l2)
%
% 配平不依赖气动表；若 δ 超过机械行程，直接报错说明物理原因，不修改参数。

m = p.mass_kg;
g = p.gravity_m_s2;
T = m * g;

Omega = sqrt(T / p.duct.k_T0);
Ve = sqrt(T / (p.diffuser_ratio * p.air_density_kg_m3 * p.disc_area_m2));

Dir = p.rotor_direction;
M_fan = -Dir * p.duct.k_Q0 * Omega^2;
M_sta = Dir * p.fixed_vane_coefficient * Ve^2;
denominator = 6 * p.servo_torque_coefficient * Ve^2 * p.l2_m;
delta = -(M_fan + M_sta) / denominator;

if abs(delta) > p.servo_effective_range_rad(2)
    error('shc09:trim:deflection', ...
        '配平舵角 %.3f° 超过生效行程 [%.1f°, %.1f°]；请检查旋翼反扭矩与舵面参数，不得强行改参数', ...
        rad2deg(delta), rad2deg(p.servo_effective_range_rad(1)), ...
        rad2deg(p.servo_effective_range_rad(2)));
end

s = shc09_state_layout();
x0 = zeros(s.nx, 1);
x0(s.quaternion) = [1; 0; 0; 0];
x0(s.servo) = delta;
x0(s.motor) = Omega;

trim = struct();
trim.omega_rad_s = Omega;
trim.exit_speed_m_s = Ve;
trim.delta_rad = delta;
trim.delta_deg = rad2deg(delta);
trim.x0 = x0;
trim.command = struct('servos', delta * ones(6, 1), 'motor', Omega);
trim.thrust_n = T;
trim.M_fan_nm = M_fan;
trim.M_sta_nm = M_sta;
trim.residual = 'Va=0 悬停：机翼项为零，涵道表项乘 Va² 为零，配平不依赖气动表';

end
