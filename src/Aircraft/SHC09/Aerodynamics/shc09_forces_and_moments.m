function [F_b, M_b, Ve, detail] = shc09_forces_and_moments(v_air_body, omega, delta, Omega, p, aero)
%SHC09_FORCES_AND_MOMENTS 合成 SHC09 全部气动力与力矩（FRD，不含外部扰动）
%
% 输入：
%   v_air_body - 机体系相对空气速度（FRD，3×1，m/s）
%   omega      - 机体角速度（FRD，3×1，rad/s）
%   delta      - 六片实际舵面偏角（6×1，rad）
%   Omega      - 实际旋翼转速（rad/s）
%
% 输出：
%   F_b    - 机体系气动力（3×1，N）
%   M_b    - 机体系气动力矩（3×1，N·m）
%   Ve     - 出口流速（m/s）
%   detail - 各分项，便于输出与测试核对
%
% 力的方向（由 FRD 与参考模型逐项换算）：
%   F_x = -N·Cduct - L_wing
%   F_y =  N·Sduct
%   F_z = -T - D_wing
% 力矩：
%   M_x =  M_duct·Sduct
%   M_y =  M_duct·Cduct + M_wing - M_my
%   M_z =  M_fan + M_sta + M_cs,z + M_gyro,z
% 其中 Sduct = -v_y/V_uv、Cduct = v_x/V_uv 为横向气流分解。

[duct, Ve] = shc09_duct(v_air_body, Omega, p, aero);
wing = shc09_wing(v_air_body, p, aero);

u = v_air_body(1);
v = v_air_body(2);
Vuv = sqrt(u^2 + v^2);
if Vuv <= 1e-12
    Sduct = 0;
    Cduct = 1;
else
    Sduct = -v / Vuv;
    Cduct = u / Vuv;
end

F_b = [ -duct(2) * Cduct - wing(1);
         duct(2) * Sduct;
        -duct(1) - wing(2) ];

Vvw = sqrt(u^2 + v_air_body(3)^2);
M_my = omega(2) * (p.damping_k_my0 + p.damping_k_myv * Vvw);
M_aero = [ duct(3) * Sduct;
           duct(3) * Cduct + wing(3) - M_my;
           0 ];

Dir = p.rotor_direction;
M_gyro = -Dir * p.fan_inertia_kg_m2 * Omega * [omega(2); -omega(1); 0];
M_fan = -Dir * duct(4);
M_sta = Dir * p.fixed_vane_coefficient * Ve^2;
M_cs = shc09_control_surfaces(delta, Ve, p);

M_b = M_aero + M_gyro + [0; 0; M_fan + M_sta] + M_cs;

detail = struct();
detail.duct = duct;
detail.wing = wing;
detail.exit_speed_m_s = Ve;
detail.Sduct = Sduct;
detail.Cduct = Cduct;
detail.M_aero = M_aero;
detail.M_gyro = M_gyro;
detail.M_fan = M_fan;
detail.M_sta = M_sta;
detail.M_cs = M_cs;

end
