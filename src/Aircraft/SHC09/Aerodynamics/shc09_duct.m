function [y, Ve] = shc09_duct(v_air_body, Omega, p, aero)
%SHC09_DUCT 涵道风扇气动力、力矩与出口流速（FRD）
%
% 输入：
%   v_air_body - 机体系相对空气速度（FRD，3×1，m/s）
%   Omega      - 旋翼实际转速（rad/s，绕 +Z 正）
%   p          - SHC09 参数
%   aero       - 气动样条数据
%
% 输出：
%   y  - [拉力; 侧向力; 俯仰力矩; 风扇反扭矩]（标量，N/N·m）
%        约定：拉力沿 -Z；y(4) 为空气动力反扭矩大小，方向由符号约定处理
%   Ve - 出口流速（m/s）
%
% 方程（与参考模型一致，已换到 FRD 的力/力矩方向）：
%   α_df = acos(-w/Va)，Va 为总空速
%   T    = k_T0 Ω² + Va Ω (k_Th + k_Ts cosα) + Va² dt(α)
%   N    = Va Ω k_Ns sinα + Va² dn(α)
%   M    = N l_cpz + T k_cpx (Va/Ω) sinα （Ω>0）
%   Q    = k_Q0 Ω²
%   Ve   = -w/2 + sqrt((w/2)² + T/(sd ρ S))

u = v_air_body(1);
v = v_air_body(2);
w = v_air_body(3);

Va = sqrt(u^2 + v^2 + w^2);
Vuv = sqrt(u^2 + v^2);

if Va <= 1e-6
    cosA = 0;
    sinA = 1;
    angle = pi / 2;
else
    cosA = max(min(-w / Va, 1), -1);
    angle = acos(cosA);
    sinA = Vuv / Va;
end

policy = 'error';
if isfield(p, 'aero_domain_policy')
    policy = p.aero_domain_policy;
end

dt = shc09_eval_table(aero.dt, angle, aero.duct_domain_rad, policy, 'duct.dt');
dn = shc09_eval_table(aero.dn, angle, aero.duct_domain_rad, policy, 'duct.dn');

D = p.duct;
y1 = D.k_T0 * Omega^2 + Va * Omega * (D.k_Th + D.k_Ts * cosA) + Va^2 * dt;
y2 = Va * Omega * D.k_Ns * sinA + Va^2 * dn;

if Omega > 0
    y3 = y2 * D.l_cpz + y1 * D.k_cpx * (Va / Omega) * sinA;
else
    y3 = y2 * D.l_cpz;
end
y4 = D.k_Q0 * Omega^2;

inner = (w / 2)^2 + y1 / (p.diffuser_ratio * p.air_density_kg_m3 * p.disc_area_m2);
if inner < 0
    error('shc09:duct:exitSpeed', ...
        '出口流速计算出现负根（w=%.6f, T=%.6f），检查推力模型与状态', w, y1);
end
Ve = -w / 2 + sqrt(inner);

y = [y1; y2; y3; y4];

end
