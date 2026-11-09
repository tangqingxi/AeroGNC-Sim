function y = shc09_wing(v_air_body, p, aero)
%SHC09_WING 机翼气动力与力矩（FRD）
%
% 输入：
%   v_air_body - 机体系相对空气速度（FRD，3×1，m/s）
%   p          - SHC09 参数
%   aero       - 气动样条数据
%
% 输出：
%   y = [升力相关项; 阻力相关项; 绕机体 X 轴力矩]（标量，N/N·m）
%
% 方程：
%   V_vw = sqrt(u² + w²)（前向-垂向平面空速）
%   α_w  = acos(-w/V_vw)（V_vw ≤ 1 时取 π/2，与参考实现一致）
%   L    = V_vw² wl(α_w)
%   D    = V_vw² wd(α_w)
%   M    = V_vw² wm(α_w)
%
% 注意：机翼迎角与涵道迎角定义不同（涵道用总空速，机翼只用前向-垂向
% 平面），不得合并成一个变量。

u = v_air_body(1);
w = v_air_body(3);

Vvw = sqrt(u^2 + w^2);
if Vvw <= 1
    angle = pi / 2;
else
    angle = acos(max(min(-w / Vvw, 1), -1));
end

policy = 'error';
if isfield(p, 'aero_domain_policy')
    policy = p.aero_domain_policy;
end

wl = shc09_eval_table(aero.wl, angle, aero.wing_domain_rad, policy, 'wing.wl');
wd = shc09_eval_table(aero.wd, angle, aero.wing_domain_rad, policy, 'wing.wd');
wm = shc09_eval_table(aero.wm, angle, aero.wing_domain_rad, policy, 'wing.wm');

y = [Vvw^2 * wl; Vvw^2 * wd; Vvw^2 * wm];

end
