function aero = shc09_aero_fixture()
%SHC09_AERO_FIXTURE 测试用合成气动系数（非物理数据）
%
% 仅用于验证代码通路、符号与积分行为：所有气动表系数恒为 0。
% 涵道推力仍由参数 k_T0/k_Th 决定，因此可以检查旋翼推力与反扭矩符号；
% 但机翼升力、阻力、力矩恒为零，不能用于配平精度、性能或气动验证。
%
% 真实物理验证必须使用 shc09_aero_data('external') 加载有授权的外部数据。

anglesDuct = [0, 2.2689];        % rad，覆盖涵道表有效域
anglesWing = [-0.3491, 2.2689];  % rad，覆盖机翼表有效域

aero = struct();
aero.dt = spline(anglesDuct, [0, 0]);
aero.dn = spline(anglesDuct, [0, 0]);
aero.dm = spline(anglesDuct, [0, 0]);
aero.wl = spline(anglesWing, [0, 0]);
aero.wd = spline(anglesWing, [0, 0]);
aero.wm = spline(anglesWing, [0, 0]);
aero.duct_domain_rad = anglesDuct;
aero.wing_domain_rad = anglesWing;
aero.source_type = 'fixture';
aero.source_file = '';
aero.is_physical = false;

end
