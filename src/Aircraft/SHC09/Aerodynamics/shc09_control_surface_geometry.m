function g = shc09_control_surface_geometry(p)
%SHC09_CONTROL_SURFACE_GEOMETRY 六舵面安装几何与效能矩阵（唯一定义处）
%
% 输出 g：
%   phi_rad     六舵面方位角（rad），CS1..CS6 = 0,60,...,300°
%   position_m  舵面位置 r_i = [l2 cosφ; l2 sinφ; l1]（FRD，m）
%   axis        正向旋转轴 a_i = [cosφ; sinφ; 0]（径向向外）
%   force_dir   正偏角产生的气动合力方向 [-sinφ; cosφ; 0]（切向）
%   B_M         力矩效能矩阵（无量纲），列对应 CS1..CS6
%
% 力矩关系：
%   B_tau = d_cs · Ve² · diag([l1, l1, l2]) · B_M
%   M_cs  = B_tau · delta
% 其中 B_M 的第 i 列等于 (r_i × force_dir_i) 除以 diag([l1,l1,l2])，
% 即由几何直接导出，测试会核对它与冻结矩阵一致。

if ~isfield(p, 'control_surface_angle_rad')
    error('shc09:geometry:parameters', '缺少 control_surface_angle_rad');
end
phi = p.control_surface_angle_rad(:).';
n = numel(phi);

g = struct();
g.phi_rad = phi;
g.position_m = [p.l2_m * cos(phi); p.l2_m * sin(phi); p.l1_m * ones(1, n)];
g.axis = [cos(phi); sin(phi); zeros(1, n)];
g.force_dir = [-sin(phi); cos(phi); zeros(1, n)];
g.B_M = [-cos(phi); -sin(phi); ones(1, n)];

end
