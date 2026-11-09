function [M, B_tau, B_M] = shc09_control_surfaces(delta, Ve, p)
%SHC09_CONTROL_SURFACES 六舵面力矩与效能矩阵
%
% 输入：
%   delta - 六片实际舵面偏角（6×1，rad）
%   Ve    - 出口流速（m/s）
%   p     - SHC09 参数
%
% 输出：
%   M     - 机体力矩（3×1，N·m，FRD）
%   B_tau - 力矩效能矩阵（3×6，N·m/rad）
%   B_M   - 无量纲力矩效能矩阵（3×6）
%
% 三种矩阵不得混用：
%   B_M   无量纲几何矩阵
%   B_tau = d_cs Ve² diag([l1,l1,l2]) B_M
%   B_acc = J⁻¹ B_tau（由动力学使用，见 shc09_dynamics）

g = shc09_control_surface_geometry(p);
B_M = g.B_M;
arm = diag([p.l1_m, p.l1_m, p.l2_m]);
B_tau = p.servo_torque_coefficient * Ve^2 * arm * B_M;
M = B_tau * delta(:);

end
