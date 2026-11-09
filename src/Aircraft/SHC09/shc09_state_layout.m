function s = shc09_state_layout()
%SHC09_STATE_LAYOUT SHC09 连续状态排列（唯一定义处）
%
% 状态向量共 20 维：
%   [1:3]   位置（NED，m）
%   [4:6]   速度（NED，m/s）
%   [7:10]  姿态四元数 q_bn = [w x y z]（机体系→NED，标量在前，Hamilton）
%   [11:13] 机体角速度（FRD，rad/s）
%   [14:19] 六片舵面实际偏角（rad，CS1..CS6）
%   [20]    旋翼实际转速（rad/s，绕 +Z 右手正向为正）
%
% 任何模块都不得在别处硬编码这些下标；一律通过本结构取索引。

s.nx = 20;
s.position = 1:3;
s.velocity = 4:6;
s.quaternion = 7:10;
s.omega = 11:13;
s.servo = 14:19;
s.motor = 20;
s.units = {'m', 'm/s', '-', 'rad/s', 'rad', 'rad/s'};
s.description = 'SHC09 20-state layout: p_n, v_n, q_bn(wxyz), omega_b, delta(6), Omega';

end
