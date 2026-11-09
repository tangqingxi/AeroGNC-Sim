function qdot = shc09_quat_kinematics(q, omega_body)
%SHC09_QUAT_KINEMATICS 姿态四元数微分方程
%
% 输入：
%   q          - [w x y z]，机体系→NED
%   omega_body - 机体系角速度（FRD，rad/s）
%
% 输出：
%   qdot       - 四元数导数
%
% 方程（Hamilton 乘法，右乘机体系角速度）：
%   q̇ = 0.5 · q ⊗ [0; ω_x; ω_y; ω_z]
%
% 展开：
%   ẇ = -0.5 (x ωx + y ωy + z ωz)
%   ẋ =  0.5 (w ωx + y ωz - z ωy)
%   ẏ =  0.5 (w ωy - x ωz + z ωx)
%   ż =  0.5 (w ωz + x ωy - y ωx)
%
% 范数由 Aircraft 的状态投影（shc09_project）保持，不在导数中做归一化。

w = q(1);
x = q(2);
y = q(3);
z = q(4);
wx = omega_body(1);
wy = omega_body(2);
wz = omega_body(3);

qdot = 0.5 * [ -(x*wx + y*wy + z*wz);
                w*wx + y*wz - z*wy;
                w*wy - x*wz + z*wx;
                w*wz + x*wy - y*wx ];

end
