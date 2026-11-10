function e = quat_error(q, q_desired)
%QUAT_ERROR 机体四元数姿态误差（标量在前，Hamilton 约定）
%
% 输入：
%   q         - 当前 q_bn（机体系到 NED，4×1）
%   q_desired - 期望 q_bn（4×1）
%
% 输出：
%   e - 机体系姿态误差向量，e = 2·vec(q⁻¹ ⊗ q_d)，小角度下约等于
%       “把当前姿态转到期望姿态”的旋转向量。q 与 -q 表示同一姿态，
%       因此先规范化符号，保证最短旋转。
%
% 该定义对应 PX4 AttitudeControl 的 quaternion-imag 误差（2·n·sin(θ/2)），
% 不做 DCM 反对称矩阵提取，避免在 θ≈π 时的数值病态。

q = q(:) / norm(q);
q_desired = q_desired(:) / norm(q_desired);

% q⁻¹ = [w; -v]
qe = [ ...
    q(1) * q_desired(1) + q(2:4).' * q_desired(2:4);
    q(1) * q_desired(2:4) - q_desired(1) * q(2:4) - cross(q(2:4), q_desired(2:4))];

if qe(1) < 0
    qe = -qe;
end
e = 2 * qe(2:4);

end
