function R = shc09_quat_to_dcm(q)
%SHC09_QUAT_TO_DCM 四元数转机体系→NED 旋转矩阵
%
% q = [w x y z]（标量在前，Hamilton 约定），表示机体系在 NED 中的姿态。
% 返回 R_nb，使 v_ned = R_nb * v_body。
%
% 公式：
%   R = [1-2(y²+z²)   2(xy-wz)     2(xz+wy);
%        2(xy+wz)     1-2(x²+z²)   2(yz-wx);
%        2(xz-wy)     2(yz+wx)     1-2(x²+y²)]

w = q(1);
x = q(2);
y = q(3);
z = q(4);

R = [1 - 2*(y^2 + z^2), 2*(x*y - w*z),     2*(x*z + w*y);
     2*(x*y + w*z),     1 - 2*(x^2 + z^2), 2*(y*z - w*x);
     2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x^2 + y^2)];

end
