function [q, zBody] = attitude_from_thrust_direction(zBody, yawRad)
%ATTITUDE_FROM_THRUST_DIRECTION 由推力方向与偏航构造期望姿态
%
% 输入：
%   zBody  - 期望机体系 z 轴（FRD，单位向量，推力沿 −zBody）
%   yawRad - 期望偏航角（rad，NED 航向）
%
% 输出：
%   q      - 期望 q_bn（标量在前 Hamilton）
%   zBody  - 归一化后的机体 z 轴
%
% 由推力方向与偏航构造旋转矩阵；本函数不重新计算推力大小，供
% Velocity 级在完成 PX4 推力/倾角限幅后使用。

zBody = zBody(:);
zBody = zBody / max(norm(zBody), 1e-12);
heading = [cos(yawRad); sin(yawRad); 0];
yBody = cross(zBody, heading);
if norm(yBody) < 1e-9
    yBody = cross(zBody, [0; 1; 0]);
    if norm(yBody) < 1e-9
        yBody = cross(zBody, [1; 0; 0]);
    end
end
yBody = yBody / norm(yBody);
xBody = cross(yBody, zBody);
xBody = xBody / norm(xBody);
yBody = cross(zBody, xBody);

q = rotation_to_quat([xBody, yBody, zBody]);

end
