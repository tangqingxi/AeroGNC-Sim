function q = euler_to_quat_bn(roll, pitch, yaw)
%EULER_TO_QUAT_BN 3-2-1 Euler 角 → FRD→NED 四元数（标量在前，Hamilton）
%
% R_bn = Rz(yaw) * Ry(pitch) * Rx(roll)；通过项目现有 rotation_to_quat
% 完成矩阵→四元数转换，不另建一套坐标/四元数约定。
%
% 输入：roll/pitch/yaw（rad，标量）
% 输出：q（4×1，单位化，标量在前）

if ~isscalar(roll) || ~isscalar(pitch) || ~isscalar(yaw) ...
        || any(~isfinite([roll, pitch, yaw]))
    error('gnc:attitude:euler', 'roll/pitch/yaw 必须是有限标量');
end

cr = cos(roll); sr = sin(roll);
cp = cos(pitch); sp = sin(pitch);
cy = cos(yaw); sy = sin(yaw);

Rx = [1, 0, 0; 0, cr, -sr; 0, sr, cr];
Ry = [cp, 0, sp; 0, 1, 0; -sp, 0, cp];
Rz = [cy, -sy, 0; sy, cy, 0; 0, 0, 1];
q = rotation_to_quat(Rz * Ry * Rx);

end
