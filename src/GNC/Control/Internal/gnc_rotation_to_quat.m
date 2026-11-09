function q = gnc_rotation_to_quat(R)
%GNC_ROTATION_TO_QUAT 旋转矩阵转四元数（标量在前，Hamilton）
%
% 使用 Shepperd 分支：比较迹与三个对角元，选择数值最稳定的分支，
% 避免 θ≈π 时直接除以小量。

R = double(R);
if ~isequal(size(R), [3 3]) || any(~isfinite(R(:)))
    error('gnc:quat:rotation', '旋转矩阵必须是 3×3 有限实数');
end

tr = trace(R);
if tr > 0
    s = sqrt(tr + 1) * 2;
    q = [0.25 * s;
          (R(3, 2) - R(2, 3)) / s;
          (R(1, 3) - R(3, 1)) / s;
          (R(2, 1) - R(1, 2)) / s];
elseif R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
    s = sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3)) * 2;
    q = [(R(3, 2) - R(2, 3)) / s;
         0.25 * s;
         (R(1, 2) + R(2, 1)) / s;
         (R(1, 3) + R(3, 1)) / s];
elseif R(2, 2) > R(3, 3)
    s = sqrt(1 + R(2, 2) - R(1, 1) - R(3, 3)) * 2;
    q = [(R(1, 3) - R(3, 1)) / s;
         (R(1, 2) + R(2, 1)) / s;
         0.25 * s;
         (R(2, 3) + R(3, 2)) / s];
else
    s = sqrt(1 + R(3, 3) - R(1, 1) - R(2, 2)) * 2;
    q = [(R(2, 1) - R(1, 2)) / s;
         (R(1, 3) + R(3, 1)) / s;
         (R(2, 3) + R(3, 2)) / s;
         0.25 * s];
end
q = q / norm(q);
if q(1) < 0
    q = -q;
end

end
