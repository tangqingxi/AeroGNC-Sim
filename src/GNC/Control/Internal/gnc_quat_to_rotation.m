function R = gnc_quat_to_rotation(q)
%GNC_QUAT_TO_ROTATION 四元数（标量在前，Hamilton）转旋转矩阵
%
% q 表示从机体系 FRD 到 NED 的旋转：v_n = R * v_b。
% 输入不要求严格单位化，内部归一化；零四元数直接报错。

q = q(:);
if numel(q) ~= 4 || any(~isfinite(q))
    error('gnc:quat:format', '四元数必须是 4 个有限实数');
end
n = norm(q);
if n < 1e-12
    error('gnc:quat:zero', '四元数模长为零');
end
q = q / n;
w = q(1);
v = q(2:4);

R = (w^2 - v.' * v) * eye(3) + 2 * (v * v.') + 2 * w * skew(v);

end

% ===========================================================================
function S = skew(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
