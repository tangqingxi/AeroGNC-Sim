function v = constrain_xy(v0, v1, maxNorm)
%CONSTRAIN_XY 按优先级约束二维速度向量（PX4 ControlMath::constrainXY 的 SI 版）
%
% 输入：
%   v0      - 优先分量（位置反馈产生的速度），2×1，m/s
%   v1      - 次优先分量（速度前馈），2×1，m/s
%   maxNorm - 合成速度上限，标量，m/s
%
% 输出：
%   v       - ||v|| <= maxNorm 的合成速度；优先保证 v0 的方向与幅值。
%
% 语义与 PX4 一致：位置反馈分量优先；只有剩余能力才用于速度前馈。不能把
% 两个分量相加后统一缩放，否则位置误差方向会被前馈“拉偏”。

v0 = v0(:);
v1 = v1(:);
if numel(v0) ~= 2 || numel(v1) ~= 2 || ~isfinite(maxNorm) || maxNorm < 0
    error('gnc:control:constrainXY', '输入必须是有限二维向量和非负上限');
end

if norm(v0 + v1) <= maxNorm
    v = v0 + v1;
    return;
end

if norm(v0) >= maxNorm
    if norm(v0) < 1e-12
        v = zeros(2, 1);
    else
        v = v0 / norm(v0) * maxNorm;
    end
    return;
end

if norm(v1 - v0) < 0.001
    v = v0 / max(norm(v0), 1e-12) * maxNorm;
    return;
end

if norm(v0) < 0.001
    v = v1 / max(norm(v1), 1e-12) * maxNorm;
    return;
end

% 解 ||v0 + s·u1|| = maxNorm，取正根 s
u1 = v1 / norm(v1);
m = u1.' * v0;
c = v0.' * v0 - maxNorm^2;
s = -m + sqrt(max(m^2 - c, 0));
v = v0 + u1 * s;

end
