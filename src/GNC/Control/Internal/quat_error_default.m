function eq = quat_error_default(q, qd, yawWeight)
%QUAT_ERROR_DEFAULT PX4 默认 reduced-attitude + yaw-weight 姿态误差
%
% 输入：
%   q         - 当前 q_bn（机体系到 NED，标量在前 Hamilton）
%   qd        - 期望 q_bn
%   yawWeight - 偏航权重 [0,1]；PX4 MC_YAW_WEIGHT
%
% 输出：
%   eq - 机体系误差向量，PX4 默认模式（ATTITUDE_ERROR_DEFAULT）：
%        先由当前/期望推力轴构造 reduced attitude，再按权重缩放偏航差，
%        最后取 eq = 2·vec(q⁻¹⊗q_d)。
%
% 对应 PX4 AttitudeControl::calculateAttitudeErrorDefault()。仅实现默认
% 路径；DCM log、tilt-prioritized 等可选模式不在 V1.0 范围内。

q = q(:) / norm(q);
qd = qd(:) / norm(qd);
yawWeight = min(max(yawWeight, 0), 1);

eZ = quat_rotate(q, [0; 0; 1]);
eZd = quat_rotate(qd, [0; 0; 1]);
qTilt = quat_between(eZ, eZd);

if abs(qTilt(2)) > (1 - 1e-5) || abs(qTilt(3)) > (1 - 1e-5)
    qdReduced = qd;
else
    qTiltBody = quat_mul(quat_mul(quat_conj(q), qTilt), q);
    qdReduced = quat_mul(q, qTiltBody);
end

qDeltaYaw = quat_mul(quat_conj(qdReduced), qd);
qDeltaYaw = quat_canonical(qDeltaYaw);
qDeltaYaw(1) = min(max(qDeltaYaw(1), -1), 1);
qDeltaYaw(4) = min(max(qDeltaYaw(4), -1), 1);

qYawWeighted = [cos(yawWeight * acos(qDeltaYaw(1))); 0; 0; ...
    sin(yawWeight * asin(qDeltaYaw(4)))];
qdWeighted = quat_mul(qdReduced, qYawWeighted);

qError = quat_canonical(quat_mul(quat_conj(q), qdWeighted));
eq = 2 * qError(2:4);

end

% ===========================================================================
function q = quat_mul(a, b)
w = a(1) * b(1) - a(2:4).' * b(2:4);
v = a(1) * b(2:4) + b(1) * a(2:4) + cross(a(2:4), b(2:4));
q = [w; v];
end

function q = quat_conj(q)
q = [q(1); -q(2:4)];
end

function q = quat_canonical(q)
q = q / norm(q);
if q(1) < 0
    q = -q;
elseif abs(q(1)) < 1e-12
    idx = find(abs(q(2:4)) > 1e-12, 1, 'first');
    if ~isempty(idx) && q(idx + 1) < 0
        q = -q;
    end
end
end

function v = quat_rotate(q, v)
v = quat_to_rotation(q) * v;
end

function q = quat_between(v1, v2)
v1 = v1 / max(norm(v1), 1e-12);
v2 = v2 / max(norm(v2), 1e-12);
c = min(1, max(-1, v1.' * v2));
if c < -1 + 1e-9
    axis = cross(v1, [1; 0; 0]);
    if norm(axis) < 1e-9
        axis = cross(v1, [0; 1; 0]);
    end
    axis = axis / norm(axis);
    q = [0; axis];
else
    q = [1 + c; cross(v1, v2)];
    q = q / norm(q);
end
end
