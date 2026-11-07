function [x1, info] = aero_integrate(model, t0, t1, x0, u, env, solver)
%AERO_INTEGRATE 在单个事件段内用 ode45 积分
%
% 输入：
%   model  - 含 dynamics 函数句柄的模型结构
%   t0, t1 - 段起止时刻（s），要求 t1 > t0
%   x0     - 段初状态（列向量）
%   u      - 段内恒定的锁存输入（零阶保持）
%   env    - 原样传递给动力学方程的环境结构
%   solver - RelTol/AbsTol/MaxStep/InitialStep
%
% 输出：
%   x1   - 段末状态
%   info - status（ok/skipped）、internal_points（ode45 内部点数）、t（内部时间点）
%
% 说明：
%   ode45 的内部步长由容差自适应决定，info.t 只用于诊断“积分点不是采样点”，
%   不得作为离散更新或日志的时间基准。

if t1 <= t0
    x1 = x0;
    info = struct('status', 'skipped', 'internal_points', 0, 't', t0);
    return;
end

options = odeset('RelTol', solver.RelTol, 'AbsTol', solver.AbsTol);
if isfinite(solver.MaxStep)
    options = odeset(options, 'MaxStep', solver.MaxStep);
end
if ~isempty(solver.InitialStep)
    options = odeset(options, 'InitialStep', solver.InitialStep);
end

rhs = @(t, x) model.dynamics(t, x, u, env);
try
    [tOut, xOut] = ode45(rhs, [t0, t1], x0, options);
catch err
    error('aero:integration:failed', ...
        'ode45 在区间 [%.17g, %.17g] 积分失败：%s', t0, t1, err.message);
end

if any(~isfinite(xOut(:)))
    error('aero:integration:nonfinite', ...
        'ode45 在区间 [%.17g, %.17g] 产生了非有限状态', t0, t1);
end

x1 = xOut(end, :).';
info = struct('status', 'ok', 'internal_points', numel(tOut), 't', tOut(:));

end
