function model = scalar_integrator_model()
%SCALAR_INTEGRATOR_MODEL 单状态积分器：dy/dt = u
%
% 用途：
%   内核数值验证的最小测试替身，不是飞行器模型。
%
% 状态：
%   x(1) = y，单位与 u 的时间积分一致（此处视为无量纲，仅用于测试）。
%
% 输入：
%   u - 锁存控制输入，标量，段内保持恒定（零阶保持）。
%
% 解析解：
%   分段常值 u 下 y(t) = y(0) + ∫u dt，用于逐段对照数值积分结果。

model = struct();
model.name = 'scalar-integrator';
model.x0 = 0;
model.dynamics = @(t, x, u, env) u; %#ok<INUSD>

end
