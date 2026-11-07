function model = first_order_model(a, b)
%FIRST_ORDER_MODEL 一阶线性系统：dx/dt = -a*x + b*u
%
% 用途：
%   验证分段 ode45 与解析阶跃响应一致性的测试替身，不是飞行器模型。
%
% 参数：
%   a - 时间常数倒数，单位 1/s，要求 a > 0
%   b - 输入增益，单位与状态/输入之比一致
%
% 状态：
%   x(1) = x，单位由 b*u 决定。
%
% 解析解（x(0)=0，从 t0 起施加常值 u）：
%   x(t) = (b*u/a) * (1 - exp(-a*(t - t0)))

if nargin < 1, a = 2; end
if nargin < 2, b = 3; end

model = struct();
model.name = sprintf('first-order(a=%.6g,b=%.6g)', a, b);
model.x0 = 0;
model.dynamics = @(t, x, u, env) -a * x + b * u; %#ok<INUSD>

end
