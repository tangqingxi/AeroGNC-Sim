function model = diagonal_model(lambda, ~)
%DIAGONAL_MODEL 对角多状态系统：dx_i/dt = -lambda_i*x_i + u
%
% 用途：
%   验证内核支持任意状态维度的测试替身，不是飞行器模型。
%
% 参数：
%   lambda - n×1 正实数向量，元素单位 1/s。
%
% 状态：
%   x 为 n×1 列向量，第 i 个分量对应 lambda_i；输入 u 对标量（广播到各状态）。
%
% 解析解（x(0)=0，常值 u，从 t0 起）：
%   x_i(t) = (u/lambda_i) * (1 - exp(-lambda_i*(t - t0)))

if nargin < 1
    lambda = [1; 2; 3];
end
lambda = lambda(:);

model = struct();
model.name = sprintf('diagonal(%s)', mat2str(lambda.'));
model.x0 = zeros(numel(lambda), 1);
model.dynamics = @(t, x, u, env) -lambda .* x + u; %#ok<INUSD>

end
