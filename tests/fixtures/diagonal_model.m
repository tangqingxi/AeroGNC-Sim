function model = diagonal_model(lambda)
%DIAGONAL_MODEL dx_i/dt = -lambda_i*x_i + u, an n-state analytic test model.

if nargin < 1
    lambda = [1; 2; 3];
end
lambda = lambda(:);

model = struct();
model.name = sprintf('diagonal(%s)', mat2str(lambda.'));
model.x0 = zeros(numel(lambda), 1);
model.dynamics = @(t, x, u, env) -lambda .* x + u; %#ok<INUSD>

end
