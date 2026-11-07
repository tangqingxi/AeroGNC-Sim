function model = first_order_model(a, b)
%FIRST_ORDER_MODEL dx/dt = -a*x + b*u with analytic step response.
%
%   For x(0) = 0 and a constant u applied from t0:
%       x(t) = (b*u/a) * (1 - exp(-a*(t - t0)))
%   Test double only; not a flight-dynamics model.

if nargin < 1, a = 2; end
if nargin < 2, b = 3; end

model = struct();
model.name = sprintf('first-order(a=%.6g,b=%.6g)', a, b);
model.x0 = 0;
model.dynamics = @(t, x, u, env) -a * x + b * u; %#ok<INUSD>

end
