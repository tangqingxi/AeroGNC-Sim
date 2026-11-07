function model = scalar_integrator_model()
%SCALAR_INTEGRATOR_MODEL dy/dt = u, one state, unit gain on the latched input.
%
%   Analytic response for a piecewise-constant u is y(t) = y(0) + integral(u).
%   This model is a test double for the kernel, not an Aircraft implementation.

model = struct();
model.name = 'scalar-integrator';
model.x0 = 0;
model.dynamics = @(t, x, u, env) u; %#ok<INUSD>

end
