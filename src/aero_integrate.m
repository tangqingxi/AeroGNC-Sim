function [x1, info] = aero_integrate(model, t0, t1, x0, u, env, solver)
%AERO_INTEGRATE Integrate one constant-input segment with ode45.
%
%   [x1, info] = aero_integrate(model, t0, t1, x0, u, env, solver) integrates
%   dx = model.dynamics(t, x, u, env) from t0 to t1 with the latched input u
%   held constant. The solver is adaptive; info.t contains the internal
%   points chosen by ode45 and is never treated as a sampling grid.

if t1 <= t0
    x1 = x0;
    info = struct('status', 'skipped', 'internal_points', 0, 't', t0);
    return;
end

opts = odeset('RelTol', solver.RelTol, 'AbsTol', solver.AbsTol);
if isfinite(solver.MaxStep)
    opts = odeset(opts, 'MaxStep', solver.MaxStep);
end
if ~isempty(solver.InitialStep)
    opts = odeset(opts, 'InitialStep', solver.InitialStep);
end

rhs = @(t, x) model.dynamics(t, x, u, env);
try
    [tOut, xOut] = ode45(rhs, [t0, t1], x0, opts);
catch err
    error('aero:integration:failed', ...
        'ode45 failed on segment [%.17g, %.17g]: %s', t0, t1, err.message);
end

if any(~isfinite(xOut(:)))
    error('aero:integration:nonfinite', ...
        'ode45 produced non-finite state on segment [%.17g, %.17g]', t0, t1);
end

x1 = xOut(end, :).';
info = struct('status', 'ok', 'internal_points', numel(tOut), 't', tOut(:));

end
