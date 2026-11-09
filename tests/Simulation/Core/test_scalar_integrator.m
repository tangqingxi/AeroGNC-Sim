function info = test_scalar_integrator()
%TEST_SCALAR_INTEGRATOR dy/dt = u with a step of the latched input at t = 0.5.

model = scalar_integrator_model();
cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = 0;
cfg.initial_command = 1;
cfg.components = struct('name', 'latch', 'kind', 'periodic', 'period', 0.5, ...
    'priority', 50, 'latches', true, 'step', @latch_step);

res = aero_sim(model, cfg);

expected = zeros(numel(res.time), 1);
for k = 1:numel(res.time)
    t = res.time(k);
    if t < 0.5 - 1e-12
        expected(k) = t;
    else
        expected(k) = 0.5 - 2 * (t - 0.5);
    end
end

err = max(abs(res.state(:, 1) - expected));
check_that(err <= 1e-9, 'scalar integrator: max error %.3e exceeds 1e-9', err);
check_that(abs(res.state(end, 1) + 0.5) <= 1e-9, ...
    'final state %.12g differs from analytic -0.5', res.state(end, 1));
check_that(numel(res.segments) == numel(res.time) - 1, 'segment/time length mismatch');

info = struct('passed', true, 'summary', sprintf( ...
    'RelTol=%.0e AbsTol=%.0e max|y-y_analytic|=%.2e (analytic y(1)=-0.5)', ...
    cfg.solver.RelTol, cfg.solver.AbsTol, err));

end

function bus = latch_step(t, bus)
if t >= 0.5 - 1e-12
    bus.held_command = -2;
else
    bus.held_command = 1;
end
end
