function info = test_first_order()
%TEST_FIRST_ORDER dx/dt = -a*x + b*u compared with the analytic step response.

a = 2;
b = 3;
model = first_order_model(a, b);
cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.components = struct('name', 'latch', 'kind', 'periodic', 'period', 0.2, ...
    'priority', 50, 'latches', true, 'step', @latch_step);

res = aero_sim(model, cfg);

expected = zeros(numel(res.time), 1);
for k = 1:numel(res.time)
    t = res.time(k);
    if t < 0.2 - 1e-12
        expected(k) = 0;
    else
        expected(k) = (b / a) * (1 - exp(-a * (t - 0.2)));
    end
end

err = max(abs(res.state(:, 1) - expected));
check_that(err <= 1e-9, 'first order: max error %.3e exceeds 1e-9', err);
check_that(abs(res.state(end, 1) - 1.5 * (1 - exp(-2 * 0.8))) <= 1e-9, ...
    'final state %.12g differs from analytic %.12g', ...
    res.state(end, 1), 1.5 * (1 - exp(-2 * 0.8)));
check_that(strcmp(res.model, model.name), 'model name not recorded');

info = struct('passed', true, 'summary', sprintf( ...
    'RelTol=%.0e AbsTol=%.0e max|x-x_analytic|=%.2e (analytic x(1)=%.6f)', ...
    cfg.solver.RelTol, cfg.solver.AbsTol, err, 1.5 * (1 - exp(-2 * 0.8))));

end

function bus = latch_step(t, bus)
if t >= 0.2 - 1e-12
    bus.held_command = 1;
else
    bus.held_command = 0;
end
end
