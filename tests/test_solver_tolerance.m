function info = test_solver_tolerance()
%TEST_SOLVER_TOLERANCE Changing solver tolerances never moves declared events.

model = first_order_model(2, 3);

base = aero_default_config();
base.tspan = [0, 1];
base.x0 = 0;
base.initial_command = 0;
base.components = struct('name', 'latch', 'kind', 'periodic', 'period', 0.2, ...
    'priority', 50, 'latches', true, 'step', @latch_step);

loose = base;
loose.solver = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
tight = base;
tight.solver = struct('RelTol', 1e-10, 'AbsTol', 1e-12, 'MaxStep', Inf, 'InitialStep', []);

resLoose = aero_sim(model, loose);
resTight = aero_sim(model, tight);

check_that(isequal(resLoose.time, resTight.time), ...
    'event/boundary times changed with solver tolerance (%d vs %d groups)', ...
    numel(resLoose.time), numel(resTight.time));
check_that(isequal([resLoose.events.time], [resTight.events.time]), ...
    'event timestamps changed with solver tolerance');

d = max(abs(resLoose.state(:) - resTight.state(:)));
check_that(d <= 1e-4, 'state difference %.3e between tolerances exceeds 1e-4', d);

info = struct('passed', true, 'summary', sprintf( ...
    'identical event times; max state difference between RelTol 1e-6/1e-10 = %.2e', d));

end

function bus = latch_step(t, bus)
if t >= 0.2 - 1e-12
    bus.held_command = 1;
else
    bus.held_command = 0;
end
end
