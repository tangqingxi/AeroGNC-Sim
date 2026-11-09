function info = test_invalid_input()
%TEST_INVALID_INPUT Invalid configurations fail loudly instead of being repaired.

model = scalar_integrator_model();
base = aero_default_config();
base.tspan = [0, 0.1];
base.x0 = 0;
base.initial_command = 0;

bad = base;
bad.tspan = [1, 0];
check_error(@() aero_sim(model, bad), 'aero:config:tspan', 'reversed tspan');

check_error(@() aero_sim(struct('name', 'no-dynamics'), base), ...
    'aero:config:model', 'missing dynamics');

bad = base;
bad.x0 = NaN;
check_error(@() aero_sim(model, bad), 'aero:config:x0', 'NaN initial state');

bad = base;
bad.solver.RelTol = -1;
check_error(@() aero_sim(model, bad), 'aero:config:solver', 'negative RelTol');

bad = base;
bad.save.enabled = true;
bad.save.path = '';
check_error(@() aero_sim(model, bad), 'aero:config:save', 'save without path');

bad = base;
bad.components = struct('name', 'c', 'kind', 'periodic', 'period', 0, 'step', @noop);
check_error(@() aero_sim(model, bad), 'aero:config:period', 'zero period');

bad.components = struct('name', 'c', 'kind', 'periodic', 'period', 0.1, ...
    'offset', 0.1, 'step', @noop);
check_error(@() aero_sim(model, bad), 'aero:config:offset', 'offset >= period');

bad.components = struct('name', 'c', 'kind', 'bogus', 'period', 0.1, 'step', @noop);
check_error(@() aero_sim(model, bad), 'aero:config:kind', 'unknown kind');

bad.components = struct('name', 'c', 'kind', 'sensor', 'period', 0.1);
check_error(@() aero_sim(model, bad), 'aero:config:callback', 'sensor without sample');

bad.components = struct('name', 'c', 'kind', 'periodic', 'period', 0.1);
check_error(@() aero_sim(model, bad), 'aero:config:callback', 'periodic without step');

bad.components = struct('name', 'c', 'kind', 'sensor', 'period', 0.1, ...
    'delay', -1, 'sample', @sample_ok);
check_error(@() aero_sim(model, bad), 'aero:config:delay', 'negative delay');

bad.components = struct('name', {'c', 'c'}, 'kind', 'periodic', ...
    'period', 0.1, 'step', @noop);
check_error(@() aero_sim(model, bad), 'aero:config:duplicate', 'duplicate names');

bad = base;
bad.components = struct('name', 'c', 'kind', 'periodic', 'period', 0.05, ...
    'latches', false, 'step', @illegal_latch);
check_error(@() aero_sim(model, bad), 'aero:kernel:latch', 'latch without permission');

bad = base;
bad.components = struct('name', 'c', 'kind', 'sensor', 'period', 0.05, ...
    'sample', @sample_nan);
check_error(@() aero_sim(model, bad), 'aero:kernel:sample', 'non-finite sensor value');

info = struct('passed', true, 'summary', ...
    '14 invalid inputs rejected with aero:* identifiers, no silent repair');

end

function bus = noop(t, bus) %#ok<INUSD>
end

function v = sample_ok(t, x) %#ok<INUSD>
v = x(1);
end

function v = sample_nan(t, x) %#ok<INUSD>
v = NaN;
end

function bus = illegal_latch(t, bus) %#ok<INUSD>
bus.held_command = 1;
end
