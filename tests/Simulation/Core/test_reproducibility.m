function info = test_reproducibility()
%TEST_REPRODUCIBILITY Same configuration, environment and seed give equal results.

model = first_order_model(1.5, 2);
cfg = aero_default_config();
cfg.tspan = [0, 0.5];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.seed = 42;
cfg.communication.topics = {struct('name', 'sensor/truth', ...
    'spec', aero_topic_spec({'value', 'sample_time_s'}, {[1 1], [1 1]}, {'-', 's'}))};
cfg.components = struct( ...
    'name', {'truth', 'control'}, ...
    'kind', {'sensor', 'periodic'}, ...
    'period', {0.01, 0.02}, ...
    'priority', {10, 30}, ...
    'delay', {0.005, 0}, ...
    'latches', {false, false}, ...
    'topic', {'sensor/truth', ''}, ...
    'sample', {@truth_sample, []}, ...
    'step', {[], @control_step});

res1 = aero_sim(model, cfg);
res2 = aero_sim(model, cfg);

check_that(isequaln(res1.state, res2.state), 'state differs between identical runs');
check_that(isequal(res1.time, res2.time), 'time vector differs between identical runs');
check_that(isequal(res1.events, res2.events), 'event log differs between identical runs');
check_that(isequal(res1.measurements, res2.measurements), 'measurements differ between identical runs');
check_that(res1.seed == 42, 'seed not recorded in the result');
check_that(~isempty(res1.environment.matlab_version), 'MATLAB version missing from environment');
check_that(~isempty(res1.environment.git_commit), 'git revision missing from environment');

info = struct('passed', true, 'summary', sprintf( ...
    'identical state/time/events/measurements; git=%s dirty=%d; seed=42', ...
    res1.environment.git_commit(1:min(12, numel(res1.environment.git_commit))), ...
    res1.environment.git_dirty));

end

function v = truth_sample(t, x) %#ok<INUSD>
v = x(1);
end

function bus = control_step(t, bus)
[msg, status] = bus.topics.tryRead('sensor/truth', t);
if strcmp(status, 'ok')
    bus.command = -msg.data.value;
end
end
