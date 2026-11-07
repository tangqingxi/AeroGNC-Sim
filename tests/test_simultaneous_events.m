function info = test_simultaneous_events()
%TEST_SIMULTANEOUS_EVENTS One instant, six causal steps, deterministic order.

model = scalar_integrator_model();
cfg = aero_default_config();
cfg.tspan = [0, 0.35];
cfg.x0 = 0;
cfg.initial_command = 0;

cfg.components = struct( ...
    'name', {'truth', 'nav', 'guidance', 'control', 'latch'}, ...
    'kind', {'sensor', 'periodic', 'periodic', 'periodic', 'periodic'}, ...
    'period', {0.1, 0.1, 0.1, 0.1, 0.1}, ...
    'priority', {10, 20, 30, 40, 50}, ...
    'latches', {false, false, false, false, true}, ...
    'sample', {@truth_sample, [], [], [], []}, ...
    'step', {[], @nav_step, @guidance_step, @control_step, @latch_step});

res = aero_sim(model, cfg);

% order at t = 0.1
atT = find(abs([res.events.time] - 0.1) <= 1e-12);
check_that(numel(atT) == 6, 'expected 6 events at t=0.1, got %d', numel(atT));
seq = {res.events(atT).action};
expectedOrder = {'sample', 'publish', 'step', 'step', 'step', 'step'};
check_that(isequal(seq, expectedOrder), 'event order at t=0.1 is %s', strjoin(seq, ','));

compSeq = {res.events(atT).component};
check_that(isequal(compSeq, {'truth', 'truth', 'nav', 'guidance', 'control', 'latch'}), ...
    'component order at t=0.1 is %s', strjoin(compSeq, ','));

prio = [res.events(atT).priority];
check_that(all(diff(prio) >= 0), 'priorities are not monotone: %s', mat2str(prio));

% values propagate through the chain at every tick that has component events
% (the terminal boundary tf has no component update by design)
for g = 1:numel(res.time) - 1
    held = res.held{g};
    x = res.state(g, 1);
    check_that(abs(held - 2 * (x + 1)) <= 1e-9, ...
        't=%.3f: held=%.12g but 2*(x+1)=%.12g (same-instant propagation failed)', ...
        res.time(g), held, 2 * (x + 1));
end

% determinism
res2 = aero_sim(model, cfg);
check_that(isequaln(res.state, res2.state), 'state is not reproducible');
check_that(isequal(res.events, res2.events), 'event log is not reproducible');

info = struct('passed', true, 'summary', sprintf( ...
    'sample->publish->nav->guidance->control->latch at every 0.1 s tick; held=%.4f at t=0.3', ...
    res.held{end}));

end

function v = truth_sample(t, x) %#ok<INUSD>
v = x(1);
end

function bus = nav_step(t, bus) %#ok<INUSD>
m = aero_measurement(bus, 'truth');
if isempty(m)
    bus.nav = [];
else
    bus.nav = m.value;
end
end

function bus = guidance_step(t, bus) %#ok<INUSD>
if isfield(bus, 'nav') && ~isempty(bus.nav)
    bus.guidance = bus.nav + 1;
else
    bus.guidance = [];
end
end

function bus = control_step(t, bus) %#ok<INUSD>
if isfield(bus, 'guidance') && ~isempty(bus.guidance)
    bus.command = 2 * bus.guidance;
else
    bus.command = 0;
end
end

function bus = latch_step(t, bus) %#ok<INUSD>
bus.held_command = bus.command;
end
