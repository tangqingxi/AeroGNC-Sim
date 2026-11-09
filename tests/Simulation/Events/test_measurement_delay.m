function info = test_measurement_delay()
%TEST_MEASUREMENT_DELAY A delayed measurement is never visible before it exists.

model = scalar_integrator_model();
delay = 0.05;

cfg = aero_default_config();
cfg.tspan = [0, 0.2];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.communication.topics = {struct('name', 'sensor/truth', ...
    'spec', aero_topic_spec({'value', 'sample_time_s'}, {[1 1], [1 1]}, {'-', 's'}))};
cfg.components = struct( ...
    'name', {'truth', 'control'}, ...
    'kind', {'sensor', 'periodic'}, ...
    'period', {0.02, 0.01}, ...
    'priority', {10, 30}, ...
    'delay', {delay, 0}, ...
    'latches', {false, false}, ...
    'topic', {'sensor/truth', ''}, ...
    'sample', {@truth_sample, []}, ...
    'step', {[], @control_step});

res = aero_sim(model, cfg);

ms = [res.measurements{:}];
check_that(~isempty(ms), 'no measurement was published');
for k = 1:numel(ms)
    check_that(abs(ms(k).available_time - (ms(k).sample_time + delay)) <= 1e-12, ...
        'measurement %d: available_time %.17g != sample_time %.17g + %.17g', ...
        k, ms(k).available_time, ms(k).sample_time, delay);
    check_that(ms(k).available_time >= ms(k).sample_time, ...
        'measurement %d becomes available before it is sampled', k);
end

avail = [ms.available_time];
samp = [ms.sample_time];
for g = 1:numel(res.time)
    t = res.time(g);
    c = res.computed{g};
    idx = avail <= t + 1e-12;
    if any(idx)
        expected = max(samp(idx));
    else
        expected = -1;
    end
    check_that(abs(c - expected) <= 1e-9, ...
        't=%.3f: control used %.12g but latest available sample is %.12g', ...
        t, c, expected);
end

check_that(min(avail) >= delay - 1e-12, ...
    'first measurement became available at %.17g, before one delay (%.17g)', min(avail), delay);

info = struct('passed', true, 'summary', sprintf( ...
    'delay=%.3f s, %d measurements, control never used a future sample', ...
    delay, numel(ms)));

end

function v = truth_sample(t, x) %#ok<INUSD>
v = x(1);
end

function bus = control_step(t, bus)
[msg, status] = bus.topics.tryRead('sensor/truth', t);
if strcmp(status, 'ok')
    bus.command = msg.data.sample_time_s;
else
    bus.command = -1;
end
end
