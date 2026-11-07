function info = test_multirate()
%TEST_MULTIRATE Declared periods are honoured without grid rounding drift.

model = first_order_model(1, 1);
cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.solver = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
cfg.components = struct( ...
    'name', {'fast800', 'mid200', 'slow125'}, ...
    'kind', 'periodic', ...
    'period', {0.00125, 0.005, 0.008}, ...
    'priority', 30, ...
    'latches', false, ...
    'step', @noop);

res = aero_sim(model, cfg);

counts = zeros(1, 3);
drift = zeros(1, 3);
names = {'fast800', 'mid200', 'slow125'};
periods = [0.00125, 0.005, 0.008];
expectedCounts = [800, 200, 125];
for k = 1:3
    t = event_times(res, names{k});
    counts(k) = numel(t);
    expectedT = (0:numel(t) - 1).' * periods(k);
    drift(k) = max(abs(t(:) - expectedT));
end
check_that(all(counts == expectedCounts), ...
    'event counts [%s] != expected [%s]', num2str(counts), num2str(expectedCounts));
check_that(max(drift) <= 1e-9, 'multirate drift %.3e exceeds 1e-9', max(drift));

% non-integer ratio: 73 Hz and 127 Hz over 5 s
cfg2 = cfg;
cfg2.tspan = [0, 5];
cfg2.components = struct( ...
    'name', {'f73', 'f127'}, ...
    'kind', 'periodic', ...
    'period', {1 / 73, 1 / 127}, ...
    'priority', 30, ...
    'latches', false, ...
    'step', @noop);
res2 = aero_sim(model, cfg2);

t73 = event_times(res2, 'f73');
t127 = event_times(res2, 'f127');
d73 = max(abs(t73(:) - (0:numel(t73) - 1).' / 73));
d127 = max(abs(t127(:) - (0:numel(t127) - 1).' / 127));
check_that(numel(t73) == 365, '73 Hz produced %d events over 5 s, expected 365', numel(t73));
check_that(numel(t127) == 635, '127 Hz produced %d events over 5 s, expected 635', numel(t127));
check_that(max(d73, d127) <= 1e-9, 'non-integer drift %.3e exceeds 1e-9', max(d73, d127));

info = struct('passed', true, 'summary', sprintf( ...
    '800/200/125 Hz counts %d/%d/%d drift=%.1e; 73/127 Hz drift=%.1e', ...
    counts(1), counts(2), counts(3), max(drift), max(d73, d127)));

end

function bus = noop(t, bus) %#ok<INUSD>
end
