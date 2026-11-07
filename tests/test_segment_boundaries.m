function info = test_segment_boundaries()
%TEST_SEGMENT_BOUNDARIES Integration runs exactly between event boundaries.

model = first_order_model(1, 1);
cfg = aero_default_config();
cfg.tspan = [0, 1];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.components = struct( ...
    'name', {'a', 'b', 'c'}, ...
    'kind', 'periodic', ...
    'period', {0.03, 0.017, 0.01}, ...
    'priority', 30, ...
    'latches', false, ...
    'step', @noop);

res = aero_sim(model, cfg);

check_that(res.time(1) == 0, 'first logged time %.17g != 0', res.time(1));
check_that(res.time(end) == 1, 'last logged time %.17g != 1', res.time(end));
check_that(all(diff(res.time) > 0), 'logged times are not strictly increasing');

nSeg = numel(res.segments);
check_that(nSeg == numel(res.time) - 1, '%d segments for %d times', nSeg, numel(res.time));
for k = 2:nSeg
    check_that(res.segments{k}.t0 == res.segments{k - 1}.t1, ...
        'segment %d does not start at the previous end (%.17g vs %.17g)', ...
        k, res.segments{k}.t0, res.segments{k - 1}.t1);
end

evTimes = [res.events.time];
check_that(all(ismembertol(evTimes, res.time, 1e-12)), ...
    'some events do not coincide with a logged boundary');

for k = 1:numel(res.segments)
    s = res.segments{k};
    inside = evTimes > s.t0 + 1e-15 & evTimes < s.t1 - 1e-15;
    check_that(~any(inside), 'segment %d contains an event boundary inside it', k);
end

info = struct('passed', true, 'summary', sprintf( ...
    '%d segments, all events on boundaries, chain continuous', nSeg));

end

function bus = noop(t, bus) %#ok<INUSD>
end
