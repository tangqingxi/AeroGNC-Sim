function info = test_command_latch()
%TEST_COMMAND_LATCH A 100 Hz command reaches the dynamics only at 40 Hz latches.

model = scalar_integrator_model();
cfg = aero_default_config();
cfg.tspan = [0, 0.1];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.components = struct( ...
    'name', {'control', 'latch'}, ...
    'kind', {'periodic', 'periodic'}, ...
    'period', {0.01, 0.025}, ...
    'priority', {40, 50}, ...
    'latches', {false, true}, ...
    'step', {@control_step, @latch_step});

res = aero_sim(model, cfg);

ctrlTimes = event_times(res, 'control');
latchTimes = event_times(res, 'latch');
check_that(numel(ctrlTimes) == 10, 'control ran %d times, expected 10', numel(ctrlTimes));
check_that(numel(latchTimes) == 4, 'latch ran %d times, expected 4', numel(latchTimes));

prevHeld = cfg.initial_command;
changes = 0;
for g = 1:numel(res.time)
    t = res.time(g);
    held = res.held{g};
    isLatch = any(abs(latchTimes - t) <= 1e-12);
    if isLatch
        expected = max(ctrlTimes(ctrlTimes <= t + 1e-12));
        check_that(abs(held - expected) <= 1e-12, ...
            'latch at t=%.6f took %.12g, latest command is %.12g', t, held, expected);
    else
        check_that(held == prevHeld, ...
            'held command changed at t=%.6f, which is not a latch time', t);
    end
    if held ~= prevHeld
        changes = changes + 1;
    end
    prevHeld = held;
end
check_that(changes == 3, 'held command changed %d times, expected 3', changes);

% the last latch (0.075) takes the command computed at 0.07
check_that(abs(res.held{end - 1} - 0.07) <= 1e-12, ...
    'held at 0.075 is %.17g, expected 0.07', res.held{end - 1});

info = struct('passed', true, 'summary', sprintf( ...
    '%d control steps, %d latches, %d held-command changes, last latch takes t=0.07', ...
    numel(ctrlTimes), numel(latchTimes), changes));

end

function bus = control_step(t, bus)
bus.command = t;
end

function bus = latch_step(t, bus) %#ok<INUSD>
bus.held_command = bus.command;
end
