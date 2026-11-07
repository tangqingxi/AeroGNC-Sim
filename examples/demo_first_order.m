function demo_first_order()
%DEMO_FIRST_ORDER Minimal end-to-end example of the Simulation-Kernel.
%
%   Run from the repository root:
%       matlab -batch "run('examples/demo_first_order.m')"
%
%   The model is an analytic first-order plant (a test double, not an
%   aircraft). A 2 Hz "command" component latches u = 1 at t = 0.5 s; the
%   kernel integrates piecewise with ode45 and records states, events and
%   commands.

root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root, 'src'));
addpath(fullfile(root, 'models'));

model = first_order_model(2, 3);

cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.seed = 0;
cfg.verbose = true;
cfg.components = struct('name', 'command', 'kind', 'periodic', 'period', 0.5, ...
    'priority', 50, 'latches', true, 'step', @command_step);

result = aero_sim(model, cfg);

fprintf('\nmodel            : %s\n', result.model);
fprintf('events           : %d\n', result.stats.events);
fprintf('segments         : %d\n', result.stats.segments);
fprintf('internal points  : %d (ode45 chooses them, they are not samples)\n', ...
    result.stats.total_internal_points);
fprintf('MATLAB           : %s (%s)\n', result.environment.matlab_version, ...
    result.environment.matlab_release);
fprintf('git commit       : %s (dirty=%d)\n', result.environment.git_commit, ...
    result.environment.git_dirty);

fprintf('\n     t [s]      x(t)      held u\n');
for k = 1:numel(result.time)
    fprintf('%10.3f  %9.5f  %7.3f\n', result.time(k), result.state(k, 1), result.held{k});
end

analytic = 1.5 * (1 - exp(-2 * (2 - 0.5)));
fprintf('\nanalytic x(2) = %.6f, simulated = %.6f, error = %.2e\n', ...
    analytic, result.state(end, 1), abs(analytic - result.state(end, 1)));

end

function bus = command_step(t, bus)
if t >= 0.5 - 1e-12
    bus.held_command = 1;
else
    bus.held_command = 0;
end
end
