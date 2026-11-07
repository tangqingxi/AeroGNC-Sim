function result = aero_sim(model, cfg)
%AERO_SIM Run a fixed-step-event / piecewise-ode45 simulation.
%
%   result = aero_sim(model, cfg) integrates the model between consecutive
%   event boundaries. The latched input bus.held_command is constant inside a
%   segment and may only be changed by components declared with latches=true.
%
%   model: struct with fields
%       name       short identifier recorded in the result
%       x0         optional initial state (used when cfg.x0 is empty)
%       dynamics   @(t, x, u, env) returning dx/dt (no side effects)
%
%   cfg: see aero_default_config. Truth (the continuous state) reaches
%   components only through sensor sample callbacks; periodic step callbacks
%   never receive it. Ground truth is recorded by the kernel for verification.
%
%   See also aero_default_config, aero_schedule, aero_save_result.

if nargin < 2
    cfg = struct();
end

cfg = aero_fill_defaults(cfg, aero_default_config());
model = validate_model(model);
if isempty(cfg.x0) && isfield(model, 'x0')
    cfg.x0 = model.x0;
end
cfg = aero_validate_config(cfg);

sched = aero_schedule(cfg);
G = numel(sched.times);
E = numel(sched.event_time);

nX = numel(cfg.x0);
stateLog = zeros(nX, G);
heldLog = cell(G, 1);
cmdLog = cell(G, 1);
evCompName = cell(E, 1);
evActName = cell(E, 1);

maxSid = 0;
if E > 0
    maxSid = max(sched.event_sample_id);
end
pending = cell(maxSid, 1);
pubCount = nnz(sched.event_action == 2);
measLog = cell(pubCount, 1);
mIdx = 0;

bus = struct();
bus.measurements = {};
bus.command = cfg.initial_command;
bus.held_command = cfg.initial_command;
bus.truth = [];

x = cfg.x0;
t = cfg.tspan(1);
segs = cell(max(G - 1, 0), 1);
totalInternal = 0;
latchCount = 0;
timer = tic;

for g = 1:G
    tg = sched.times(g);
    if g > 1
        [x, info] = aero_integrate(model, t, tg, x, bus.held_command, ...
            cfg.environment, cfg.solver);
        segs{g - 1} = struct('t0', t, 't1', tg, 'held', bus.held_command, ...
            'internal_points', info.internal_points);
        totalInternal = totalInternal + info.internal_points;
    end
    t = tg;

    for k = sched.group_start(g):sched.group_end(g)
        ci = sched.event_component(k);
        c = cfg.components(ci);
        action = sched.event_action(k);

        if action == 1
            value = c.sample(t, x);
            if ~isnumeric(value) || ~isreal(value) || isempty(value) ...
                    || any(~isfinite(value(:)))
                error('aero:kernel:sample', ...
                    'sensor "%s" returned a non-finite or non-numeric value at t=%.17g', ...
                    c.name, t);
            end
            pending{sched.event_sample_id(k)} = struct( ...
                'name', c.name, 'sample_time', sched.event_time(k), ...
                'available_time', t, 'value', double(value));
            evCompName{k} = c.name;
            evActName{k} = 'sample';

        elseif action == 2
            s = sched.event_sample_id(k);
            m = pending{s};
            if isempty(m)
                error('aero:kernel:publish', ...
                    'publish event for "%s" at t=%.17g has no pending sample', c.name, t);
            end
            pending{s} = [];
            m.available_time = t;
            mIdx = mIdx + 1;
            measLog{mIdx} = m;
            bus.measurements{end + 1} = m; %#ok<AGROW>
            evCompName{k} = c.name;
            evActName{k} = 'publish';

        else
            prevHeld = bus.held_command;
            bus = c.step(t, bus);
            if ~isstruct(bus) || ~isscalar(bus)
                error('aero:kernel:bus', ...
                    'component "%s" must return a scalar bus struct', c.name);
            end
            if ~isfield(bus, 'command') || ~isfield(bus, 'held_command')
                error('aero:kernel:bus', ...
                    'component "%s" removed bus.command or bus.held_command', c.name);
            end
            if ~isequaln(bus.held_command, prevHeld)
                if ~c.latches
                    error('aero:kernel:latch', ...
                        'component "%s" changed held_command at t=%.17g without latches=true', ...
                        c.name, t);
                end
                latchCount = latchCount + 1;
            end
            evCompName{k} = c.name;
            evActName{k} = 'step';
        end
    end

    stateLog(:, g) = x;
    heldLog{g} = bus.held_command;
    cmdLog{g} = bus.command;
end

wall = toc(timer);

evExecTime = zeros(E, 1);
if E > 0
    evExecTime = sched.times(sched.group);
end

result = struct();
result.schema = 'aero-sim-result/1';
result.model = model.name;
result.time = sched.times(:);
result.state = stateLog.';
result.events = struct( ...
    'time', num2cell(evExecTime), ...
    'component', evCompName, ...
    'action', evActName, ...
    'priority', num2cell(sched.event_priority));
result.measurements = measLog(1:mIdx);
result.held = heldLog;
result.computed = cmdLog;
result.segments = segs(~cellfun(@isempty, segs));
result.solver = cfg.solver;
result.config = cfg;
result.environment = aero_env_info();
result.seed = cfg.seed;
result.stats = struct( ...
    'groups', G, ...
    'events', E, ...
    'segments', numel(result.segments), ...
    'latch_events', latchCount, ...
    'total_internal_points', totalInternal, ...
    'wall_time_s', wall);

if cfg.save.enabled
    [savedPath, check] = aero_save_result(result, cfg.save, cfg.results_root);
    result.saved_path = savedPath;
    result.results_check = check;
end

if cfg.verbose
    fprintf('aero_sim: model=%s groups=%d events=%d segments=%d internal_points=%d wall=%.3fs\n', ...
        model.name, G, E, numel(result.segments), totalInternal, wall);
end

end

% ===========================================================================
function model = validate_model(model)
if ~isstruct(model) || ~isscalar(model)
    error('aero:config:model', 'model must be a scalar struct');
end
if ~isfield(model, 'dynamics') || ~isa(model.dynamics, 'function_handle')
    error('aero:config:model', ...
        'model.dynamics must be a function handle f(t, x, u, env)');
end
if ~isfield(model, 'name') || isempty(model.name)
    model.name = 'unnamed';
end
model.name = char(model.name);
end
