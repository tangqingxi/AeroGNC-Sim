function cfg = aero_validate_config(cfg)
%AERO_VALIDATE_CONFIG Validate and normalise a simulation configuration.
%
%   Errors use identifiers under the aero:config: namespace and never repair
%   invalid input silently.

% ---- time span ------------------------------------------------------------
if ~isnumeric(cfg.tspan) || ~isequal(size(cfg.tspan), [1 2]) ...
        || ~isreal(cfg.tspan) || any(~isfinite(cfg.tspan)) ...
        || cfg.tspan(2) <= cfg.tspan(1)
    error('aero:config:tspan', 'tspan must be a finite increasing 1x2 vector');
end

% ---- initial state --------------------------------------------------------
if ~isnumeric(cfg.x0) || isempty(cfg.x0) || ~isreal(cfg.x0) || any(~isfinite(cfg.x0(:)))
    error('aero:config:x0', 'x0 must be a non-empty finite real numeric vector');
end
cfg.x0 = cfg.x0(:);

% ---- initial command ------------------------------------------------------
if ~isnumeric(cfg.initial_command) || ~isreal(cfg.initial_command) ...
        || any(~isfinite(cfg.initial_command(:)))
    error('aero:config:command', 'initial_command must be finite real numeric');
end

% ---- solver ---------------------------------------------------------------
s = cfg.solver;
if ~isstruct(s) || ~isscalar(s)
    error('aero:config:solver', 'solver must be a scalar struct');
end
require_positive_scalar(s.RelTol, 'aero:config:solver', 'solver.RelTol');
require_positive_scalar(s.AbsTol, 'aero:config:solver', 'solver.AbsTol');
if ~isnumeric(s.MaxStep) || ~isscalar(s.MaxStep) || ~isreal(s.MaxStep) ...
        || ~(s.MaxStep > 0)
    error('aero:config:solver', 'solver.MaxStep must be a positive scalar or Inf');
end
if ~isempty(s.InitialStep)
    require_positive_scalar(s.InitialStep, 'aero:config:solver', 'solver.InitialStep');
end

% ---- seed / tolerance / results -------------------------------------------
if ~isnumeric(cfg.seed) || ~isscalar(cfg.seed) || ~isreal(cfg.seed) ...
        || ~isfinite(cfg.seed) || cfg.seed < 0 || cfg.seed ~= fix(cfg.seed)
    error('aero:config:seed', 'seed must be a non-negative integer scalar');
end

if isempty(cfg.time_tolerance)
    cfg.time_tolerance = 1e-12 * max(1, abs(cfg.tspan(2)));
else
    require_positive_scalar(cfg.time_tolerance, 'aero:config:tolerance', 'time_tolerance');
end

if ~ischar(cfg.results_root) && ~(isstring(cfg.results_root) && isscalar(cfg.results_root))
    error('aero:config:results', 'results_root must be a character row or string');
end
cfg.results_root = char(cfg.results_root);

save = cfg.save;
if ~isstruct(save) || ~isscalar(save)
    error('aero:config:save', 'save must be a scalar struct');
end
if ~islogical(save.enabled) && ~isnumeric(save.enabled)
    error('aero:config:save', 'save.enabled must be logical');
end
save.enabled = logical(save.enabled);
if ~ischar(save.path) && ~(isstring(save.path) && isscalar(save.path))
    error('aero:config:save', 'save.path must be a character row or string');
end
save.path = char(save.path);
if ~isnumeric(save.estimated_bytes) || ~isscalar(save.estimated_bytes) ...
        || ~isfinite(save.estimated_bytes) || save.estimated_bytes < 0
    error('aero:config:save', 'save.estimated_bytes must be a non-negative scalar');
end
if save.enabled && isempty(save.path)
    error('aero:config:save', 'save.enabled requires save.path');
end
cfg.save = save;

% ---- components -----------------------------------------------------------
cfg.components = normalize_components(cfg.components);

end

% ===========================================================================
function c = normalize_components(in)
if isempty(in)
    c = aero_empty_component();
    return;
end
if ~isstruct(in)
    error('aero:config:component', 'components must be a struct array');
end

n = numel(in);
template = struct('name', '', 'kind', '', 'period', 0, 'offset', 0, ...
    'priority', 0, 'delay', 0, 'latches', false, 'step', [], 'sample', []);
c = repmat(template, 1, n);

seen = cell(1, n);
for i = 1:n
    e = in(i);

    if ~isfield(e, 'name') || isempty(e.name)
        error('aero:config:component', 'component %d has no name', i);
    end
    name = char(e.name);
    if any(strcmp(seen(1:i-1), name))
        error('aero:config:duplicate', 'duplicate component name "%s"', name);
    end
    seen{i} = name;

    if ~isfield(e, 'kind') || isempty(e.kind)
        error('aero:config:kind', 'component "%s" has no kind', name);
    end
    kind = lower(char(e.kind));
    if ~any(strcmp(kind, {'sensor', 'periodic'}))
        error('aero:config:kind', 'component "%s" kind must be sensor or periodic', name);
    end

    if ~isfield(e, 'period')
        error('aero:config:period', 'component "%s" has no period', name);
    end
    require_positive_scalar(e.period, 'aero:config:period', ...
        sprintf('component "%s" period', name));

    offset = 0;
    if isfield(e, 'offset') && ~isempty(e.offset)
        offset = e.offset;
    end
    if ~isnumeric(offset) || ~isscalar(offset) || ~isreal(offset) ...
            || ~isfinite(offset) || offset < 0 || offset >= e.period
        error('aero:config:offset', ...
            'component "%s" offset must satisfy 0 <= offset < period', name);
    end

    if isfield(e, 'priority') && ~isempty(e.priority)
        priority = e.priority;
    elseif strcmp(kind, 'sensor')
        priority = 10;
    else
        priority = 30;
    end
    if ~isnumeric(priority) || ~isscalar(priority) || ~isreal(priority) ...
            || ~isfinite(priority)
        error('aero:config:priority', 'component "%s" priority must be a finite scalar', name);
    end

    delay = 0;
    if isfield(e, 'delay') && ~isempty(e.delay)
        delay = e.delay;
    end
    if ~isnumeric(delay) || ~isscalar(delay) || ~isreal(delay) ...
            || ~isfinite(delay) || delay < 0
        error('aero:config:delay', 'component "%s" delay must be a non-negative scalar', name);
    end
    if ~strcmp(kind, 'sensor') && delay ~= 0
        error('aero:config:delay', 'component "%s" is periodic; only sensors may declare delay', name);
    end

    latches = false;
    if isfield(e, 'latches') && ~isempty(e.latches)
        latches = logical(e.latches);
    end
    if strcmp(kind, 'sensor') && latches
        error('aero:config:latches', 'component "%s" is a sensor; sensors cannot latch commands', name);
    end

    step = [];
    sample = [];
    if strcmp(kind, 'sensor')
        if ~isfield(e, 'sample') || ~isa(e.sample, 'function_handle')
            error('aero:config:callback', 'sensor "%s" requires a sample function handle', name);
        end
        sample = e.sample;
    else
        if ~isfield(e, 'step') || ~isa(e.step, 'function_handle')
            error('aero:config:callback', 'periodic component "%s" requires a step function handle', name);
        end
        step = e.step;
    end

    c(i).name = name;
    c(i).kind = kind;
    c(i).period = double(e.period);
    c(i).offset = double(offset);
    c(i).priority = double(priority);
    c(i).delay = double(delay);
    c(i).latches = latches;
    c(i).step = step;
    c(i).sample = sample;
end
end

% ===========================================================================
function require_positive_scalar(value, id, label)
if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) ...
        || ~isfinite(value) || value <= 0
    error(id, '%s must be a positive finite scalar', label);
end
end
