function cfg = aero_validate_config(cfg)
%AERO_VALIDATE_CONFIG 校验并规范化仿真配置
%
% 所有错误使用 aero:config:* 标识符；非法输入直接失败，不做静默修补
% （不自动改正零周期、负容差、重复组件名等）。
%
% 规范化内容：
%   - x0 统一为列向量；
%   - 组件补齐默认 priority / offset / delay / latches，并校验唯一性；
%   - time_tolerance 为空时取 1e-12*max(1,|tf|)。

% ---- 时间区间 ---------------------------------------------------------
if ~isnumeric(cfg.tspan) || ~isequal(size(cfg.tspan), [1 2]) ...
        || ~isreal(cfg.tspan) || any(~isfinite(cfg.tspan)) ...
        || cfg.tspan(2) <= cfg.tspan(1)
    error('aero:config:tspan', 'tspan 必须是有限递增的 1×2 向量');
end

% ---- 初始状态 ---------------------------------------------------------
if ~isnumeric(cfg.x0) || isempty(cfg.x0) || ~isreal(cfg.x0) || any(~isfinite(cfg.x0(:)))
    error('aero:config:x0', 'x0 必须是非空、有限、实数的数值向量');
end
cfg.x0 = cfg.x0(:);

% ---- 初始命令（数值向量，或字段均为有限实数的结构体） -----------------
% 结构体命令用于同时承载互不耦合的多路指令（例如舵面数组与电机转速）。
% 内核只做通用校验与锁存，不解释字段含义，也不预设字段名。
aero_validate_command(cfg.initial_command, 'aero:config:command', 'initial_command');

% ---- 求解器 -----------------------------------------------------------
s = cfg.solver;
if ~isstruct(s) || ~isscalar(s)
    error('aero:config:solver', 'solver 必须是标量结构体');
end
require_positive_scalar(s.RelTol, 'aero:config:solver', 'solver.RelTol');
require_positive_scalar(s.AbsTol, 'aero:config:solver', 'solver.AbsTol');
if ~isnumeric(s.MaxStep) || ~isscalar(s.MaxStep) || ~isreal(s.MaxStep) ...
        || ~(s.MaxStep > 0)
    error('aero:config:solver', 'solver.MaxStep 必须是正标量或 Inf');
end
if ~isempty(s.InitialStep)
    require_positive_scalar(s.InitialStep, 'aero:config:solver', 'solver.InitialStep');
end

% ---- 种子、容差、结果目录 --------------------------------------------
if ~isnumeric(cfg.seed) || ~isscalar(cfg.seed) || ~isreal(cfg.seed) ...
        || ~isfinite(cfg.seed) || cfg.seed < 0 || cfg.seed ~= fix(cfg.seed)
    error('aero:config:seed', 'seed 必须是非负整数标量');
end

if isempty(cfg.time_tolerance)
    % 机器精度量级的合并容差：只用于判定“同一时刻”，不改变组件频率
    cfg.time_tolerance = 1e-12 * max(1, abs(cfg.tspan(2)));
else
    require_positive_scalar(cfg.time_tolerance, 'aero:config:tolerance', 'time_tolerance');
end

if ~ischar(cfg.results_root) && ~(isstring(cfg.results_root) && isscalar(cfg.results_root))
    error('aero:config:results', 'results_root 必须是字符向量或字符串');
end
cfg.results_root = char(cfg.results_root);

save = cfg.save;
if ~isstruct(save) || ~isscalar(save)
    error('aero:config:save', 'save 必须是标量结构体');
end
if ~islogical(save.enabled) && ~isnumeric(save.enabled)
    error('aero:config:save', 'save.enabled 必须是逻辑值');
end
save.enabled = logical(save.enabled);
if ~ischar(save.path) && ~(isstring(save.path) && isscalar(save.path))
    error('aero:config:save', 'save.path 必须是字符向量或字符串');
end
save.path = char(save.path);
if ~isnumeric(save.estimated_bytes) || ~isscalar(save.estimated_bytes) ...
        || ~isfinite(save.estimated_bytes) || save.estimated_bytes < 0
    error('aero:config:save', 'save.estimated_bytes 必须是非负标量');
end
if save.enabled && isempty(save.path)
    error('aero:config:save', 'save.enabled 为真时必须提供 save.path');
end
cfg.save = save;

% ---- 通信配置（V1.0 始终启用，不存在关闭模式） ------------------------
comm = cfg.communication;
if ~isstruct(comm) || ~isscalar(comm)
    error('aero:config:communication', 'communication 必须是标量结构体');
end
if isfield(comm, 'enabled')
    error('aero:config:communication', ...
        'communication.enabled 已移除：V1.0 始终使用唯一 Topic Bus');
end
if ~isfield(comm, 'topics')
    comm.topics = {};
end
if ~iscell(comm.topics)
    error('aero:config:communication', 'communication.topics 必须是 cell 数组');
end
comm.command_topic = normalize_topic_name(comm, 'command_topic');
comm.held_command_topic = normalize_topic_name(comm, 'held_command_topic');

logConfig = struct('enabled', false, 'topics', {{}}, 'max_records', 10000);
if isfield(comm, 'log') && ~isempty(comm.log)
    if ~isstruct(comm.log) || ~isscalar(comm.log)
        error('aero:config:communication', 'communication.log 必须是标量结构体');
    end
    logConfig = comm.log;
end
if ~isfield(logConfig, 'enabled') || isempty(logConfig.enabled)
    logConfig.enabled = false;
end
if ~isscalar(logConfig.enabled) ...
        || ~(islogical(logConfig.enabled) || isnumeric(logConfig.enabled))
    error('aero:config:communication', 'communication.log.enabled 必须是逻辑标量');
end
logConfig.enabled = logical(logConfig.enabled);
if ~isfield(logConfig, 'topics')
    logConfig.topics = {};
end
if ~iscell(logConfig.topics)
    error('aero:config:communication', 'communication.log.topics 必须是 cell 数组');
end
if ~isfield(logConfig, 'max_records') || isempty(logConfig.max_records)
    logConfig.max_records = 10000;
end
if ~isnumeric(logConfig.max_records) || ~isscalar(logConfig.max_records) ...
        || ~isfinite(logConfig.max_records) || logConfig.max_records < 1 ...
        || logConfig.max_records ~= fix(logConfig.max_records)
    error('aero:config:communication', 'communication.log.max_records 必须是正整数');
end
comm.log = logConfig;

cfg.communication = comm;

% ---- 组件 -------------------------------------------------------------
cfg.components = normalize_components(cfg.components);

end

% ===========================================================================
function components = normalize_components(input)
% 校验组件并补齐默认字段；保持声明顺序，因为同优先级事件按声明顺序执行
if isempty(input)
    components = aero_empty_component();
    return;
end
if ~isstruct(input)
    error('aero:config:component', 'components 必须是结构数组');
end

n = numel(input);
template = struct('name', '', 'kind', '', 'period', 0, 'offset', 0, ...
    'priority', 0, 'delay', 0, 'latches', false, 'topic', '', 'step', [], 'sample', []);
components = repmat(template, 1, n);

seenNames = cell(1, n);
for i = 1:n
    entry = input(i);

    if ~isfield(entry, 'name') || isempty(entry.name)
        error('aero:config:component', '第 %d 个组件缺少 name', i);
    end
    name = char(entry.name);
    if any(strcmp(seenNames(1:i - 1), name))
        error('aero:config:duplicate', '组件名重复："%s"', name);
    end
    seenNames{i} = name;

    if ~isfield(entry, 'kind') || isempty(entry.kind)
        error('aero:config:kind', '组件 "%s" 缺少 kind', name);
    end
    kind = lower(char(entry.kind));
    if ~any(strcmp(kind, {'sensor', 'periodic'}))
        error('aero:config:kind', '组件 "%s" 的 kind 必须是 sensor 或 periodic', name);
    end

    if ~isfield(entry, 'period')
        error('aero:config:period', '组件 "%s" 缺少 period', name);
    end
    require_positive_scalar(entry.period, 'aero:config:period', ...
        sprintf('组件 "%s" 的 period', name));

    offset = 0;
    if isfield(entry, 'offset') && ~isempty(entry.offset)
        offset = entry.offset;
    end
    if ~isnumeric(offset) || ~isscalar(offset) || ~isreal(offset) ...
            || ~isfinite(offset) || offset < 0 || offset >= entry.period
        error('aero:config:offset', ...
            '组件 "%s" 的 offset 必须满足 0 <= offset < period', name);
    end

    if isfield(entry, 'priority') && ~isempty(entry.priority)
        priority = entry.priority;
    elseif strcmp(kind, 'sensor')
        priority = 10;
    else
        priority = 30;
    end
    if ~isnumeric(priority) || ~isscalar(priority) || ~isreal(priority) ...
            || ~isfinite(priority)
        error('aero:config:priority', '组件 "%s" 的 priority 必须是有限标量', name);
    end

    delay = 0;
    if isfield(entry, 'delay') && ~isempty(entry.delay)
        delay = entry.delay;
    end
    if ~isnumeric(delay) || ~isscalar(delay) || ~isreal(delay) ...
            || ~isfinite(delay) || delay < 0
        error('aero:config:delay', '组件 "%s" 的 delay 必须是非负标量', name);
    end
    if ~strcmp(kind, 'sensor') && delay ~= 0
        error('aero:config:delay', ...
            '组件 "%s" 是 periodic，只有 sensor 可以声明 delay', name);
    end

    latches = false;
    if isfield(entry, 'latches') && ~isempty(entry.latches)
        latches = logical(entry.latches);
    end
    if strcmp(kind, 'sensor') && latches
        error('aero:config:latches', ...
            '组件 "%s" 是 sensor，不能锁存控制输入', name);
    end

    topic = '';
    if isfield(entry, 'topic') && ~isempty(entry.topic)
        topic = char(entry.topic);
    end
    if strcmp(kind, 'sensor')
        if isempty(topic)
            error('aero:config:topic', ...
                '传感器 "%s" 必须声明 topic：V1.0 的测量统一经 Topic Bus 传递', name);
        end
    elseif ~isempty(topic)
        error('aero:config:topic', ...
            '组件 "%s" 是 periodic，只有 sensor 可以声明 topic（命令 Topic 由通信配置统一发布）', name);
    end

    step = [];
    sample = [];
    if strcmp(kind, 'sensor')
        if ~isfield(entry, 'sample') || ~isa(entry.sample, 'function_handle')
            error('aero:config:callback', 'sensor "%s" 必须提供 sample 函数句柄', name);
        end
        sample = entry.sample;
    else
        if ~isfield(entry, 'step') || ~isa(entry.step, 'function_handle')
            error('aero:config:callback', 'periodic 组件 "%s" 必须提供 step 函数句柄', name);
        end
        step = entry.step;
    end

    components(i).name = name;
    components(i).kind = kind;
    components(i).period = double(entry.period);
    components(i).offset = double(offset);
    components(i).priority = double(priority);
    components(i).delay = double(delay);
    components(i).latches = latches;
    components(i).topic = topic;
    components(i).step = step;
    components(i).sample = sample;
end
end

% ===========================================================================
function value = normalize_topic_name(comm, field)
value = '';
if isfield(comm, field) && ~isempty(comm.(field))
    raw = comm.(field);
    if ~(ischar(raw) || (isstring(raw) && isscalar(raw)))
        error('aero:config:communication', 'communication.%s 必须是字符或字符串', field);
    end
    value = char(raw);
end
end

% ===========================================================================
function require_positive_scalar(value, identifier, label)
if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) ...
        || ~isfinite(value) || value <= 0
    error(identifier, '%s 必须是正有限标量', label);
end
end
