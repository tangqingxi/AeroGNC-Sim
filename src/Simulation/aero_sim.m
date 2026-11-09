function result = aero_sim(model, cfg)
%AERO_SIM 运行事件驱动的分段 ode45 仿真
%
% 输入：
%   model - 被控对象/机型描述结构：
%       name     短名称（写入结果）
%       x0       可选初态；cfg.x0 为空时使用
%       dynamics 函数句柄 @(t,x,u,env) -> dx，必须无副作用
%   cfg   - 配置结构（aero_default_config），缺省字段自动补齐
%
% 输出：
%   result - SimulationResult，字段见 README“结果结构”一节
%
% 时间语义：
%   在相邻事件边界之间调用 ode45 积分，段内锁存输入 held_command 保持不变
%   （零阶保持）；ode45 的内部步点只用于积分，不作为采样点或日志时刻。
%
% 真值隔离：
%   连续状态只通过 sensor 组件的 sample(t,x) 读取；periodic 组件永远拿不到
%   真值（bus.truth 恒为空）。内核记录真值仅用于验证与日志。
%
% 通信（可选，默认关闭）：
%   cfg.communication.enabled=true 时创建唯一 aero_topic_bus 并作为 bus.topics
%   提供给组件；sensor 在测量可用事件发布 Topic，计算结果与锁存结果分别发布到
%   command_topic / held_command_topic。关闭时行为与旧版完全一致。
%
% 参见 aero_default_config, aero_schedule, aero_save_result。

if nargin < 2
    cfg = struct();
end

% ---- 1. 配置与模型校验 ------------------------------------------------
cfg = aero_fill_defaults(cfg, aero_default_config());
model = validate_model(model);
if isempty(cfg.x0) && isfield(model, 'x0')
    cfg.x0 = model.x0;
end
cfg = aero_validate_config(cfg);

% ---- 2. 事件调度表 ----------------------------------------------------
sched = aero_schedule(cfg);
nGroups = numel(sched.times);
nEvents = numel(sched.event_time);
actions = sched.event_action;

% ---- 3. 日志与状态预分配 ----------------------------------------------
nStates = numel(cfg.x0);
stateLog = zeros(nStates, nGroups);      % 每列是一个事件边界时刻的状态
heldLog = cell(nGroups, 1);              % 边界后的锁存输入
commandLog = cell(nGroups, 1);           % 边界后的最新计算命令
eventComponent = cell(nEvents, 1);       % 事件组件名（便于阅读）
eventAction = cell(nEvents, 1);          % 事件动作名

maxSampleId = 0;
if nEvents > 0
    maxSampleId = max(sched.event_sample_id);
end
pending = cell(maxSampleId, 1);          % 已采样但尚未发布的测量
nPublishes = nnz(strcmp(actions, 'publish'));
measurementLog = cell(nPublishes, 1);

% 总线：内核保证以下字段存在，其余字段由组件自行读写
bus = struct();
bus.measurements = {};                   % 已可用测量（按发布时间排序）
bus.command = cfg.initial_command;       % 控制计算值
bus.held_command = cfg.initial_command;  % 真正进入动力学的锁存值
bus.truth = [];                          % 恒为空：周期组件不得读取真值

% ---- 3b. 通信总线（唯一实例，默认关闭） -------------------------------
comm = aero_communication_setup(cfg.communication);
bus.topics = comm.bus;
lastCommand = bus.command;
lastHeld = bus.held_command;
if comm.enabled
    sensorTopics = cell(1, 0);
    for cIdx = 1:numel(cfg.components)
        topicName = cfg.components(cIdx).topic;
        if isempty(topicName)
            continue;
        end
        if ~comm.bus.isRegistered(topicName)
            error('aero:comm:unknownMeasurementTopic', ...
                '传感器 "%s" 的 topic "%s" 未注册', cfg.components(cIdx).name, topicName);
        end
        if any(strcmp(sensorTopics, topicName))
            error('aero:comm:topicConflict', ...
                '多个传感器声明了同一 Topic "%s"', topicName);
        end
        if strcmp(topicName, comm.command_topic) || strcmp(topicName, comm.held_command_topic)
            error('aero:comm:topicConflict', ...
                '传感器 Topic "%s" 与命令 Topic 冲突', topicName);
        end
        sensorTopics{end + 1} = topicName; %#ok<AGROW>
    end
    t0 = cfg.tspan(1);
    if ~isempty(comm.command_topic)
        comm.bus.publish(comm.command_topic, aero_command_payload(bus.command), ...
            t0, t0, t0, 'simulation');
    end
    if ~isempty(comm.held_command_topic)
        comm.bus.publish(comm.held_command_topic, aero_command_payload(bus.held_command), ...
            t0, t0, t0, 'simulation');
    end
end

x = cfg.x0;
if isfield(model, 'project') && ~isempty(model.project)
    x = model.project(x);
end
t = cfg.tspan(1);
segments = cell(max(nGroups - 1, 0), 1);
totalInternalPoints = 0;
latchCount = 0;
measurementCount = 0;
timer = tic;

% ---- 4. 主循环：积分到边界 -> 执行同刻事件 -> 记录 --------------------
for g = 1:nGroups
    boundary = sched.times(g);

    if g > 1
        [x, integratorInfo] = aero_integrate(model, t, boundary, x, ...
            bus.held_command, cfg.environment, cfg.solver);
        % 机型可声明状态投影（例如四元数归一化）；内核不解释其物理含义
        if isfield(model, 'project') && ~isempty(model.project)
            x = model.project(x);
        end
        segments{g - 1} = struct( ...
            't0', t, ...
            't1', boundary, ...
            'held', bus.held_command, ...
            'internal_points', integratorInfo.internal_points);
        totalInternalPoints = totalInternalPoints + integratorInfo.internal_points;
    end
    t = boundary;

    for k = sched.group_start(g):sched.group_end(g)
        component = cfg.components(sched.event_component(k));
        action = actions{k};

        if strcmp(action, 'sample')
            % 传感器是唯一被授权读取真值的接口
            value = component.sample(t, x);
            if ~isnumeric(value) || ~isreal(value) || isempty(value) ...
                    || any(~isfinite(value(:)))
                error('aero:kernel:sample', ...
                    '传感器 "%s" 在 t=%.17g 返回了非有限或非数值的测量', component.name, t);
            end
            pending{sched.event_sample_id(k)} = struct( ...
                'name', component.name, ...
                'sample_time', sched.event_time(k), ...
                'available_time', t, ...
                'value', double(value));
            eventComponent{k} = component.name;
            eventAction{k} = 'sample';

        elseif strcmp(action, 'publish')
            % 测量在此刻才对所有组件可见（因果可用时刻）
            sampleId = sched.event_sample_id(k);
            measurement = pending{sampleId};
            if isempty(measurement)
                error('aero:kernel:publish', ...
                    '"%s" 在 t=%.17g 的发布事件找不到对应采样', component.name, t);
            end
            pending{sampleId} = [];
            measurement.available_time = t;
            measurementCount = measurementCount + 1;
            measurementLog{measurementCount} = measurement;
            bus.measurements{end + 1} = measurement; %#ok<AGROW>
            % 只有到了实际可用时刻才发布到 Topic Bus：
            % sample_time 保留采样时刻，available_time 等于当前事件时刻。
            if comm.enabled && ~isempty(component.topic)
                payload = struct('value', measurement.value(:), ...
                    'sample_time_s', measurement.sample_time);
                comm.bus.publish(component.topic, payload, ...
                    measurement.sample_time, t, t, component.name);
            end
            eventComponent{k} = component.name;
            eventAction{k} = 'publish';

        else
            % 周期组件更新：只允许声明 latches=true 的组件修改锁存输入
            previousHeld = bus.held_command;
            bus = component.step(t, bus);
            if ~isstruct(bus) || ~isscalar(bus)
                error('aero:kernel:bus', '组件 "%s" 必须返回标量 bus 结构', component.name);
            end
            if ~isfield(bus, 'command') || ~isfield(bus, 'held_command')
                error('aero:kernel:bus', ...
                    '组件 "%s" 删除了 bus.command 或 bus.held_command', component.name);
            end
            if ~isequaln(bus.held_command, previousHeld)
                if ~component.latches
                    error('aero:kernel:latch', ...
                        '组件 "%s" 在 t=%.17g 修改锁存输入但未声明 latches=true', ...
                        component.name, t);
                end
                latchCount = latchCount + 1;
            end
            % 唯一通信实例检查：组件不得删除或替换 bus.topics
            if comm.enabled
                if ~isfield(bus, 'topics') || ~isa(bus.topics, 'aero_topic_bus') ...
                        || bus.topics ~= comm.bus
                    error('aero:comm:busReplaced', ...
                        '组件 "%s" 在 t=%.17g 删除或替换了唯一通信总线', component.name, t);
                end
            end
            % 组件更新后立即检查命令值：任意字段名均可，但必须是有限数值结构
            aero_validate_command(bus.command, 'aero:kernel:command', ...
                sprintf('组件 "%s" 在 t=%.17g 写入的 bus.command', component.name, t));
            aero_validate_command(bus.held_command, 'aero:kernel:command', ...
                sprintf('组件 "%s" 在 t=%.17g 写入的 bus.held_command', component.name, t));
            eventComponent{k} = component.name;
            eventAction{k} = 'step';
        end

        % 计算命令与锁存命令分别映射到独立 Topic；只发布“发生变化”的值，
        % 不改变原有 command / held_command 的锁存语义。
        if comm.enabled
            if ~isequaln(bus.command, lastCommand)
                if ~isempty(comm.command_topic)
                    comm.bus.publish(comm.command_topic, ...
                        aero_command_payload(bus.command), t, t, t, 'simulation');
                end
                lastCommand = bus.command;
            end
            if ~isequaln(bus.held_command, lastHeld)
                if ~isempty(comm.held_command_topic)
                    comm.bus.publish(comm.held_command_topic, ...
                        aero_command_payload(bus.held_command), t, t, t, 'simulation');
                end
                lastHeld = bus.held_command;
            end
        end
    end

    stateLog(:, g) = x;
    heldLog{g} = bus.held_command;
    commandLog{g} = bus.command;
end

wallTime = toc(timer);

% ---- 5. 组装结果 ------------------------------------------------------
eventExecTime = zeros(nEvents, 1);
if nEvents > 0
    eventExecTime = sched.times(sched.group);
end

result = struct();
result.schema = 'aero-sim-result/1';
result.model = model.name;
result.time = sched.times(:);
result.state = stateLog.';
result.events = struct( ...
    'time', num2cell(eventExecTime), ...
    'component', eventComponent, ...
    'action', eventAction, ...
    'priority', num2cell(sched.event_priority));
result.measurements = measurementLog(1:measurementCount);
result.held = heldLog;
result.computed = commandLog;
result.segments = segments(~cellfun(@isempty, segments));
result.solver = cfg.solver;
result.config = cfg;
% 机型可提供通用元数据（版本、质量、设备型号、数据来源等）；
% 内核只原样记录，不解释字段，也不要求特定结构。
if isfield(model, 'metadata') && isstruct(model.metadata)
    result.model_metadata = model.metadata;
end
result.environment = aero_env_info();
result.seed = cfg.seed;
result.stats = struct( ...
    'groups', nGroups, ...
    'events', nEvents, ...
    'segments', numel(result.segments), ...
    'latch_events', latchCount, ...
    'total_internal_points', totalInternalPoints, ...
    'wall_time_s', wallTime);

% 通信启用时把总线转为普通值快照（不含 handle），便于持久化与核查。
if comm.enabled
    result.communication = aero_communication_snapshot(comm.bus);
end

% ---- 6. 按配置持久化（默认只在内存中返回） ----------------------------
if cfg.save.enabled
    [savedPath, capacityCheck] = aero_save_result(result, cfg.save, cfg.results_root);
    result.saved_path = savedPath;
    result.results_check = capacityCheck;
end

if cfg.verbose
    fprintf(['aero_sim: model=%s groups=%d events=%d segments=%d ' ...
        'internal_points=%d wall=%.3fs\n'], model.name, nGroups, nEvents, ...
        numel(result.segments), totalInternalPoints, wallTime);
end

end

% ===========================================================================
function model = validate_model(model)
% 模型最小契约：名称（可缺省）与无副作用的动力学句柄
if ~isstruct(model) || ~isscalar(model)
    error('aero:config:model', 'model 必须是标量结构体');
end
if ~isfield(model, 'dynamics') || ~isa(model.dynamics, 'function_handle')
    error('aero:config:model', ...
        'model.dynamics 必须是函数句柄 f(t, x, u, env)');
end
if ~isfield(model, 'name') || isempty(model.name)
    model.name = 'unnamed';
end
model.name = char(model.name);
end
