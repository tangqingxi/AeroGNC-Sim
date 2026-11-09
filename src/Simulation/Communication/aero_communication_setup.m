function comm = aero_communication_setup(config)
%AERO_COMMUNICATION_SETUP 创建并注册 V1.0 唯一的 Topic Bus
%
% 输入 config 为 cfg.communication：
%   topics              cell 数组，每项 struct('name', char, 'spec', spec)
%   command_topic       计算命令 Topic（空表示不发布）
%   held_command_topic  锁存命令 Topic（空表示不发布）
%   log                 可选发布日志：enabled/topics/max_records
%
% 输出 comm：
%   bus, topic_names, command_topic, held_command_topic
%
% 说明：
%   - V1.0 中通信始终启用，Simulation 只持有一个 Bus，不存在关闭模式；
%   - 命令 Topic 必须与已注册 Topic 同名，防止发布到未声明契约；
%   - 该函数是 aero_sim 内部装配步骤，不实现时钟或调度。

if ~isstruct(config) || ~isscalar(config)
    error('aero:comm:config', 'communication 配置必须是标量结构体');
end

if ~isfield(config, 'topics')
    config.topics = {};
end
if ~iscell(config.topics)
    error('aero:comm:config', 'communication.topics 必须是 cell 数组');
end

bus = aero_topic_bus();
for k = 1:numel(config.topics)
    entry = config.topics{k};
    if ~isstruct(entry) || ~isfield(entry, 'name') || ~isfield(entry, 'spec')
        error('aero:comm:config', 'communication.topics{%d} 必须含 name 与 spec', k);
    end
    bus.register(entry.name, entry.spec);
end

commandTopic = '';
if isfield(config, 'command_topic') && ~isempty(config.command_topic)
    commandTopic = char(config.command_topic);
end
heldTopic = '';
if isfield(config, 'held_command_topic') && ~isempty(config.held_command_topic)
    heldTopic = char(config.held_command_topic);
end

if ~isempty(commandTopic) && ~isempty(heldTopic) && strcmp(commandTopic, heldTopic)
    error('aero:comm:duplicateCommandTopic', ...
        'command_topic 与 held_command_topic 不能是同一 Topic："%s"', commandTopic);
end
if ~isempty(commandTopic) && ~bus.isRegistered(commandTopic)
    error('aero:comm:unknownCommandTopic', ...
        'command_topic "%s" 不在已注册 Topic 中', commandTopic);
end
if ~isempty(heldTopic) && ~bus.isRegistered(heldTopic)
    error('aero:comm:unknownCommandTopic', ...
        'held_command_topic "%s" 不在已注册 Topic 中', heldTopic);
end

% ---- 可选发布日志（默认关闭） ---------------------------------------
logConfig = struct('enabled', false, 'topics', {{}}, 'max_records', 10000);
if isfield(config, 'log') && ~isempty(config.log)
    if ~isstruct(config.log) || ~isscalar(config.log)
        error('aero:comm:config', 'communication.log 必须是标量结构体');
    end
    logConfig = config.log;
end
if isfield(logConfig, 'enabled') && logical(logConfig.enabled)
    if ~isfield(logConfig, 'topics')
        error('aero:comm:config', '日志启用时必须给出 topics');
    end
    maxRecords = 10000;
    if isfield(logConfig, 'max_records') && ~isempty(logConfig.max_records)
        maxRecords = logConfig.max_records;
    end
    bus.enableLog(logConfig.topics, maxRecords);
end

comm = struct();
comm.bus = bus;
comm.topic_names = bus.topicNames();
comm.command_topic = commandTopic;
comm.held_command_topic = heldTopic;

end
