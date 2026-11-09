function comm = aero_communication_setup(config)
%AERO_COMMUNICATION_SETUP 根据配置创建并注册唯一的 Topic Bus
%
% 输入 config 为 cfg.communication：
%   enabled             逻辑标量，默认 false
%   topics              cell 数组，每项 struct('name', char, 'spec', spec)
%   command_topic       计算命令 Topic（空表示不发布）
%   held_command_topic  锁存命令 Topic（空表示不发布）
%
% 输出 comm：
%   enabled, bus（aero_topic_bus 或 []）, topic_names, command_topic, held_command_topic
%
% 说明：
%   - 通信关闭时不创建 Bus，避免无用的通信状态；
%   - 命令 Topic 必须与已注册 Topic 同名，防止发布到未声明契约；
%   - 该函数是 aero_sim 内部装配步骤，不实现时钟或调度。

comm = struct();
comm.enabled = false;
comm.bus = [];
comm.topic_names = {};
comm.command_topic = '';
comm.held_command_topic = '';

if ~isstruct(config) || ~isfield(config, 'enabled') || ~config.enabled
    return;
end

topics = config.topics;
if ~iscell(topics)
    error('aero:comm:config', 'communication.topics 必须是 cell 数组');
end

bus = aero_topic_bus();
for k = 1:numel(topics)
    entry = topics{k};
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

if ~isempty(commandTopic) && ~bus.isRegistered(commandTopic)
    error('aero:comm:unknownCommandTopic', ...
        'command_topic "%s" 不在已注册 Topic 中', commandTopic);
end
if ~isempty(heldTopic) && ~bus.isRegistered(heldTopic)
    error('aero:comm:unknownCommandTopic', ...
        'held_command_topic "%s" 不在已注册 Topic 中', heldTopic);
end

comm.enabled = true;
comm.bus = bus;
comm.topic_names = bus.topicNames();
comm.command_topic = commandTopic;
comm.held_command_topic = heldTopic;

end
