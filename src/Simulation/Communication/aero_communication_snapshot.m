function snap = aero_communication_snapshot(bus)
%AERO_COMMUNICATION_SNAPSHOT 把运行时总线转为可持久化的普通值结构
%
% 输入：bus - aero_topic_bus 实例（运行时唯一通信状态）
% 输出：snap - 不含任何 handle 的普通结构体：
%   schema         快照格式标识
%   enabled        恒为 true（只有启用通信时才生成快照）
%   topic_names    注册 Topic 列表
%   publish_count  全局发布次数
%   messages       每个 Topic 的最后一条消息（未发布为 []），按 topic_names 顺序
%   log            发布日志快照（含 records/dropped/容量）
%
% 结果中只保存该快照，不保存 Bus 句柄，避免可变对象进入正式科研数据契约。
% 快照中的 data 为发布时的载荷副本（MATLAB 值语义）。

if nargin < 1 || isempty(bus) || ~isa(bus, 'aero_topic_bus')
    error('aero:comm:snapshot', 'aero_communication_snapshot 需要 aero_topic_bus 实例');
end

names = bus.topicNames();
messages = cell(1, numel(names));
for k = 1:numel(names)
    if bus.hasMessage(names{k})
        messages{k} = bus.latest(names{k}, 'diagnostic');
    else
        messages{k} = [];
    end
end

snap = struct();
snap.schema = 'aero-comm-snapshot/1';
snap.enabled = true;
snap.topic_names = names;
snap.publish_count = bus.sequence();
snap.messages = messages;
snap.log = bus.logSnapshot();

end
