function components = aero_experiment_components(cfgComponents)
%AERO_EXPERIMENT_COMPONENTS 提取可序列化的组件调度字段
%
% 只保留签名/实验定义需要的字段，明确排除 step/sample 函数句柄。
% 输入为经过 aero_validate_config 的 cfg.components。

components = struct('name', {}, 'kind', {}, 'period', {}, 'priority', {}, ...
    'offset', {}, 'delay', {}, 'latches', {}, 'topic', {});
for k = 1:numel(cfgComponents)
    c = cfgComponents(k);
    components(k).name = c.name;
    components(k).kind = c.kind;
    components(k).period = c.period;
    components(k).priority = c.priority;
    components(k).offset = c.offset;
    components(k).delay = c.delay;
    components(k).latches = c.latches;
    components(k).topic = c.topic;
end
end
