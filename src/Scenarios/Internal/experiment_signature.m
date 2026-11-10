function signature = experiment_signature(name, cfg, model, gains, extra)
%EXPERIMENT_SIGNATURE 生成 WTR/实验使用的完整身份签名
%
% 覆盖场景、参考条件、方法、种子、机型版本、完整生效控制参数、求解器、
% 时间区间、气动来源/可信等级/SHA256、参数覆盖与组件频率。签名由导出器
% 哈希后写入 manifest.source.input_signature；不得含时间戳或文件路径噪音。

if nargin < 5
    extra = struct();
end

components = struct('name', {}, 'kind', {}, 'period', {}, 'priority', {}, ...
    'offset', {}, 'delay', {}, 'latches', {}, 'topic', {});
for k = 1:numel(cfg.components)
    c = cfg.components(k);
    components(k).name = c.name;
    components(k).kind = c.kind;
    components(k).period = c.period;
    components(k).priority = c.priority;
    components(k).offset = c.offset;
    components(k).delay = c.delay;
    components(k).latches = c.latches;
    components(k).topic = c.topic;
end

aero = struct('source_type', model.metadata.aero.source_type, ...
    'trust', model.metadata.aero.trust, ...
    'file_sha256', model.metadata.aero.file_sha256, ...
    'physical_validation_allowed', model.aero.physical_validation_allowed);

signature = struct();
signature.scenario = name;
signature.method = 'P_PID_P_PID_PINV';
signature.seed = cfg.seed;
signature.tspan = cfg.tspan;
signature.solver = cfg.solver;
signature.environment = cfg.environment;
signature.time_tolerance = cfg.time_tolerance;
signature.model = struct('name', model.name, 'version', model.version);
signature.gains = gains;
signature.aero = aero;
signature.overrides = model.parameters.overrides;
signature.components = components;
signature.extra = extra;
signature.initial_command = cfg.initial_command;
signature.initial_state = cfg.x0;
end
