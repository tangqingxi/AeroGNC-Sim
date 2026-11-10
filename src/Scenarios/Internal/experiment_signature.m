function signature = experiment_signature(name, cfg, model, gains, extra)
%EXPERIMENT_SIGNATURE 生成 WTR/实验使用的完整身份签名
%
% 覆盖场景、参考条件、方法、种子、机型版本、完整生效控制参数、求解器、
% 时间区间、气动来源/可信等级/SHA256、参数覆盖与组件频率。签名由导出器
% 哈希后写入 manifest.source.input_signature；不得含时间戳或文件路径噪音。

if nargin < 5
    extra = struct();
end

components = struct('name', {}, 'period', {}, 'priority', {});
for k = 1:numel(cfg.components)
    components(k).name = cfg.components(k).name;
    components(k).period = cfg.components(k).period;
    components(k).priority = cfg.components(k).priority;
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
signature.model = struct('name', model.name, 'version', model.version);
signature.gains = gains;
signature.aero = aero;
signature.overrides = model.parameters.overrides;
signature.components = components;
signature.extra = extra;
signature.initial_command = cfg.initial_command;
signature.x0 = cfg.x0;
end
