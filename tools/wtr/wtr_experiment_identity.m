function identity = wtr_experiment_identity(result, model)
%WTR_EXPERIMENT_IDENTITY 从结果自身恢复并校验实验身份
%
% 只读取 result.config.experiment、result.config、result.model_metadata；
% 不接受调用者提供的 gains/signature/scenario/method/seed。
% 任何定义或运行配置被篡改都会在这里被拒绝。

if ~isfield(result, 'config') || ~isfield(result.config, 'experiment')
    error('wtr:identity:missing', '结果缺少 result.config.experiment');
end
experiment = result.config.experiment;
if ~isstruct(experiment) || ~isfield(experiment, 'schema') ...
        || ~isfield(experiment, 'definition') ...
        || ~isfield(experiment, 'definition_sha256')
    error('wtr:identity:schema', '实验定义结构不完整');
end
if ~strcmp(experiment.schema, 'aero-experiment-definition/1')
    error('wtr:identity:schema', '不支持的实验定义 schema：%s', experiment.schema);
end

definitionPayload = struct('schema', experiment.schema, ...
    'definition', experiment.definition);
actualDefinitionHash = aero_sha256_text(aero_canonical_json(definitionPayload));
if ~strcmp(actualDefinitionHash, experiment.definition_sha256)
    error('wtr:identity:definition', '实验定义 SHA256 不一致');
end

components = aero_experiment_components(result.config.components);
identity = aero_experiment_identity(experiment, result.config, ...
    result.model_metadata, components);
if ~isfield(experiment, 'identity_sha256') ...
        || ~strcmp(identity.input_signature_sha256, experiment.identity_sha256)
    error('wtr:identity:input', '实验身份与仿真时保存的 identity_sha256 不一致');
end

if ~isfield(result, 'model') || ~strcmp(result.model, model.name)
    error('wtr:identity:model', '传入 model.name 与结果记录不一致');
end
if ~isequaln(model.metadata, result.model_metadata)
    error('wtr:identity:model', '传入 model.metadata 与结果快照不一致');
end
if ~isfield(model, 'state_layout') || ~isequaln(model.state_layout, ...
        result.model_metadata.state_layout)
    error('wtr:identity:layout', '传入 model.state_layout 与结果快照不一致');
end
if ~isfield(result.model_metadata, 'model_name') ...
        || ~strcmp(result.model_metadata.model_name, model.name) ...
        || ~strcmp(result.model_metadata.model_version, model.version)
    error('wtr:identity:model', '模型名称/版本与结果快照不一致');
end

definition = experiment.definition;
identity.scenario = definition.scenario;
identity.method = definition.method;
identity.seed = result.config.seed;
end
