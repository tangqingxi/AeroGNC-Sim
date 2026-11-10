function identity = aero_experiment_identity(experiment, cfg, modelMetadata, components)
%AERO_EXPERIMENT_IDENTITY 由实验定义、运行配置和模型快照生成完整身份
%
% 输入：
%   experiment    - cfg.experiment（schema/definition/definition_sha256，
%                   可含 identity_sha256；计算前会移除该自引用字段）
%   cfg           - 实际运行配置（result.config）
%   modelMetadata - result.model_metadata
%   components    - 可序列化组件表（aero_experiment_components）
%
% 输出 identity 及 input_signature_sha256。相同输入必须得到相同文本/哈希；
% 不包含 run_id、output_root、wall_time 或函数句柄。

if isfield(experiment, 'identity_sha256')
    experiment = rmfield(experiment, 'identity_sha256');
end

runtime = struct();
runtime.tspan = cfg.tspan;
runtime.x0 = cfg.x0;
runtime.initial_command = cfg.initial_command;
runtime.seed = cfg.seed;
runtime.solver = cfg.solver;
runtime.environment = cfg.environment;
runtime.time_tolerance = cfg.time_tolerance;
runtime.components = components;
runtime.communication = cfg.communication;

identity = struct();
identity.schema = 'aero-experiment-identity/1';
identity.experiment = experiment;
identity.runtime = runtime;
identity.model = modelMetadata;
identity.input_signature_sha256 = '';
text = jsonencode(identity);
identity.input_signature_sha256 = aero_sha256_text(text);

end
