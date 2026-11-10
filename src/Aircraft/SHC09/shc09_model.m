function model = shc09_model(options)
%SHC09_MODEL 构造可直接交给 aero_sim 的 SHC09 机型模型
%
% 用法：
%   model = shc09_model();                       % 悬停配平初态
%   model = shc09_model(struct('aero_source', ...));
%   cfg.initial_command = model.initial_command;
%   result = aero_sim(model, cfg);
%
% options 字段（全部可选）：
%   overrides          参数覆盖结构体（记录在 p.overrides）
%   aero_source        气动数据来源（默认 external，需外部数据文件）
%   aero_domain_policy 'error'（默认）或 'clip'
%   initial_mode       'hover'（默认，静态配平）或 'rest'
%
% 返回 model 字段：
%   name/version, x0, dynamics, project, outputs, state_layout,
%   parameters, aero, initial_command, trim
%
% 该函数只做装配：物理方程在 shc09_dynamics，设备动态在 Components。

if nargin < 1
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('shc09:model:options', 'options 必须是标量结构体');
end

overrides = struct();
if isfield(options, 'overrides')
    overrides = options.overrides;
end
p = shc09_parameters(overrides);

if isfield(options, 'aero_source')
    p.aero_source = options.aero_source;
end
if isfield(options, 'aero_domain_policy')
    p.aero_domain_policy = options.aero_domain_policy;
end

aero = shc09_aero_data(p.aero_source);

mode = 'hover';
if isfield(options, 'initial_mode')
    mode = lower(char(options.initial_mode));
end

switch mode
    case 'hover'
        trim = shc09_hover_equilibrium(p);
        x0 = trim.x0;
        initialCommand = trim.command;
    case 'rest'
        trim = struct();
        s = shc09_state_layout();
        x0 = zeros(s.nx, 1);
        x0(s.quaternion) = [1; 0; 0; 0];
        initialCommand = struct('servos', zeros(6, 1), 'motor', 0);
    otherwise
        error('shc09:model:initialMode', '未知 initial_mode：%s', mode);
end

model = struct();
model.name = 'SHC09';
model.version = p.version;
model.x0 = x0;
model.dynamics = @(t, x, command, env) shc09_dynamics(t, x, command, env, p, aero);
model.project = @(x) shc09_project(x, p);
model.outputs = @(t, x, env) shc09_outputs(t, x, p, aero, env);
model.state_layout = shc09_state_layout();
model.parameters = p;
model.aero = aero;
model.metadata = shc09_metadata(p, aero, trim);
model.initial_command = initialCommand;
model.trim = trim;
model.initial_mode = mode;

end
