function s = velocity_step_scenario(options)
%VELOCITY_STEP_SCENARIO 构建 SHC09 独立速度参考闭环场景
%
% 控制链：
%   Guidance（velocity_step）→ Velocity PID → Attitude P → Rate PID
%   → PINV Allocation → SHC09
%
% 该场景不装配 Position P，Velocity PID 直接订阅
% `guidance/velocity_reference`；Attitude/Rate/Allocation 使用现有控制器。
%
% 选项（全部可选）：
%   aero_source         气动数据来源（默认 external）
%   aero_domain_policy  'error'（默认）或 'clip'
%   velocity_reference  阶跃速度参考（NED，默认 [0.8;0;0] m/s）
%   acceleration_ff     前馈加速度（NED，默认零 m/s²）
%   start_s/end_s       阶跃起止（默认 0.5/2.5 s）
%   tspan               仿真区间（默认 [0,5] s）
%   yaw/yaw_rate        航向参考（默认 0）
%   gains               控制参数（默认 control_defaults）
%   max_records         发布日志容量（默认 300000）
%
% 输出 s：name/mode/model/cfg；唯一实验定义在 s.cfg.experiment。
%
% 物理边界：默认 fixture 只证明软件通路，不能作为真实气动/实机验证。

if nargin < 1
    options = struct();
end
options = fill_options(options);

model = shc09_model(struct( ...
    'aero_source', options.aero_source, ...
    'aero_domain_policy', options.aero_domain_policy));
gains = options.gains;

cfg = build_config(model, 'velocity', struct( ...
    'tspan', options.tspan, ...
    'velocity_reference', options.velocity_reference, ...
    'acceleration_ff', options.acceleration_ff, ...
    'yaw', options.yaw, ...
    'yaw_rate', options.yaw_rate, ...
    'start_s', options.start_s, ...
    'end_s', options.end_s, ...
    'gains', gains, ...
    'max_records', options.max_records));

s = struct();
s.name = cfg.experiment.definition.scenario;
s.mode = 'velocity';
s.model = model;
s.cfg = cfg;

end

% ===========================================================================
function options = fill_options(options)
if ~isstruct(options) || ~isscalar(options)
    error('gnc:scenario:options', 'options 必须是标量结构体');
end
defaults = struct( ...
    'aero_source', struct('type', 'external', 'file', '', 'tables', []), ...
    'aero_domain_policy', 'error', ...
    'velocity_reference', [0.8; 0; 0], ...
    'acceleration_ff', zeros(3, 1), ...
    'start_s', 0.5, ...
    'end_s', 2.5, ...
    'tspan', [0, 5], ...
    'yaw', 0, ...
    'yaw_rate', 0, ...
    'gains', control_defaults(), ...
    'max_records', 300000);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(options, names{k})
        options.(names{k}) = defaults.(names{k});
    end
end

if numel(options.velocity_reference) ~= 3 ...
        || any(~isfinite(options.velocity_reference(:)))
    error('gnc:scenario:velocityReference', ...
        'velocity_reference 必须是有限的 3×1 NED 速度');
end
if numel(options.acceleration_ff) ~= 3 ...
        || any(~isfinite(options.acceleration_ff(:)))
    error('gnc:scenario:accelerationFF', ...
        'acceleration_ff 必须是有限的 3×1 NED 加速度');
end
if numel(options.tspan) ~= 2 || any(~isfinite(options.tspan)) ...
        || options.tspan(2) <= options.tspan(1)
    error('gnc:scenario:tspan', 'tspan 必须是有限递增区间');
end
if ~isscalar(options.start_s) || ~isscalar(options.end_s) ...
        || ~isfinite(options.start_s) || ~isfinite(options.end_s)
    error('gnc:scenario:timing', 'start_s/end_s 必须是有限标量');
end
if options.start_s < options.tspan(1) || options.end_s <= options.start_s ...
        || options.end_s > options.tspan(2)
    error('gnc:scenario:timing', ...
        '必须满足 tspan(1) <= start_s < end_s <= tspan(2)');
end
if ~isscalar(options.yaw) || ~isscalar(options.yaw_rate) ...
        || ~isfinite(options.yaw) || ~isfinite(options.yaw_rate)
    error('gnc:scenario:yaw', 'yaw/yaw_rate 必须是有限标量');
end
end
