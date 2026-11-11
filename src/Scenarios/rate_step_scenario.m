function s = rate_step_scenario(options)
%RATE_STEP_SCENARIO 构建 SHC09 单轴角速度阶跃场景
%
% 用法：
%   s = rate_step_scenario(struct('aero_source', fixture));
%   result = aero_sim(s.model, s.cfg);
%
% 选项（全部可选）：
%   aero_source, aero_domain_policy
%   axis          1=滚转、2=俯仰、3=偏航（默认 1）
%   amplitude     阶跃幅值（rad/s，默认 0.5）
%   start_s/end_s 阶跃区间（默认 0.5/1.5 s）
%   tspan         默认 [0, 2.5] s
%   thrust_reference_n 固定总推力（N，默认 trim.thrust_n）
%   gains/max_records
%
% 只运行 角速度环 + PINV 分配 + 执行器，不启用位置/姿态外环，因此可以
% 独立检查内环方向、时延与饱和。

if nargin < 1
    options = struct();
end
options = fill_options(options);

model = shc09_model(struct( ...
    'aero_source', options.aero_source, ...
    'aero_domain_policy', options.aero_domain_policy));
% 固定推力是场景参考，按所装配电机的实际范围校验，不依赖分配器默认值。
thrust = options.thrust_reference_n;
if isempty(thrust)
    thrust = model.trim.thrust_n;
end
kT0 = model.parameters.duct.k_T0;
omegaRange = model.parameters.motor_device.omega_range_rad_s;
if ~isnumeric(thrust) || ~isreal(thrust) || ~isscalar(thrust) || ~isfinite(thrust) ...
        || thrust < kT0 * omegaRange(1)^2 || thrust > kT0 * omegaRange(2)^2
    error('gnc:scenario:thrust', 'thrust_reference_n 必须落在电机实际推力范围内（N）');
end
gains = options.gains;

cfg = build_config(model, 'rate', struct( ...
    'tspan', options.tspan, ...
    'axis', options.axis, ...
    'amplitude', options.amplitude, ...
    'thrust_reference_n', thrust, ...
    'start_s', options.start_s, ...
    'end_s', options.end_s, ...
    'gains', gains, ...
    'max_records', options.max_records));

s = struct();
s.name = cfg.experiment.definition.scenario;
s.mode = 'rate';
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
    'axis', 1, ...
    'amplitude', 0.5, ...
    'thrust_reference_n', [], ...
    'start_s', 0.5, ...
    'end_s', 1.5, ...
    'tspan', [0, 2.5], ...
    'gains', control_defaults(), ...
    'max_records', 200000);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(options, names{k})
        options.(names{k}) = defaults.(names{k});
    end
end
if ~any(options.axis == [1, 2, 3])
    error('gnc:scenario:axis', 'axis 必须是 1、2 或 3');
end
if options.end_s <= options.start_s
    error('gnc:scenario:timing', 'end_s 必须大于 start_s');
end
end
