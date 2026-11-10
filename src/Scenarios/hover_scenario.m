function s = hover_scenario(options)
%HOVER_SCENARIO 构建 SHC09 悬停位置保持闭环场景
%
% 用法：
%   s = hover_scenario(struct('aero_source', fixture));
%   result = aero_sim(s.model, s.cfg);
%
% 选项（全部可选）：
%   aero_source       气动数据来源（默认 external，需外部文件）
%   aero_domain_policy 'error'（默认）或 'clip'
%   tspan             仿真区间（默认 [0, 10] s）
%   position_offset   初始位置偏差（NED，默认 [0;0;0]）
%   reference_position 目标位置（NED，默认原点）
%   yaw               目标偏航（rad，默认 0）
%   yaw_rate          目标偏航角速率（rad/s，默认 0）
%   gains             控制参数（默认 control_defaults）
%   max_records       发布日志容量（默认 300000）
%
% 输出 s：name/model/cfg/gains/mode，可直接交给 aero_sim。
%
% 物理边界：真实气动数据未入库时，fixture 只能证明软件通路，不能作为
% 物理闭环验证；调用方必须保留这一区分。

if nargin < 1
    options = struct();
end
options = fill_options(options);

model = shc09_model(struct( ...
    'aero_source', options.aero_source, ...
    'aero_domain_policy', options.aero_domain_policy));

gains = options.gains;

cfg = build_config(model, 'hover', struct( ...
    'tspan', options.tspan, ...
    'position_offset', options.position_offset, ...
    'reference_position', options.reference_position, ...
    'yaw', options.yaw, ...
    'yaw_rate', options.yaw_rate, ...
    'gains', gains, ...
    'max_records', options.max_records));
cfg = aero_validate_config(cfg);

s = struct();
s.name = 'hover_hold';
s.mode = 'hover';
s.model = model;
s.cfg = cfg;
s.gains = gains;
s.trim = model.trim;
s.reference = struct('position', options.reference_position(:), ...
    'velocity', zeros(3, 1), 'acceleration', zeros(3, 1), ...
    'yaw', options.yaw, 'yaw_rate', options.yaw_rate);
s.signature = experiment_signature(s.name, cfg, model, gains, s.reference);

end

% ===========================================================================
function options = fill_options(options)
if ~isstruct(options) || ~isscalar(options)
    error('gnc:scenario:options', 'options 必须是标量结构体');
end
defaults = struct( ...
    'aero_source', struct('type', 'external', 'file', '', 'tables', []), ...
    'aero_domain_policy', 'error', ...
    'tspan', [0, 10], ...
    'position_offset', zeros(3, 1), ...
    'reference_position', zeros(3, 1), ...
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
if numel(options.position_offset) ~= 3 || numel(options.reference_position) ~= 3
    error('gnc:scenario:position', 'position_offset/reference_position 必须是 3×1');
end
end
