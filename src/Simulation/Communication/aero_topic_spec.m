function spec = aero_topic_spec(fields, dimensions, units, options)
%AERO_TOPIC_SPEC 构造并校验 Topic 数据契约
%
% 输入：
%   fields     - 字段名 cellstr，例如 {'position','velocity'}
%   dimensions - 每个字段的尺寸 cell，例如 {[3 1],[3 1]}，标量为 [1 1]
%   units      - 每个字段的单位 cellstr，例如 {'m','m/s'}
%   options    - 可选结构体：
%       allow_ground_truth  逻辑标量，默认 false；仅允许以 "truth/" 开头的 Topic 置 true
%       max_age_s           最大允许消息年龄（s），正标量或 Inf，默认 Inf
%       description         用途说明（字符/字符串），默认 ''
%
% 输出 spec 为 Topic 契约结构，供 aero_topic_bus.register 使用。
%
% 契约只描述“数据长什么样、单位是什么、能否承载真值、多久算过期”，
% 不描述发布者身份或调度频率；时间与调度仍由 Simulation 负责。

if nargin < 3
    error('aero:bus:invalidSpec', 'aero_topic_spec 需要 fields、dimensions、units');
end
if nargin < 4 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('aero:bus:invalidSpec', 'options 必须是标量结构体');
end

fields = normalize_cellstr(fields, 'fields');
n = numel(fields);
if n == 0
    error('aero:bus:invalidSpec', 'Topic 至少需要一个字段');
end
if numel(unique(fields)) ~= n
    error('aero:bus:invalidSpec', 'Topic 字段名必须唯一');
end
for k = 1:n
    if ~isvarname(fields{k})
        error('aero:bus:invalidSpec', '字段名 "%s" 不是合法 MATLAB 标识符', fields{k});
    end
end

if ~iscell(dimensions) || numel(dimensions) ~= n
    error('aero:bus:invalidSpec', 'dimensions 必须是长度等于字段数的 cell');
end
for k = 1:n
    d = dimensions{k};
    if ~isnumeric(d) || numel(d) ~= 2 || any(~isfinite(d)) || any(d < 1) ...
            || any(d ~= fix(d))
        error('aero:bus:invalidSpec', '字段 "%s" 的尺寸必须为正整数 [rows cols]', fields{k});
    end
    dimensions{k} = double(d(:).');
end

units = normalize_cellstr(units, 'units');
if numel(units) ~= n
    error('aero:bus:invalidSpec', 'units 数量必须与字段数一致');
end
for k = 1:n
    if isempty(units{k})
        error('aero:bus:invalidSpec', '字段 "%s" 缺少单位', fields{k});
    end
end

spec = struct();
spec.fields = fields;
spec.dimensions = dimensions;
spec.units = units;
spec.allow_ground_truth = false;
spec.max_age_s = Inf;
spec.description = '';

if isfield(options, 'allow_ground_truth') && ~isempty(options.allow_ground_truth)
    if ~isscalar(options.allow_ground_truth) ...
            || ~(islogical(options.allow_ground_truth) || isnumeric(options.allow_ground_truth))
        error('aero:bus:invalidSpec', 'allow_ground_truth 必须是逻辑标量');
    end
    spec.allow_ground_truth = logical(options.allow_ground_truth);
end
if isfield(options, 'max_age_s') && ~isempty(options.max_age_s)
    age = options.max_age_s;
    if ~isnumeric(age) || ~isscalar(age) || ~isreal(age) || isnan(age) ...
            || age <= 0 || ~(isfinite(age) || isinf(age))
        error('aero:bus:invalidSpec', 'max_age_s 必须是正标量或 Inf');
    end
    spec.max_age_s = double(age);
end
if isfield(options, 'description') && ~isempty(options.description)
    if ~(ischar(options.description) || (isstring(options.description) ...
            && isscalar(options.description)))
        error('aero:bus:invalidSpec', 'description 必须是字符或字符串');
    end
    spec.description = char(options.description);
end

end

% ===========================================================================
function values = normalize_cellstr(values, label)
if ischar(values)
    values = {values};
elseif isstring(values)
    values = cellstr(values(:).');
elseif iscell(values)
    values = values(:).';
    if ~all(cellfun(@(v) ischar(v) || (isstring(v) && isscalar(v)), values))
        error('aero:bus:invalidSpec', '%s 必须是字符/字符串 cell 数组', label);
    end
    values = cellfun(@char, values, 'UniformOutput', false);
else
    error('aero:bus:invalidSpec', '%s 必须是字符/字符串 cell 数组', label);
end
end
