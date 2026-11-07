function cfg = aero_fill_defaults(cfg, defaults)
%AERO_FILL_DEFAULTS 用默认值补齐缺失的配置字段
%
% 只填充“不存在”的字段；已有字段（包括空值）保持原样，交由
% aero_validate_config 判定合法性，避免默认值掩盖用户错误。
% 标量结构体递归补齐（例如 solver），结构数组（components）不逐元素合并。

if isempty(cfg)
    cfg = defaults;
    return;
end
if ~isstruct(cfg) || ~isscalar(cfg)
    error('aero:config:type', '配置必须是标量结构体');
end

names = fieldnames(defaults);
for k = 1:numel(names)
    name = names{k};
    if ~isfield(cfg, name)
        cfg.(name) = defaults.(name);
    elseif isstruct(defaults.(name)) && isstruct(cfg.(name)) ...
            && isscalar(defaults.(name)) && isscalar(cfg.(name))
        cfg.(name) = aero_fill_defaults(cfg.(name), defaults.(name));
    end
end

end
