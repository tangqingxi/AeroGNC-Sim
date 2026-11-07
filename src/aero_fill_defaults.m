function cfg = aero_fill_defaults(cfg, defaults)
%AERO_FILL_DEFAULTS Fill missing configuration fields from a defaults struct.
%
%   Only absent fields are filled. Nested structs are filled recursively when
%   both sides are scalar structs; struct arrays (for example cfg.components)
%   are never merged element-wise.

if isempty(cfg)
    cfg = defaults;
    return;
end
if ~isstruct(cfg) || ~isscalar(cfg)
    error('aero:config:type', 'configuration must be a scalar struct');
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
