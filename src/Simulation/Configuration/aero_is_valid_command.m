function ok = aero_is_valid_command(value)
%AERO_IS_VALID_COMMAND 判断命令值是否合法（通用，不解释字段含义）
%
% 合法命令：
%   - 非空有限实数向量/标量；或
%   - 单层标量结构体，且每个字段都是非空有限实数。
%
% 该函数只做类型与数值检查，不预设字段名、字段数量或物理含义，
% 因此 Simulation 不需要知道 servos / motor 等具体字段。

if isnumeric(value)
    ok = ~isempty(value) && isreal(value) && all(isfinite(value(:)));
    return;
end

if isstruct(value) && isscalar(value)
    names = fieldnames(value);
    ok = ~isempty(names);
    for k = 1:numel(names)
        field = value.(names{k});
        if ~(isnumeric(field) && isreal(field) && ~isempty(field) ...
                && all(isfinite(field(:))))
            ok = false;
            return;
        end
    end
    return;
end

ok = false;

end
