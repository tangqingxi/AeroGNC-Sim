function aero_validate_command(value, identifier, label)
%AERO_VALIDATE_COMMAND 校验命令，失败时抛出带标识符的错误
%
% 输入：
%   value      - 待校验命令
%   identifier - 错误标识符（例如 aero:config:command 或 aero:kernel:command）
%   label      - 错误信息中的人类可读位置说明
%
% 规则见 aero_is_valid_command：数值向量，或字段均为有限实数的单层结构体。

if ~aero_is_valid_command(value)
    error(identifier, ...
        '%s 必须是有限实数向量，或所有字段均为有限实数的结构体', label);
end

end
