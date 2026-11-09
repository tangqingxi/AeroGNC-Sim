function payload = aero_command_payload(command)
%AERO_COMMAND_PAYLOAD 把命令值转成 Topic 载荷结构（通用，不解释字段）
%
% 规则：
%   - 数值命令 → struct('value', command(:))，与契约 {'value'} 配套；
%   - 结构体命令 → 原样返回，由调用方在契约中声明同名字段；
%   - 其他类型不允许进入通信总线。
%
% 该函数只做形状整理，不改变命令语义，也不参与锁存。

if isnumeric(command)
    payload = struct('value', command(:));
    return;
end
if isstruct(command) && isscalar(command)
    payload = command;
    return;
end
error('aero:comm:commandPayload', '命令必须是数值或标量结构体');
end
