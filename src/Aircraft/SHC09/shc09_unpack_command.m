function [servos, motor] = shc09_unpack_command(command)
%SHC09_UNPACK_COMMAND 解析 SHC09 的独立指令容器
%
% 命令是两条彼此独立的信号，不是七维向量：
%   command.servos - 六片舵面目标偏角（6×1，rad）
%   command.motor  - 旋翼目标转速（标量，rad/s）
%
% 该函数只做字段存在性、维度与有限性检查；不解释控制算法。

if ~isstruct(command) || ~isscalar(command)
    error('shc09:command:format', ...
        'SHC09 命令必须是结构体 command.servos / command.motor');
end
if ~isfield(command, 'servos') || ~isfield(command, 'motor')
    error('shc09:command:format', ...
        'SHC09 命令缺少 servos 或 motor 字段');
end

servos = command.servos(:);
motor = command.motor;

if ~isnumeric(servos) || numel(servos) ~= 6 || any(~isfinite(servos))
    error('shc09:command:servos', 'command.servos 必须是 6 个有限实数');
end
if ~isnumeric(motor) || ~isscalar(motor) || ~isfinite(motor)
    error('shc09:command:motor', 'command.motor 必须是有限实数标量');
end

end
