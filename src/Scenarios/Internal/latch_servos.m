function bus = latch_servos(t, bus) %#ok<INUSD>
%LATCH_SERVOS 把计算舵面命令锁存到执行器输入（200 Hz）
%
% 只复制 servos 字段；motor 由独立锁存组件按自身频率更新，保留
% command（计算值）与 held_command（真正进入动力学）的独立语义。

if ~isfield(bus.command, 'servos') || ~isfield(bus.command, 'motor')
    error('gnc:latch:command', 'bus.command 必须是含 servos/motor 的结构体');
end
bus.held_command.servos = bus.command.servos;

end
