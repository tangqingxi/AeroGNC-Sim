function bus = gnc_latch_motor_step(t, bus) %#ok<INUSD>
%GNC_LATCH_MOTOR_STEP 把计算电机命令锁存到执行器输入（50 Hz）
%
% 与舵面锁存独立；不改变内核的锁存机制，只按声明频率复制 motor 字段。

if ~isfield(bus.command, 'servos') || ~isfield(bus.command, 'motor')
    error('gnc:latch:command', 'bus.command 必须是含 servos/motor 的结构体');
end
bus.held_command.motor = bus.command.motor;

end
