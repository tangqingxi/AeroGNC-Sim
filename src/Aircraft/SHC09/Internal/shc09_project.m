function x = shc09_project(x, p)
%SHC09_PROJECT SHC09 状态投影（四元数归一化与执行器约束）
%
% 由 Simulation 在每个积分段结束后调用（model.project），保证：
%   1. 四元数单位范数（抑制数值漂移，不改变姿态方向）；
%   2. 舵面偏角在生效行程内（设备能力 ∩ 装配限制）；
%   3. 旋翼转速在 [物理停机转速, 最大转速] 内；允许 0 转速停机。
%
% 该函数只做数值约束，不修改位置、速度、角速度。

s = shc09_state_layout();

q = x(s.quaternion);
n = norm(q);
if n <= 0
    error('shc09:state:quaternion', '四元数范数为零，状态无效');
end
x(s.quaternion) = q / n;

range = p.servo_effective_range_rad;
x(s.servo) = min(max(x(s.servo), range(1)), range(2));

stopSpeed = p.motor_device.omega_stop_rad_s;
maxSpeed = p.motor_device.omega_range_rad_s(2);
x(s.motor) = min(max(x(s.motor), stopSpeed), maxSpeed);

end
