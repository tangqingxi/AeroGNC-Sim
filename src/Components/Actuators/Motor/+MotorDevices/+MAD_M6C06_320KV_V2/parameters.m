function p = parameters()
%PARAMETERS MAD M6C06 320KV V2 电机型号参数
%
% 用途：作为 Motor 设备型号示例，供 Aircraft 装配时选择。
% 只描述电机/ESC 组合的转速动态，不包含 SHC09 的旋翼方向或安装信息。
%
% 字段：
%   device_name / device_class / model_type
%   tau_s                一阶时间常数（s）
%   omega_range_rad_s    运行转速范围 [min max]（rad/s）；min 是可控最低运行转速
%   omega_stop_rad_s     物理停机转速（0）
%   stop_threshold_rad_s 指令低于该值即视为停机指令（rad/s）
%   accel_limit_rad_s2   加速度限制（rad/s²，Inf 表示不限）
%   kv_rpm_per_v         厂家标称 KV 值（rpm/V），仅作型号标识
%
% 说明：
%   200 rad/s 是最低**运行**转速，不是物理下限；停机指令允许真实 0 转速。
%   tau_s、转速范围沿用历史仿真设置，未经实机辨识。

p = struct();
p.device_name = 'MAD M6C06 320KV V2';
p.device_class = 'motor';
p.model_type = 'first_order';

p.tau_s = 0.05;                       % 名义一阶时间常数，仿真假设
p.omega_range_rad_s = [200, 1200];    % 运行转速范围，仿真假设
p.omega_stop_rad_s = 0;               % 物理停机转速
p.stop_threshold_rad_s = 1;           % 停机指令阈值
p.accel_limit_rad_s2 = Inf;           % 未声明加速度限制
p.kv_rpm_per_v = 320;                 % 厂家型号标识

p.provenance = struct( ...
    'tau_s', '仿真假设（历史模型 50 ms，未实机辨识）', ...
    'omega_range_rad_s', '仿真假设（历史模型 200–1200 rad/s，最低为运行转速）', ...
    'omega_stop_rad_s', '物理定义（停机允许 0 转速）', ...
    'stop_threshold_rad_s', '接口约定（低于 1 rad/s 视为停机指令）', ...
    'accel_limit_rad_s2', '未声明（Inf，表示不加限制）', ...
    'kv_rpm_per_v', '厂家标称型号参数（仅作标识）');

p.notes = '最低运行转速 200 rad/s 不阻止停机；如获得辨识结果，只改本文件。';

end
