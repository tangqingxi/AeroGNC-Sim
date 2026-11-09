function p = parameters()
%PARAMETERS MAD M6C06 320KV V2 电机型号参数
%
% 用途：作为 Motor 设备型号示例，供 Aircraft 装配时选择。
% 只描述电机/ESC 组合的转速动态，不包含 SHC09 的旋翼方向或安装信息。
%
% 字段：
%   device_name / device_class / model_type
%   tau_s               一阶时间常数（s）
%   omega_range_rad_s   允许转速范围 [min max]（rad/s）
%   accel_limit_rad_s2  加速度限制（rad/s²，Inf 表示不限）
%   kv_rpm_per_v        厂家标称 KV 值（rpm/V），仅作型号标识
%
% 注意：tau_s 与转速范围沿用历史仿真设置，未经实机辨识；KV 值不代表
% 本模型直接用电压驱动转速，接口仍以转速指令 rad/s 为准。

p = struct();
p.device_name = 'MAD M6C06 320KV V2';
p.device_class = 'motor';
p.model_type = 'first_order';

p.tau_s = 0.05;                       % 名义一阶时间常数，仿真假设
p.omega_range_rad_s = [200, 1200];    % 转速范围，仿真假设
p.accel_limit_rad_s2 = Inf;           % 未声明加速度限制
p.kv_rpm_per_v = 320;                 % 厂家型号标识

p.provenance = struct( ...
    'tau_s', '仿真假设（历史模型 50 ms，未实机辨识）', ...
    'omega_range_rad_s', '仿真假设（历史模型 200–1200 rad/s）', ...
    'accel_limit_rad_s2', '未声明（Inf，表示不加限制）', ...
    'kv_rpm_per_v', '厂家标称型号参数（仅作标识）');

p.notes = '转速范围沿用历史设置；如获得实测辨识结果，只改本文件即可。';

end
