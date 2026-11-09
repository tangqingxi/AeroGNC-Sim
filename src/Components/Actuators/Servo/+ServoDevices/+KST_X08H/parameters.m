function p = parameters()
%PARAMETERS KST X08H 舵机型号参数（8 V 供电）
%
% 用途：作为 Servo 设备型号示例，供 Aircraft 装配时选择。
% 只描述舵机自身，不包含任何机型的安装位置、编号、方向、PWM 中值或行程限制。
%
% 字段：
%   device_name / device_class / model_type
%   tau_s              一阶时间常数（s）
%   rate_limit_rad_s   最大角速率（rad/s）
%   range_rad          设备自身行程能力；[-Inf, Inf] 表示型号文件不声明
%   rated_voltage_v    额定供电电压（V）
%   provenance         每个参数来源与可信等级
%
% 行程归属：
%   本文件**不声明** ±40° 行程。±40° 是 SHC09 六舵面装配的机械限制，
%   写在 shc09_parameters；最终生效行程由装配层做“设备能力 ∩ 装配限制”。
%
% 注意：tau_s 与 rate_limit_rad_s 目前是名义参数，不得表述为已辨识结果。

p = struct();
p.device_name = 'KST X08H';
p.device_class = 'servo';
p.model_type = 'first_order';

p.tau_s = 0.03;                        % 名义一阶时间常数，仿真假设
p.rate_limit_rad_s = deg2rad(600);     % 名义最大角速率，仿真假设
p.range_rad = [-Inf, Inf];             % 型号文件不声明行程
p.rated_voltage_v = 8.0;               % 硬件配置

p.provenance = struct( ...
    'tau_s', '仿真假设（历史模型 30 ms，未实机辨识）', ...
    'rate_limit_rad_s', '仿真假设（历史模型 600 deg/s，未实机辨识）', ...
    'range_rad', '未声明：±40° 属 SHC09 装配限制，不是本型号固有规格', ...
    'rated_voltage_v', '硬件配置（8 V）');

p.notes = '型号与供电来自实机配置；动态参数为名义值；行程由装配决定。';

end
