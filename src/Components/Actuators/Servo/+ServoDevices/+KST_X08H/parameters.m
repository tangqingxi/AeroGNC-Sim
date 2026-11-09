function p = parameters()
%PARAMETERS KST X08H 舵机型号参数（8 V 供电）
%
% 用途：作为 Servo 设备型号示例，供 Aircraft 装配时选择。
% 只描述舵机自身，不包含 SHC09 的安装位置、编号、方向或 PWM 中值。
%
% 字段：
%   device_name / device_class / model_type
%   tau_s              一阶时间常数（s）
%   rate_limit_rad_s   最大角速率（rad/s）
%   range_rad          机械行程 [min max]（rad）
%   rated_voltage_v    额定供电电压（V）
%   provenance         每个参数来源与可信等级
%
% 可信等级说明：
%   硬件确认  - 来自实机配置或厂家规格
%   仿真假设  - 历史仿真沿用值，未经实机辨识
%
% 注意：tau_s 与 rate_limit_rad_s 目前是名义参数，不得表述为已辨识结果。

p = struct();
p.device_name = 'KST X08H';
p.device_class = 'servo';
p.model_type = 'first_order';

p.tau_s = 0.03;                        % 名义一阶时间常数，仿真假设
p.rate_limit_rad_s = deg2rad(600);     % 名义最大角速率，仿真假设
p.range_rad = deg2rad([-40, 40]);      % 机械行程，硬件确认约 ±40°
p.rated_voltage_v = 8.0;               % 硬件配置

p.provenance = struct( ...
    'tau_s', '仿真假设（历史模型 30 ms，未实机辨识）', ...
    'rate_limit_rad_s', '仿真假设（历史模型 600 deg/s，未实机辨识）', ...
    'range_rad', '硬件确认（约 ±40° 机械行程）', ...
    'rated_voltage_v', '硬件配置（8 V）');

p.notes = '型号与行程来自实机配置；动态参数为名义值，替换为实测值无需修改通用动力学。';

end
