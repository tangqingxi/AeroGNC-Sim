function dOmega = aero_motor_dynamics(Omega, command, params)
%AERO_MOTOR_DYNAMICS 通用电机转速一阶连续动力学（与设备品牌无关）
%
% 输入：
%   Omega   - 当前实际转速（rad/s）
%   command - 目标转速（rad/s），Aircraft 锁存后传入
%   params  - 设备参数结构，至少含：
%       tau_s              一阶时间常数（s）
%       omega_range_rad_s  转速范围 [min max]（rad/s）
%       accel_limit_rad_s2 可选；加速度限制（rad/s²，Inf 表示不限）
%
% 输出：
%   dOmega  - 转速导数（rad/s²）
%
% 模型：
%   Ω̇ = clamp((Ω_cmd − Ω)/τ, −a_max, +a_max)
% 目标转速先按设备转速范围限幅；到达转速边界且目标继续向外时导数置零。
%
% 该函数不读取设备文件，也不区分电机型号。

limits = params.omega_range_rad_s;
tau = params.tau_s;

target = min(max(command, limits(1)), limits(2));
rate = (target - Omega) ./ tau;

if isfield(params, 'accel_limit_rad_s2') && isfinite(params.accel_limit_rad_s2)
    aMax = params.accel_limit_rad_s2;
    rate = min(max(rate, -aMax), aMax);
end

if Omega >= limits(2) - 1e-9 && rate > 0
    rate = 0;
elseif Omega <= limits(1) + 1e-9 && rate < 0
    rate = 0;
end

dOmega = rate;

end
