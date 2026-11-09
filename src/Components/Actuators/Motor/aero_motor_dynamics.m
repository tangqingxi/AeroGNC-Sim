function dOmega = aero_motor_dynamics(Omega, command, params)
%AERO_MOTOR_DYNAMICS 通用电机转速一阶连续动力学（与设备品牌无关）
%
% 输入：
%   Omega   - 当前实际转速（rad/s）
%   command - 目标转速（rad/s），Aircraft 锁存后传入
%   params  - 设备参数结构：
%       model_type          必须为 'first_order'
%       tau_s               一阶时间常数（s）
%       omega_range_rad_s   运行转速范围 [min max]（rad/s），min 为可控最低转速
%       omega_stop_rad_s    物理停机转速（默认为 0）
%       stop_threshold_rad_s 指令低于该阈值即视为停机指令（默认 1 rad/s）
%       accel_limit_rad_s2  可选；加速度限制（rad/s²，Inf 表示不限）
%
% 输出：
%   dOmega  - 转速导数（rad/s²）
%
% 语义：
%   停机指令（command ≤ stop_threshold）→ 目标为物理停机转速 0，
%   允许真实零转速保持；运行指令则先限幅到 [min, max]。
%   因此“最低运行转速 200 rad/s”不阻止停机，也不会把停机状态强行抬到 200。
%
% 该函数不读取设备文件，也不区分电机型号。

require_first_order(params, '电机');

limits = params.omega_range_rad_s;
tau = params.tau_s;

stopSpeed = 0;
if isfield(params, 'omega_stop_rad_s') && ~isempty(params.omega_stop_rad_s)
    stopSpeed = params.omega_stop_rad_s;
end
stopThreshold = 1;
if isfield(params, 'stop_threshold_rad_s') && ~isempty(params.stop_threshold_rad_s)
    stopThreshold = params.stop_threshold_rad_s;
end

if command <= stopThreshold
    target = stopSpeed;
else
    target = min(max(command, limits(1)), limits(2));
end

rate = (target - Omega) ./ tau;

if isfield(params, 'accel_limit_rad_s2') && isfinite(params.accel_limit_rad_s2)
    aMax = params.accel_limit_rad_s2;
    rate = min(max(rate, -aMax), aMax);
end

% 到达边界且目标继续向外时导数置零（含物理停机）
if Omega >= limits(2) - 1e-9 && rate > 0
    rate = 0;
elseif Omega <= stopSpeed + 1e-9 && rate < 0
    rate = 0;
end

dOmega = rate;

end

% ===========================================================================
function require_first_order(params, deviceLabel)
if ~isstruct(params) || ~isfield(params, 'model_type')
    error('aero:components:modelType', ...
        '%s 参数缺少 model_type；V0.1 仅支持 first_order', deviceLabel);
end
if ~strcmpi(char(params.model_type), 'first_order')
    error('aero:components:modelType', ...
        '%s 动力学未实现 model_type="%s"；V0.1 仅支持 first_order', ...
        deviceLabel, char(params.model_type));
end
end
