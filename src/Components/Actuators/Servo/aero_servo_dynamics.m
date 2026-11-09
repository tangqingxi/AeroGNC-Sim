function ddelta = aero_servo_dynamics(delta, command, params)
%AERO_SERVO_DYNAMICS 通用舵机一阶连续动力学（与设备品牌无关）
%
% 输入：
%   delta   - 当前实际舵面偏角（n×1，rad）
%   command - 目标偏角（n×1，rad），Aircraft 锁存后传入
%   params  - 设备参数结构：
%       model_type       必须为 'first_order'（V0.1 唯一实现）
%       tau_s            一阶时间常数（标量或 n×1，s）
%       rate_limit_rad_s 最大角速率（标量或 n×1，rad/s；Inf 表示不限）
%       range_rad        生效行程 [min max]（rad）。装配层负责把“设备能力”
%                        与“装配限制”取交集后传入，本函数只使用最终值。
%
% 输出：
%   ddelta  - 实际偏角导数（rad/s）
%
% 模型：
%   δ̇ = clamp((δ_cmd − δ)/τ, −ω_max, +ω_max)
% 并在生效行程边界处阻止继续向外运动。位置限幅本身由 Aircraft 的状态
% 投影保证；此处只保证导数不会把状态推得更远。
%
% 该函数不读取设备文件、不判断品牌，仅使用传入参数。

require_first_order(params, '舵机');

tau = params.tau_s;
rateLimit = params.rate_limit_rad_s;
limits = params.range_rad;

delta = delta(:);
command = command(:);
if isscalar(tau)
    tau = repmat(tau, numel(delta), 1);
end
if isscalar(rateLimit)
    rateLimit = repmat(rateLimit, numel(delta), 1);
end
tau = tau(:);
rateLimit = rateLimit(:);

target = min(max(command, limits(1)), limits(2));
rate = (target - delta) ./ tau;

if any(isfinite(rateLimit))
    finite = isfinite(rateLimit);
    rate(finite) = min(max(rate(finite), -rateLimit(finite)), rateLimit(finite));
end

% 已在限位且目标继续向外时，导数置零，避免积分把状态推出机械行程
atUpper = delta >= limits(2) - 1e-12 & rate > 0;
atLower = delta <= limits(1) + 1e-12 & rate < 0;
rate(atUpper) = 0;
rate(atLower) = 0;

ddelta = rate;

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
