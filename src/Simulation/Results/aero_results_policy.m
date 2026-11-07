function p = aero_results_policy(overrides)
%AERO_RESULTS_POLICY Results 存储策略
%
% 默认值：
%   limit_bytes = 10e9  （10 GB，按十进制解释，比 10 GiB 更保守）
%   warn_bytes  = 8e9   （80% 预警）
%   max_files   = 5000  （临时文件数预算，不是已冻结的工程标准）
%   warn_files  = 4000  （达到 80% 时提示复核）
%
% 字节上限会阻止新的持久化写入；文件数只提示复核，不阻断运行。
%
% 用法：
%   p = aero_results_policy();
%   p = aero_results_policy(struct('limit_bytes', 1000, 'warn_bytes', 800));
%
% 覆盖参数仅供测试使用缩放阈值触发与 10 GB 完全相同的代码路径。

p = struct();
p.limit_bytes = 10e9;
p.warn_bytes = 8e9;
p.max_files = 5000;
p.warn_files = 4000;

if nargin >= 1 && ~isempty(overrides)
    if ~isstruct(overrides) || ~isscalar(overrides)
        error('aero:results:policy', '策略覆盖值必须是标量结构体');
    end
    names = fieldnames(overrides);
    for k = 1:numel(names)
        if ~isfield(p, names{k})
            error('aero:results:policy', '未知的策略字段 "%s"', names{k});
        end
        p.(names{k}) = overrides.(names{k});
    end
end

if ~isnumeric(p.limit_bytes) || ~isscalar(p.limit_bytes) || ~isfinite(p.limit_bytes) ...
        || p.limit_bytes <= 0
    error('aero:results:policy', 'limit_bytes 必须是正有限标量');
end
if ~isnumeric(p.warn_bytes) || ~isscalar(p.warn_bytes) || ~isfinite(p.warn_bytes) ...
        || p.warn_bytes <= 0 || p.warn_bytes >= p.limit_bytes
    error('aero:results:policy', 'warn_bytes 必须为正且小于 limit_bytes');
end
if ~isnumeric(p.max_files) || ~isscalar(p.max_files) || p.max_files < 1 ...
        || p.max_files ~= fix(p.max_files)
    error('aero:results:policy', 'max_files 必须是正整数');
end
if ~isnumeric(p.warn_files) || ~isscalar(p.warn_files) || p.warn_files < 1 ...
        || p.warn_files >= p.max_files || p.warn_files ~= fix(p.warn_files)
    error('aero:results:policy', 'warn_files 必须是小于 max_files 的正整数');
end

end
