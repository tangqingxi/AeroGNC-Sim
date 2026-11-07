function check = aero_results_check(root, policy, incoming_bytes, excludePath)
%AERO_RESULTS_CHECK 评估一次写入是否符合 Results 容量策略
%
% 输入：
%   root          - Results 根目录
%   policy        - aero_results_policy 返回的策略结构
%   incoming_bytes- 即将新增的字节数（本次写入的文件大小）
%   excludePath   - 可选；统计时排除的文件（通常是本次运行的临时文件）
%
% 输出 check：
%   status          ok | warning | file-warning | file-limit | limit
%   message         中文说明
%   usage           统计结果（已排除 excludePath）
%   projected_bytes 现有用量 + 新增字节（新文件只计一次）
%   policy          生效策略
%
% 判定顺序：字节超限 > 文件数达到临时预算 > 字节预警 > 文件数 80% 提示。

if nargin < 2 || isempty(policy)
    policy = aero_results_policy();
end
if nargin < 3 || isempty(incoming_bytes)
    incoming_bytes = 0;
end
if ~isnumeric(incoming_bytes) || ~isscalar(incoming_bytes) ...
        || ~isfinite(incoming_bytes) || incoming_bytes < 0
    error('aero:results:policy', 'incoming_bytes 必须是非负有限标量');
end

if nargin >= 4
    usage = aero_results_usage(root, excludePath);
else
    usage = aero_results_usage(root);
end
projected = usage.bytes + incoming_bytes;

if projected > policy.limit_bytes
    status = 'limit';
    message = sprintf(['写入后活跃 Results 约 %.3f GB，超过 %.3f GB 上限（根目录：%s），' ...
        '已拒绝本次持久化'], projected / 1e9, policy.limit_bytes / 1e9, usage.root);
elseif usage.files >= policy.max_files
    status = 'file-limit';
    message = sprintf(['Results 已有 %d 个文件，达到临时文件预算 %d；' ...
        '请复核存储方案后再继续写入'], usage.files, policy.max_files);
elseif projected >= policy.warn_bytes
    status = 'warning';
    message = sprintf('写入后活跃 Results 约 %.3f GB，已达到 %.3f GB 预警线', ...
        projected / 1e9, policy.warn_bytes / 1e9);
elseif usage.files >= policy.warn_files
    status = 'file-warning';
    message = sprintf('Results 文件数 %d 达到临时预算 %d 的 80%%，请复核', ...
        usage.files, policy.max_files);
else
    status = 'ok';
    message = '';
end

check = struct();
check.status = status;
check.message = message;
check.usage = usage;
check.projected_bytes = projected;
check.policy = policy;

end
