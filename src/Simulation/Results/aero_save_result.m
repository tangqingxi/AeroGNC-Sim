function [savedPath, check] = aero_save_result(result, saveCfg, resultsRoot, policy)
%AERO_SAVE_RESULT 在容量与路径约束下持久化一次仿真结果
%
% 输入：
%   result      - aero_sim 返回的结果结构
%   saveCfg     - cfg.save 结构（enabled/path/estimated_bytes）
%   resultsRoot - Results 根目录；缺省为 <项目根>/results
%   policy      - 可选；容量策略，缺省 aero_results_policy()
%
% 输出：
%   savedPath   - 实际写入的绝对路径
%   check       - aero_results_check 的容量判定结果
%
% 安全约束：
%   1. 目标路径经 aero_results_path 规范化，必须位于 Results 根目录内，
%      并拒绝经过符号链接/Junction 的路径；
%   2. 目标文件已存在时默认拒绝，绝不覆盖已有科研证据；
%   3. 数据先写入同目录下的唯一临时文件，容量按“现有用量 + 临时文件大小”
%      计算，临时文件本身不计入现有用量，因此新文件只计一次；
%   4. 仅当容量与覆盖检查都通过后才原子地改名到目标路径；
%   5. 任何失败都会清理本次运行自己的临时文件，不触碰其他任务的文件。

if nargin < 4 || isempty(policy)
    policy = aero_results_policy();
end
if nargin < 3 || isempty(resultsRoot)
    resultsRoot = fullfile(aero_project_root(), 'results');
end
if ~isstruct(saveCfg) || ~isfield(saveCfg, 'path') || isempty(saveCfg.path)
    error('aero:results:path', 'save.enabled 为真时必须提供 save.path');
end

requested = char(saveCfg.path);
if isempty(regexp(requested, '\.[A-Za-z0-9]+$', 'once'))
    requested = [requested '.mat'];
end

root = char(resultsRoot);
target = aero_results_path(root, requested);

if isfolder(target)
    error('aero:results:path', '目标 "%s" 是目录，不能作为结果文件', target);
end
if isfile(target)
    error('aero:results:exists', '结果文件 "%s" 已存在，拒绝覆盖已有证据', target);
end

parent = fileparts(target);
if ~isfolder(parent)
    [ok, message] = mkdir(parent);
    if ~ok
        error('aero:results:io', '无法创建 Results 目录: %s', message);
    end
end
% 目录创建后再次校验，防止路径在创建期间经过新出现的链接
target = aero_results_path(root, requested);
parent = fileparts(target);

tmpPath = unique_temp_path(parent);
try
    save(tmpPath, 'result');
    entry = dir(tmpPath);
    incoming = 0;
    if ~isempty(entry)
        incoming = entry(1).bytes;
    end

    check = aero_results_check(root, policy, incoming, tmpPath);
    if strcmp(check.status, 'limit')
        error('aero:results:limit', '%s', check.message);
    end
    if any(strcmp(check.status, {'warning', 'file-warning', 'file-limit'}))
        warning('aero:results:capacity', '%s', check.message);
    end

    if isfile(target)
        error('aero:results:exists', '结果文件 "%s" 在写入期间出现，拒绝覆盖', target);
    end
    [ok, message] = movefile(tmpPath, target);
    if ~ok
        error('aero:results:io', '无法完成结果文件写入: %s', message);
    end
catch err
    if isfile(tmpPath)
        delete(tmpPath);
    end
    rethrow(err);
end

savedPath = target;

end

% ===========================================================================
function p = unique_temp_path(parent)
% 同目录唯一下临时文件：UUID 保证与其他运行互不冲突
try
    p = fullfile(parent, ['.aero-tmp-' char(java.util.UUID.randomUUID()) '.mat']);
catch
    p = [tempname(parent) '.mat'];
end
end
