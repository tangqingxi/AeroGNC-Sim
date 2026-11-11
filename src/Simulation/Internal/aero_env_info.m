function info = aero_env_info()
%AERO_ENV_INFO 记录本次运行的环境信息（用于复现）
%
% 输出：
%   matlab_version / matlab_release / computer - MATLAB 与平台
%   project_root                              - 项目根目录
%   git_commit / git_dirty                    - DEV 提交与工作区状态（尽力获取）
%
% 说明：git 信息用于把结果关联到具体提交；git 不可用时字段为 'unknown'/false，
% 不阻塞仿真。仅缓存 MATLAB 与平台信息；每次调用重新查询 Git 来源。

persistent cached
if isempty(cached)
    cached = struct('matlab_version', version, ...
        'matlab_release', version('-release'), 'computer', computer);
end

info = cached;
info.project_root = aero_project_root();
info.git_commit = 'unknown';
info.git_dirty = false;

root = info.project_root;
[status, out] = system(sprintf('git -C "%s" rev-parse HEAD', root));
if status == 0
    info.git_commit = strtrim(out);
end
[status2, out2] = system(sprintf('git -C "%s" status --porcelain', root));
if status2 == 0 && ~isempty(strtrim(out2))
    info.git_dirty = true;
end

end
