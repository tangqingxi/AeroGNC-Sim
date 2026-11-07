function info = aero_env_info()
%AERO_ENV_INFO Record the execution environment for reproducibility.
%
%   info.matlab_version / matlab_release / computer describe MATLAB;
%   info.git_commit / git_dirty describe the DEV checkout. Git information is
%   best effort: when git is unavailable the fields are 'unknown'/false.

persistent cached
if ~isempty(cached)
    info = cached;
    return;
end

info = struct();
info.matlab_version = version;
info.matlab_release = version('-release');
info.computer = computer;
info.project_root = aero_project_root();
info.git_commit = 'unknown';
info.git_dirty = false;

root = info.project_root;
[status, out] = system(sprintf('git -C "%s" rev-parse HEAD', root));
if status == 0
    info.git_commit = strtrim(out);
    [status2, out2] = system(sprintf('git -C "%s" status --porcelain', root));
    if status2 == 0 && ~isempty(strtrim(out2))
        info.git_dirty = true;
    end
end

cached = info;

end
