function [savedPath, check] = aero_save_result(result, saveCfg, resultsRoot, policy)
%AERO_SAVE_RESULT Persist a simulation result under the results policy.
%
%   Saving is explicit (cfg.save.enabled) and confined to the results root so
%   that simulation output never enters the source tree. Data is written to a
%   temporary file first; if the projected active-results usage would exceed
%   policy.limit_bytes the temporary file is removed and an error is raised,
%   leaving existing evidence untouched. Warning thresholds emit a warning
%   but still persist.

if nargin < 4 || isempty(policy)
    policy = aero_results_policy();
end
if nargin < 3 || isempty(resultsRoot)
    resultsRoot = fullfile(aero_project_root(), 'results');
end
if ~isstruct(saveCfg) || ~isfield(saveCfg, 'path') || isempty(saveCfg.path)
    error('aero:results:path', 'save.path is required when save.enabled is true');
end

root = char(resultsRoot);
target = char(saveCfg.path);
if ~is_absolute_path(target)
    target = fullfile(root, target);
end
if isempty(regexp(target, '\.[A-Za-z0-9]+$', 'once'))
    target = [target '.mat'];
end
rootCmp = normalize_case(root);
targetCmp = normalize_case(target);
if strncmp(targetCmp, [rootCmp filesep], numel(rootCmp) + 1) == false
    error('aero:results:outside', ...
        'save.path must be inside the results root "%s"', root);
end

parent = fileparts(target);
if ~isfolder(parent)
    [ok, msg] = mkdir(parent);
    if ~ok
        error('aero:results:io', 'cannot create results directory: %s', msg);
    end
end

tmpPath = [target '.tmp'];
try
    save(tmpPath, 'result');
catch err
    if isfile(tmpPath)
        delete(tmpPath);
    end
    error('aero:results:io', 'cannot write result: %s', err.message);
end

entry = dir(tmpPath);
incoming = 0;
if ~isempty(entry)
    incoming = entry(1).bytes;
end
check = aero_results_check(root, policy, incoming);
if strcmp(check.status, 'limit')
    delete(tmpPath);
    error('aero:results:limit', '%s', check.message);
end
if any(strcmp(check.status, {'warning', 'file-warning', 'file-limit'}))
    warning('aero:results:capacity', '%s', check.message);
end

[ok, msg] = movefile(tmpPath, target, 'f');
if ~ok
    if isfile(tmpPath)
        delete(tmpPath);
    end
    error('aero:results:io', 'cannot finalize result file: %s', msg);
end

savedPath = target;

end

% ===========================================================================
function tf = is_absolute_path(p)
tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || ...
    strncmp(p, '\\\\', 2) || ...
    (~isempty(p) && (p(1) == '/' || p(1) == '\'));
end

function p = normalize_case(p)
p = strrep(p, '/', filesep);
if ispc
    p = lower(p);
end
end
