function usage = aero_results_usage(root, excludePath)
%AERO_RESULTS_USAGE 统计 Results 根目录下的文件数与字节数
%
% 输入：
%   root        - Results 根目录
%   excludePath - 可选；统计时排除的文件（用于排除本次运行自己的临时文件，
%                 避免临时文件被重复计数）
%
% 输出：
%   usage.root    - 根目录
%   usage.exists  - 根目录是否存在
%   usage.bytes    - 文件总字节数（int64 累计，避免大容量下精度损失）
%   usage.files    - 文件总数

usage = struct('root', '', 'exists', false, 'bytes', 0, 'files', 0);
if nargin < 1 || isempty(root)
    return;
end
usage.root = char(root);
if ~isfolder(usage.root)
    return;
end

entries = dir(fullfile(usage.root, '**', '*'));
entries = entries(~[entries.isdir]);

if nargin >= 2 && ~isempty(excludePath)
    excluded = char(excludePath);
    keep = true(numel(entries), 1);
    for k = 1:numel(entries)
        full = fullfile(entries(k).folder, entries(k).name);
        if same_path(full, excluded)
            keep(k) = false;
        end
    end
    entries = entries(keep);
end

usage.exists = true;
usage.files = numel(entries);
if isempty(entries)
    usage.bytes = 0;
else
    usage.bytes = double(sum(int64([entries.bytes])));
end

end

% ===========================================================================
function tf = same_path(a, b)
a = strrep(char(a), '/', filesep);
b = strrep(char(b), '/', filesep);
if ispc
    tf = strcmpi(a, b);
else
    tf = strcmp(a, b);
end
end
