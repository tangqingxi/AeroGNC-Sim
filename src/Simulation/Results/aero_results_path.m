function target = aero_results_path(resultsRoot, requestedPath)
%AERO_RESULTS_PATH 规范化并校验结果文件目标路径
%
% 输入：
%   resultsRoot   - Results 根目录（相对路径按当前工作目录解析）
%   requestedPath - 期望的保存路径；相对路径按根目录解析，绝对路径必须位于根目录内
%
% 输出：
%   target        - 规范化后的绝对路径（不创建任何目录）
%
% 校验（任一失败即报错，不静默修补）：
%   1. 词法规范化：折叠 '.' 与 '..'，统一分隔符，拒绝 UNC 路径；
%   2. 目录归属：必须落在根目录边界内（root + 分隔符），排除 "results2" 这类同前缀误判；
%   3. 链接检查：已存在的路径分量不得是符号链接或 Windows Junction（重解析点），
%      避免实际写入位置逃出 Results 根目录。
%
% 该函数不做任何文件系统写入，因此非法路径不会在根目录外留下空目录。

if nargin < 1 || isempty(resultsRoot)
    error('aero:results:path', 'Results 根目录不能为空');
end
if nargin < 2 || isempty(requestedPath)
    error('aero:results:path', '结果保存路径不能为空');
end

root = char(resultsRoot);
requested = char(requestedPath);

% 根目录本身也必须是绝对路径，避免相对路径在运行目录变化后指错位置
rootAbs = absolute_path(root);
rootNorm = normalize_lexical(rootAbs);
check_no_links(rootNorm);

targetAbs = absolute_path_under(rootNorm, requested);
targetNorm = normalize_lexical(targetAbs);

% 目录边界判定：必须严格位于根目录之下，且不能等于根目录本身
rootPrefix = [rootNorm filesep];
inside = strncmpi(targetNorm, rootPrefix, numel(rootPrefix)) && ...
    ~strcmpi(targetNorm, rootNorm);
if ~inside
    error('aero:results:outside', ...
        '保存路径 "%s" 不在 Results 根目录 "%s" 内', targetNorm, rootNorm);
end

check_no_links(fileparts(targetNorm));

target = targetNorm;

end

% ===========================================================================
function p = absolute_path(p)
if is_absolute(p)
    p = char(p);
else
    p = fullfile(pwd, p);
end
end

function p = absolute_path_under(root, requested)
if is_absolute(requested)
    p = char(requested);
else
    p = fullfile(root, requested);
end
end

function tf = is_absolute(p)
p = char(p);
tf = ~isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once')) || ...
    strncmp(p, '\\', 2) || ...
    (~isempty(p) && (p(1) == '/' || p(1) == '\'));
end

% ===========================================================================
function p = normalize_lexical(p)
% 纯字符串规范化：折叠 '.'/'..'，不访问文件系统
p = strrep(p, '/', filesep);

if numel(p) >= 2 && strcmp(p(1:2), '\\')
    error('aero:results:path', '不支持 UNC/网络路径: %s', p);
end

if ispc
    if numel(p) < 2 || p(2) ~= ':'
        error('aero:results:path', '需要绝对路径: %s', p);
    end
    drive = p(1:2);
    rest = p(3:end);
else
    drive = '';
    rest = p;
end

parts = regexp(rest, '[\\/]+', 'split');
parts = parts(~cellfun(@isempty, parts));

out = {};
for k = 1:numel(parts)
    part = parts{k};
    if strcmp(part, '.')
        continue;
    end
    if strcmp(part, '..')
        if isempty(out)
            error('aero:results:path', '路径越过根目录: %s', p);
        end
        out(end) = []; %#ok<AGROW>
        continue;
    end
    out{end + 1} = part; %#ok<AGROW>
end

p = [drive filesep strjoin(out, filesep)];
end

% ===========================================================================
function check_no_links(p)
% 逐级检查已存在的路径分量；遇到符号链接/Junction 直接拒绝写入。
% 不存在的分量（未来的 Results 子目录）不会被检查，但父目录已全部通过。
if ~ispc
    check_canonical(p);
    return;
end

parts = path_parts(p);
current = parts{1};
for k = 2:numel(parts)
    current = fullfile(current, parts{k});
    if ~isfolder(current) && ~isfile(current)
        break;
    end
    if is_reparse_point(current)
        error('aero:results:link', ...
            '路径分量 "%s" 是符号链接或 Junction，拒绝写入 Results', current);
    end
end
end

function parts = path_parts(p)
% 拆分为 {'E:', 'AeroGNC-Sim', ...}，第一项为盘符或根
if ispc
    parts = [{p(1:2)}, regexp(p(4:end), '[\\/]+', 'split')];
    parts = parts(~cellfun(@isempty, parts));
else
    parts = [{filesep}, regexp(p(2:end), '/+', 'split')];
    parts = parts(~cellfun(@isempty, parts));
end
end

function tf = is_reparse_point(p)
% Windows 上 Junction 与符号链接都带 ReparsePoint 属性；.NET 可稳定识别。
% 常量 1024 = System.IO.FileAttributes.ReparsePoint，避免依赖枚举类型转换。
persistent reparse_flag
if isempty(reparse_flag)
    reparse_flag = 1024;
end
try
    attrs = double(System.IO.File.GetAttributes(p));
catch
    tf = false;
    return;
end
tf = bitand(attrs, reparse_flag) ~= 0;
end

function check_canonical(p)
% 非 Windows 平台：用 Java 真实路径与词法路径比较，识别符号链接
try
    canonical = char(java.io.File(p).getCanonicalPath());
catch err
    error('aero:results:link', '无法解析路径 "%s" 的真实位置: %s', p, err.message);
end
if ~strcmpi(canonical, p)
    error('aero:results:link', ...
        '路径 "%s" 解析为 "%s"，可能经过符号链接，拒绝写入', p, canonical);
end
end
