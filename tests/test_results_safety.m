function info = test_results_safety()
%TEST_RESULTS_SAFETY 验证 Results 路径隔离、覆盖保护与临时文件清理
%
% 覆盖：已存在文件拒绝覆盖、哈希不变、相对路径穿越、绝对路径越界、
%       同前缀误判、符号链接/Junction 拒绝、失败无残留、其他任务临时文件不被误删。

dummy = struct('payload', 1:10);
policy = aero_results_policy(struct('limit_bytes', 1e6, 'warn_bytes', 8e5, ...
    'max_files', 100, 'warn_files', 80));

root = make_temp_root();
cleanup = onCleanup(@() remove_dir(root)); %#ok<NASGU>

% --- 合法保存后拒绝覆盖，原文件哈希不变 ------------------------------
saved = aero_save_result(dummy, struct('path', 'evidence.mat'), root, policy);
beforeHash = sha256_file(saved);
beforeFiles = list_files(root);
check_error(@() aero_save_result(dummy, struct('path', 'evidence.mat'), root, policy), ...
    'aero:results:exists', '覆盖保护');
check_that(strcmp(sha256_file(saved), beforeHash), '拒绝覆盖后原文件哈希发生变化');
check_that(isequal(list_files(root), beforeFiles), '拒绝覆盖后出现残留文件');

% --- 相对路径穿越 ------------------------------------------------------
parent = fileparts(root);
check_error(@() aero_save_result(dummy, struct('path', '../escape.mat'), root, policy), ...
    'aero:results:outside', '相对路径穿越');
check_that(~isfile(fullfile(parent, 'escape.mat')), '穿越写入在根目录外生成了文件');
check_error(@() aero_save_result(dummy, struct('path', '../deep/a/b.mat'), root, policy), ...
    'aero:results:outside', '带新建目录的穿越');
check_that(~isfolder(fullfile(parent, 'deep')), '穿越路径在根目录外创建了目录');

% --- 绝对路径越界 ------------------------------------------------------
outsideFile = fullfile(tempdir(), 'aero_outside_result.mat');
check_error(@() aero_save_result(dummy, struct('path', outsideFile), root, policy), ...
    'aero:results:outside', '绝对路径越界');
check_that(~isfile(outsideFile), '越界写入生成了根目录外文件');

% --- 仅同前缀的路径不能被误判为根目录内 -------------------------------
lookalike = [root '-evil'];
check_error(@() aero_save_result(dummy, struct('path', fullfile(lookalike, 'x.mat')), root, policy), ...
    'aero:results:outside', '同前缀路径');

% --- 失败时不误删其他任务的临时文件 -----------------------------------
stale = fullfile(root, '.aero-tmp-other-run.mat');
write_bytes(stale, 32);
check_error(@() aero_save_result(dummy, struct('path', 'evidence.mat'), root, policy), ...
    'aero:results:exists', '覆盖保护（含临时文件）');
check_that(isfile(stale), '其他任务的临时文件被误删');

% --- 失败后不留下本次运行的临时文件 -----------------------------------
leftovers = dir(fullfile(root, '.aero-tmp-*.mat'));
check_that(numel(leftovers) == 1, '失败后临时文件数量为 %d，期望仅保留其他任务的 1 个', ...
    numel(leftovers));

% --- Junction / 符号链接必须拒绝 --------------------------------------
linkRoot = make_temp_root();
cleanupLink = onCleanup(@() remove_dir(linkRoot)); %#ok<NASGU>
outsideDir = make_temp_root();
cleanupOutside = onCleanup(@() remove_dir(outsideDir)); %#ok<NASGU>
linkPath = fullfile(linkRoot, 'link');
[status, cmdOut] = system(sprintf('cmd /c mklink /J "%s" "%s"', linkPath, outsideDir));
if status ~= 0
    info = struct('passed', true, 'skipped', true, 'summary', sprintf( ...
        'Junction 创建不可用（%s），链接拒绝路径未被执行验证', strtrim(cmdOut)));
    return;
end
check_that(isfolder(linkPath), 'Junction 未创建成功');
check_error(@() aero_save_result(dummy, struct('path', 'link/x.mat'), linkRoot, policy), ...
    'aero:results:link', 'Junction 路径');
check_that(~isfile(fullfile(outsideDir, 'x.mat')), 'Junction 写入逃逸到根目录外');
remove_junction(linkPath);
check_that(~isfolder(linkPath), 'Junction 清理失败');

info = struct('passed', true, 'summary', ...
    '覆盖保护/哈希不变、路径穿越、绝对越界、同前缀、Junction 拒绝、失败无残留均通过');

end

% ===========================================================================
function root = make_temp_root()
root = tempname();
mkdir(root);
end

function write_bytes(path, n)
fid = fopen(path, 'w');
check_that(fid >= 0, '无法创建 %s', path);
fwrite(fid, zeros(1, n, 'uint8'), 'uint8');
fclose(fid);
end

function names = list_files(root)
entries = dir(fullfile(root, '**', '*'));
entries = entries(~[entries.isdir]);
names = sort({entries.name});
end

function h = sha256_file(path)
fid = fopen(path, 'r');
check_that(fid >= 0, '无法读取 %s', path);
bytes = fread(fid, inf, 'uint8=>uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
digest = md.digest(bytes);
h = sprintf('%02x', typecast(digest, 'uint8'));
end

function remove_junction(path)
if isfolder(path)
    system(sprintf('cmd /c rmdir "%s"', path));
end
end

function remove_dir(path)
if isfolder(path)
    try
        rmdir(path, 's');
    catch
    end
end
end
