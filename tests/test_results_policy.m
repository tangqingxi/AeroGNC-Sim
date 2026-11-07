function info = test_results_policy()
%TEST_RESULTS_POLICY 验证 Results 容量策略：常量、单次计数、预警与超限拒绝
%
% 用缩放阈值（约 1 KB）触发与 10 GB 相同的代码路径，不生成真实 10 GB 文件。

p = aero_results_policy();
check_that(p.limit_bytes == 10e9, '默认上限 %.17g，期望 10e9', p.limit_bytes);
check_that(p.warn_bytes == 8e9, '默认预警 %.17g，期望 8e9', p.warn_bytes);
check_that(p.max_files == 5000 && p.warn_files == 4000, ...
    '默认文件预算 %d/%d，期望 4000/5000', p.warn_files, p.max_files);

dummy = struct('payload', 1:10);

% --- 合法保存：写入后用量等于文件大小 -------------------------------
rootA = make_temp_root();
cleanupA = onCleanup(@() remove_dir(rootA)); %#ok<NASGU>
policyA = aero_results_policy(struct('limit_bytes', 1e6, 'warn_bytes', 8e5, ...
    'max_files', 100, 'warn_files', 80));
savedA = aero_save_result(dummy, struct('path', 'ok.mat'), rootA, policyA);
sizeA = dir(savedA).bytes;
usageA = aero_results_usage(rootA);
check_that(isfile(savedA), '合法保存未生成文件 %s', savedA);
check_that(usageA.files == 1 && usageA.bytes == sizeA, ...
    '合法保存后用量 files=%d bytes=%.0f，期望 1/%d', usageA.files, usageA.bytes, sizeA);

% --- 新文件只计一次：现有 900 B + 结果 S，上限设为 900+S+50 ---------
rootB = make_temp_root();
cleanupB = onCleanup(@() remove_dir(rootB)); %#ok<NASGU>
xBytes = 900;
write_bytes(fullfile(rootB, 'existing.bin'), xBytes);
limitB = xBytes + sizeA + 50;
policyB = aero_results_policy(struct('limit_bytes', limitB, 'warn_bytes', limitB - 1, ...
    'max_files', 100, 'warn_files', 80));
savedB = aero_save_result(dummy, struct('path', 'once.mat'), rootB, policyB);
usageB = aero_results_usage(rootB);
check_that(isfile(savedB), '单次计数场景下保存失败');
check_that(usageB.bytes == xBytes + dir(savedB).bytes, ...
    '用量 %.0f 与“已有 + 新文件” %.0f 不一致（可能重复计数）', ...
    usageB.bytes, xBytes + dir(savedB).bytes);
check_that(usageB.bytes <= limitB, '写入后用量 %.0f 超过上限 %.0f', usageB.bytes, limitB);

% --- 8 GB 预警的缩放等价行为 ----------------------------------------
rootC = make_temp_root();
cleanupC = onCleanup(@() remove_dir(rootC)); %#ok<NASGU>
write_bytes(fullfile(rootC, 'existing.bin'), xBytes);
policyC = aero_results_policy(struct('limit_bytes', xBytes + sizeA + 50, ...
    'warn_bytes', xBytes + sizeA - 1, 'max_files', 100, 'warn_files', 80));
lastwarn('');
aero_save_result(dummy, struct('path', 'warn.mat'), rootC, policyC);
[msg, id] = lastwarn();
check_that(strcmp(id, 'aero:results:capacity'), ...
    '预警场景未触发 aero:results:capacity（收到 "%s"）', id);
check_that(contains(msg, '预警线'), '预警信息缺少说明: %s', msg);

% --- 10 GB 上限的缩放等价行为：拒绝且无残留 --------------------------
rootD = make_temp_root();
cleanupD = onCleanup(@() remove_dir(rootD)); %#ok<NASGU>
write_bytes(fullfile(rootD, 'existing.bin'), xBytes);
policyD = aero_results_policy(struct('limit_bytes', xBytes + sizeA - 1, ...
    'warn_bytes', xBytes, 'max_files', 100, 'warn_files', 80));
beforeD = list_files(rootD);
check_error(@() aero_save_result(dummy, struct('path', 'reject.mat'), rootD, policyD), ...
    'aero:results:limit', '超限拒绝');
check_that(~isfile(fullfile(rootD, 'reject.mat')), '超限保存仍生成了结果文件');
check_that(isequal(list_files(rootD), beforeD), '超限保存留下了残留文件');

% --- 内存模式不落盘 ---------------------------------------------------
rootE = make_temp_root();
cleanupE = onCleanup(@() remove_dir(rootE)); %#ok<NASGU>
cfg = aero_default_config();
cfg.tspan = [0, 0.05];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.results_root = rootE;
cfg.save.enabled = false;
aero_sim(scalar_integrator_model(), cfg);
check_that(aero_results_usage(rootE).files == 0, '内存模式仿真写出了文件');

% --- 持久化模式端到端 -------------------------------------------------
rootF = make_temp_root();
cleanupF = onCleanup(@() remove_dir(rootF)); %#ok<NASGU>
cfg.save.enabled = true;
cfg.save.path = 'run.mat';
cfg.results_root = rootF;
persisted = aero_sim(scalar_integrator_model(), cfg);
check_that(isfield(persisted, 'saved_path') && isfile(persisted.saved_path), ...
    '持久化模式未生成结果文件');
check_that(strncmp(persisted.saved_path, rootF, numel(rootF)), ...
    '结果文件写到了 Results 根目录之外');
check_that(strcmp(persisted.results_check.status, 'ok'), ...
    '持久化检查状态为 "%s"，期望 ok', persisted.results_check.status);

info = struct('passed', true, 'summary', sprintf( ...
    '默认 10e9/8e9/5000/4000；单次计数、缩放预警、超限拒绝、内存与持久化模式均验证（结果 %d B）', sizeA));

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

function remove_dir(path)
if isfolder(path)
    try
        rmdir(path, 's');
    catch
    end
end
end
