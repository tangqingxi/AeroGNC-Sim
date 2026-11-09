function info = test_wtr_export()
%TEST_WTR_EXPORT WTR 导出契约、覆盖保护与原始结果安全

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = gnc_rate_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.5], ...
    'max_records', 30000));
result = aero_sim(scenario.model, scenario.cfg);

root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
signature = struct('scenario', scenario.name, 'tspan', scenario.cfg.tspan, ...
    'gains', scenario.gains, 'model_version', scenario.model.version);
options = struct('scenario', scenario.name, 'output_root', root, ...
    'signature', signature, 'gains', scenario.gains, 'seed', scenario.cfg.seed);

out = wtr_export(result, scenario.model, options);
check_that(isfile(out.mat_path) && isfile(out.wtr_path), '导出文件缺失');
matHash = wtr_sha256(out.mat_path);
wtrHash = wtr_sha256(out.wtr_path);

report = wtr_validate(out.wtr_path);
check_that(report.passed, 'WTR 独立校验失败');
check_that(strcmp(report.run_id, out.run_id), ...
    'WTR run_id 与导出结果不一致');
check_that(report.topic_count == out.topic_count, 'topic 数量不一致');

% 重复导出必须拒绝，且不改变已有文件
check_error(@() wtr_export(result, scenario.model, options), ...
    'wtr:export:exists', '重复导出未拒绝');
check_that(strcmp(wtr_sha256(out.mat_path), matHash), '失败导出修改了原始 .mat');
check_that(strcmp(wtr_sha256(out.wtr_path), wtrHash), '失败导出修改了已有 .wtr');

% 未启用发布日志的结果不能导出（避免用快照拼造历史）
noLog = scenario;
noLog.cfg.communication.log.enabled = false;
shortCfg = noLog.cfg;
shortCfg.tspan = [0, 0.3];
shortResult = aero_sim(noLog.model, shortCfg);
check_error(@() wtr_export(shortResult, noLog.model, ...
    struct('scenario', 'no_log', 'output_root', root, 'signature', signature)), ...
    'wtr:export:log', '无日志结果被错误导出');

info = struct('passed', true, 'summary', sprintf( ...
    'WTR %d topics、%.1f kB，SHA256 校验与覆盖保护通过', ...
    out.topic_count, dir(out.wtr_path).bytes / 1024));

end

% ===========================================================================
function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
