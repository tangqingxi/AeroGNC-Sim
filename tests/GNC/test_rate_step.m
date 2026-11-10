function info = test_rate_step()
%TEST_RATE_STEP 角速度闭环方向、跟踪与命令锁存独立性

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = rate_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 1.2], ...
    'max_records', 60000));

result = aero_sim(scenario.model, scenario.cfg);
layout = shc09_state_layout();
t = result.time;
omega = result.state(:, layout.omega);

check_that(result.stats.events > 1000, '事件数异常：%d', result.stats.events);
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

beforeStep = nearest_index(t, 0.3);
check_that(max(abs(omega(beforeStep, :))) < 0.02, ...
    '阶跃前存在非零角速度：%s', mat2str(omega(beforeStep, :), 4));

atTracking = nearest_index(t, 1.0);
check_that(abs(omega(atTracking, 1) - 0.5) < 0.05, ...
    '滚转角速度跟踪误差过大：%.4f', omega(atTracking, 1));
check_that(abs(omega(atTracking, 2)) < 0.05, '滚转阶跃引起过大俯仰耦合');
check_that(abs(omega(atTracking, 3)) < 0.08, '滚转阶跃引起过大偏航耦合');

different = false;
for g = 1:numel(result.held)
    if ~isequaln(result.computed{g}, result.held{g})
        different = true;
        break;
    end
end
check_that(different, '计算命令与锁存命令始终相同，锁存未生效');

root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
out = wtr_export(result, scenario.model, struct('output_root', root));
check_that(isfile(out.mat_path) && isfile(out.wtr_path), '导出文件缺失');
report = wtr_validate(out.wtr_path);
check_that(report.passed && report.topic_count >= 12, ...
    'WTR 校验失败或 topic 数量不足');

info = struct('passed', true, 'summary', sprintf( ...
    '角速度跟踪 %.3f rad/s（目标 0.5），yaw %.3f；WTR %d topics 校验通过', ...
    omega(atTracking, 1), omega(atTracking, 3), report.topic_count));

end

% ===========================================================================
function index = nearest_index(t, target)
[~, index] = min(abs(t - target));
end

function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
