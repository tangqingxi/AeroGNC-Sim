function info = test_hover_hold()
%TEST_HOVER_HOLD 完整四级控制 + PINV 的悬停位置保持

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 4], ...
    'position_offset', [0.15; 0; 0.05], ...
    'max_records', 120000));

result = aero_sim(scenario.model, scenario.cfg);
layout = shc09_state_layout();
t = result.time;
position = result.state(:, layout.position);
quaternion = result.state(:, layout.quaternion);
errorNorm = sqrt(sum(position .^ 2, 2));

check_that(errorNorm(end) < 0.05, ...
    '悬停位置最终误差过大：%.4f m', errorNorm(end));
late = t >= 2;
check_that(max(errorNorm(late)) < 0.10, ...
    '2 s 后位置误差超限：%.4f m', max(errorNorm(late)));
check_that(abs(position(end, 1)) < 0.05 && abs(position(end, 3)) < 0.05, ...
    '悬停未回到目标附近');

tilt = 2 * acos(min(1, abs(quaternion(:, 1))));
check_that(max(tilt) < deg2rad(20), '姿态倾角超限：%.2f°', rad2deg(max(tilt)));

records = result.communication.log.records;
navRecords = records(cellfun(@(r) strcmp(r.name, 'navigation/state'), records));
check_that(~isempty(navRecords), '缺少 navigation/state 历史');
ideal = cellfun(@(r) r.data.ideal, navRecords);
check_that(all(ideal == 1), 'IdealState 标志未记录为 1');
names = result.communication.topic_names;
check_that(~any(startsWith(names, 'truth/')), '出现真值 Topic');
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

delta = result.state(:, layout.servo);
check_that(max(abs(delta(:))) <= max(abs(scenario.model.parameters.servo_effective_range_rad)) + 1e-9, ...
    '舵面偏角超出有效行程');

% 多速率声明：四级独立频率
periods = containers.Map();
for k = 1:numel(scenario.cfg.components)
    periods(scenario.cfg.components(k).name) = scenario.cfg.components(k).period;
end
check_that(abs(periods('position') - 0.01) < 1e-12, 'Position 周期不是 100 Hz');
check_that(abs(periods('velocity') - 0.01) < 1e-12, 'Velocity 周期不是 100 Hz');
check_that(abs(periods('attitude') - 0.005) < 1e-12, 'Attitude 周期不是 200 Hz');
check_that(abs(periods('rate') - 0.0025) < 1e-12, 'Rate 周期不是 400 Hz');

% 测量因果：导航输出不早于其采样时刻
for k = 1:numel(navRecords)
    check_that(navRecords{k}.available_time_s >= navRecords{k}.sample_time_s, ...
        '导航输出早于其测量采样时刻');
end

info = struct('passed', true, 'summary', sprintf( ...
    '悬停最终误差 %.4f m，2 s 后最大 %.4f m，最大倾角 %.2f°', ...
    errorNorm(end), max(errorNorm(late)), rad2deg(max(tilt))));

end
