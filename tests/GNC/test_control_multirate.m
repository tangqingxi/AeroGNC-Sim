function info = test_control_multirate()
%TEST_CONTROL_MULTIRATE 四级独立频率、同刻顺序与上游可用性

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
gains = control_defaults();
gains.rates.position_hz = 50;
gains.rates.velocity_hz = 100;
gains.rates.attitude_hz = 200;
gains.rates.rate_hz = 400;

scenario = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.2], ...
    'gains', gains, ...
    'max_records', 20000));
result = aero_sim(scenario.model, scenario.cfg);

counts = containers.Map();
for k = 1:numel(result.events)
    key = result.events(k).component;
    if isKey(counts, key)
        counts(key) = counts(key) + 1;
    else
        counts(key) = 1;
    end
end
check_that(counts('position') == 10, 'Position 事件数 %d ≠ 10', counts('position'));
check_that(counts('velocity') == 20, 'Velocity 事件数 %d ≠ 20', counts('velocity'));
check_that(counts('attitude') == 40, 'Attitude 事件数 %d ≠ 40', counts('attitude'));
check_that(counts('rate') == 80, 'Rate 事件数 %d ≠ 80', counts('rate'));

% 同一时刻顺序：position → velocity → attitude → rate → allocation
at = find(abs([result.events.time] - 0.02) < 1e-12);
seq = {result.events(at).component};
order = {'position', 'velocity', 'attitude', 'rate', 'allocation'};
positions = zeros(1, numel(order));
for k = 1:numel(order)
    positions(k) = find(strcmp(seq, order{k}), 1);
end
check_that(all(diff(positions) > 0), '同刻执行顺序错误：%s', strjoin(seq, '>'));

% 下游只能读取已可用上游输出：姿态首样本不早于速度首样本
records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
firstVelocity = min(cellfun(@(r) r.sample_time_s, ...
    records(strcmp(names, 'control/velocity_setpoint'))));
firstAttitude = min(cellfun(@(r) r.sample_time_s, ...
    records(strcmp(names, 'control/attitude_setpoint'))));
check_that(firstAttitude >= firstVelocity, '姿态在速度设定可用之前发布');

% 独立配置：Position 与 Velocity 不同频率时均可正常运行
check_that(gains.rates.position_hz ~= gains.rates.velocity_hz, ...
    '测试未使用不同频率');

info = struct('passed', true, 'summary', sprintf( ...
    '50/100/200/400 Hz 独立调度，同刻顺序 position→velocity→attitude→rate→allocation'));

end
