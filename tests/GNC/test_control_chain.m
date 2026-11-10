function info = test_control_chain()
%TEST_CONTROL_CHAIN 四级通过唯一 Topic 连接、时序因果与命令锁存独立

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.5], ...
    'position_offset', [0.05; 0; 0], ...
    'max_records', 40000));
result = aero_sim(scenario.model, scenario.cfg);

records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
required = {'control/velocity_setpoint', 'control/attitude_setpoint', ...
    'control/rate_setpoint', 'control/torque_setpoint', ...
    'control/allocation_status'};
for k = 1:numel(required)
    check_that(any(strcmp(names, required{k})), '缺少 Topic：%s', required{k});
end

% 首样本因果：下游不早于上游
firstTime = @(name) min(cellfun(@(r) r.sample_time_s, records(strcmp(names, name))));
check_that(firstTime('control/attitude_setpoint') >= firstTime('control/velocity_setpoint'), ...
    '姿态设定早于速度设定');
check_that(firstTime('control/rate_setpoint') >= firstTime('control/attitude_setpoint'), ...
    '速率设定早于姿态设定');
check_that(firstTime('control/torque_setpoint') >= firstTime('control/rate_setpoint'), ...
    '力矩早于速率设定');
check_that(firstTime('control/allocation_status') >= firstTime('control/torque_setpoint'), ...
    '分配结果早于力矩需求');

% 计算命令与锁存命令独立（至少一个时刻不同）
different = false;
for g = 1:numel(result.held)
    if ~isequaln(result.computed{g}, result.held{g})
        different = true;
        break;
    end
end
check_that(different, '计算命令与锁存命令始终相同');

% 分配器输出合法
status = records(strcmp(names, 'control/allocation_status'));
last = status{end}.data;
check_that(all(isfinite(last.delta)) && all(isfinite(last.residual)), ...
    '分配结果含非有限值');
check_that(all(abs(last.delta) <= max(abs(scenario.model.parameters.servo_effective_range_rad))), ...
    '分配舵角越界');

info = struct('passed', true, 'summary', sprintf( ...
    '四级 Topic 链与因果通过；%d 条日志、%d 个事件', numel(records), result.stats.events));

end
