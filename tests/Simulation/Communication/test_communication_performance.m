function info = test_communication_performance()
%TEST_COMMUNICATION_PERFORMANCE 高频多 Topic 的轻量计时与日志容量检查

model = struct('name', 'perf-probe', 'x0', 0, ...
    'dynamics', @(t, x, u, env) u); %#ok<INUSD>

[cfgOff, ~] = perf_config(false);
resOff = aero_sim(model, cfgOff);
[cfgOn, ~] = perf_config(true);
resOn = aero_sim(model, cfgOn);

check_that(resOff.stats.events > 1500, ...
    '高频事件数 %d 低于预期（配置可能未生效）', resOff.stats.events);
check_that(resOff.stats.wall_time_s < 30, ...
    '高频仿真耗时 %.2f s 超过 30 s', resOff.stats.wall_time_s);
check_that(isequaln(resOff.state, resOn.state), ...
    '开启高频日志改变了仿真轨迹');

logSnap = resOn.communication.log;
check_that(logSnap.enabled, '高频日志未启用');
check_that(numel(logSnap.records) <= logSnap.max_records, ...
    '日志条数 %d 超过容量 %d', numel(logSnap.records), logSnap.max_records);
check_that(numel(logSnap.records) == logSnap.max_records, ...
    '日志未填满设定容量（%d/%d）', numel(logSnap.records), logSnap.max_records);
check_that(logSnap.dropped > 0, '超出容量的发布未被计数丢弃');

info = struct('passed', true, 'summary', sprintf( ...
    '%d 个事件、%.2f s；日志上限 %d 条、丢弃 %d 条，轨迹与关闭日志一致', ...
    resOff.stats.events, resOff.stats.wall_time_s, ...
    logSnap.max_records, logSnap.dropped));

end

% ===========================================================================
function [cfg, capture] = perf_config(logEnabled)
sensorSpec = aero_topic_spec({'value', 'sample_time_s'}, {[1 1], [1 1]}, ...
    {'m', 's'}, struct('max_age_s', 0.05));
valueSpec = aero_topic_spec({'value'}, {[1 1]}, {'-'});

cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.solver = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
cfg.communication.enabled = true;
cfg.communication.topics = { ...
    struct('name', 'sensor/baro', 'spec', sensorSpec), ...
    struct('name', 'nav/state', 'spec', valueSpec), ...
    struct('name', 'command/computed', 'spec', valueSpec), ...
    struct('name', 'command/held', 'spec', valueSpec)};
cfg.communication.command_topic = 'command/computed';
cfg.communication.held_command_topic = 'command/held';
cfg.communication.log.enabled = logEnabled;
cfg.communication.log.topics = {'sensor/baro', 'command/computed', 'command/held'};
cfg.communication.log.max_records = 500;

capture = containers.Map('KeyType', 'char', 'ValueType', 'any');
cfg.components = struct( ...
    'name', {'baro', 'nav', 'control', 'latch', 'guard'}, ...
    'kind', {'sensor', 'periodic', 'periodic', 'periodic', 'periodic'}, ...
    'period', {0.0025, 0.005, 0.004, 0.01, 0.01}, ...
    'priority', {10, 20, 30, 40, 60}, ...
    'delay', {0.005, 0, 0, 0, 0}, ...
    'latches', {false, false, false, true, false}, ...
    'topic', {'sensor/baro', '', '', '', ''}, ...
    'sample', {@(t, x) x(1) + 10, [], [], [], []}, ...
    'step', {[], @nav_step, @control_step, @latch_step, ...
             @(t, bus) guard_step(t, bus, capture)});
end

function bus = nav_step(t, bus)
[m, status] = bus.topics.tryRead('sensor/baro', t);
if strcmp(status, 'ok')
    bus.topics.publish('nav/state', struct('value', m.data.value), ...
        m.data.sample_time_s, t, t, 'navigation');
end
end

function bus = control_step(t, bus)
[msg, status] = bus.topics.tryRead('nav/state', t);
if strcmp(status, 'ok')
    bus.command = -0.1 * msg.data.value;
end
end

function bus = latch_step(t, bus) %#ok<INUSD>
bus.held_command = bus.command;
end

function bus = guard_step(t, bus, capture) %#ok<INUSD>
if ~isKey(capture, 'bus')
    capture('bus') = bus.topics;
end
end
