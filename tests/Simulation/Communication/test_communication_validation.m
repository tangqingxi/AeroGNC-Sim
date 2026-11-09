function info = test_communication_validation()
%TEST_COMMUNICATION_VALIDATION 结果快照、持久化、日志与边界一致性

% ---- 1. 结果快照为普通值结构，可保存/加载 -------------------------
[cfg, capture] = build_config(0.2, false);
model = probe_model();
result = aero_sim(model, cfg);
snap = result.communication;

check_that(~isfield(snap, 'bus'), '快照仍包含 Bus 句柄');
check_that(strcmp(snap.schema, 'aero-comm-snapshot/1'), '快照 schema 错误');
check_that(numel(snap.messages) == numel(snap.topic_names), '消息数量与 Topic 数不一致');
check_that(~snap.log.enabled, '默认日志未关闭');
check_that(iscell(snap.messages), 'messages 不是普通 cell');
check_that(~isempty(capture('bus')), '测试未捕获运行时总线');

matFile = [tempname(), '.mat'];
cleanup = onCleanup(@() delete_if_exists(matFile)); %#ok<NASGU>
save(matFile, 'result');
beforeCount = result.communication.publish_count;
beforeNames = result.communication.topic_names;
baroBefore = find_message(result.communication.messages, 'sensor/baro');
clear result
loaded = load(matFile);
restored = loaded.result.communication;
check_that(strcmp(restored.schema, 'aero-comm-snapshot/1'), '重新加载后 schema 变化');
check_that(isequal(restored.topic_names, beforeNames), '重新加载后 Topic 列表变化');
check_that(restored.publish_count == beforeCount, '重新加载后发布计数变化');
check_that(~isfield(restored, 'bus'), '重新加载后出现 Bus 句柄');
baroAfter = find_message(restored.messages, 'sensor/baro');
check_that(abs(baroAfter.sample_time_s - baroBefore.sample_time_s) < 1e-15, ...
    '重新加载后采样时间变化');
check_that(abs(baroAfter.available_time_s - baroBefore.available_time_s) < 1e-15, ...
    '重新加载后可用时间变化');
check_that(isequal(baroAfter.data, baroBefore.data), '重新加载后载荷变化');

% ---- 2. 发布日志：只记录实际发布，且不改变轨迹 ----------------------
[cfgOff, ~] = build_config(0.5, false);
resOff = aero_sim(probe_model(), cfgOff);
[cfgOn, ~] = build_config(0.5, true);
resOn = aero_sim(probe_model(), cfgOn);

check_that(resOn.communication.log.enabled, '日志未启用');
check_that(isequaln(resOff.state, resOn.state), '开启日志改变了仿真轨迹');
check_that(isequal(resOff.time, resOn.time), '开启日志改变了时间序列');
check_that(isequal(resOff.computed, resOn.computed), '开启日志改变了计算命令');

records = resOn.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
expectedPublishes = sum(strcmp({resOn.events.component}, 'baro') & ...
    strcmp({resOn.events.action}, 'publish'));
check_that(sum(strcmp(names, 'sensor/baro')) == expectedPublishes, ...
    '传感器日志条数与实际发布事件不一致');
check_that(sum(strcmp(names, 'command/computed')) > 0, '命令日志缺失');
check_that(~any(strcmp(names, 'command/held')), '未选择的 Topic 被记录');

baroRecords = records(strcmp(names, 'sensor/baro'));
for k = 1:numel(baroRecords)
    r = baroRecords{k};
    check_that(abs(r.available_time_s - (r.sample_time_s + 0.01)) < 1e-12, ...
        '日志记录的可用时间不等于采样时间 + 延迟');
    check_that(isfield(r, 'sequence') && isfield(r, 'topic_sequence'), ...
        '日志缺少序号字段');
end
baroSeq = cellfun(@(r) double(r.topic_sequence), baroRecords);
check_that(all(diff(baroSeq) > 0), '同一 Topic 的日志序号非递增');

% ---- 3. 日志容量上限与真值拒绝 --------------------------------------
bus = aero_topic_bus();
bus.register('x/value', aero_topic_spec({'value'}, {[1 1]}, {'-'}));
bus.enableLog({'x/value'}, 3);
for k = 1:5
    bus.publish('x/value', struct('value', k), k, k, k, 'test');
end
logSnap = bus.logSnapshot();
check_that(numel(logSnap.records) == 3, '日志未按容量截断（%d）', numel(logSnap.records));
check_that(logSnap.dropped == 2, '丢弃计数错误（%d）', logSnap.dropped);

truthBus = aero_topic_bus();
truthBus.register('truth/state', aero_topic_spec({'x'}, {[1 1]}, {'-'}, ...
    struct('allow_ground_truth', true)));
check_error(@() truthBus.enableLog({'truth/state'}, 10), ...
    'aero:bus:logGroundTruth', '真值 Topic 进入日志');

% ---- 4. 边界检查 ----------------------------------------------------
dup = cfg;
dup.communication.held_command_topic = dup.communication.command_topic;
check_error(@() aero_sim(probe_model(), dup), 'aero:comm:duplicateCommandTopic', ...
    '命令与锁存命令使用同一 Topic');

conflict = cfg;
conflict.components(1).topic = 'command/computed';
check_error(@() aero_sim(probe_model(), conflict), 'aero:comm:topicConflict', ...
    '传感器 Topic 与命令 Topic 冲突');

doubleSensor = cfg;
extra = doubleSensor.components(1);
extra.name = 'baro2';
doubleSensor.components(end + 1) = extra;
check_error(@() aero_sim(probe_model(), doubleSensor), 'aero:comm:topicConflict', ...
    '两个传感器使用同一 Topic');

replaced = cfg;
evil = replaced.components(2);
evil.name = 'evil';
evil.step = @(t, bus) replace_bus(bus);
evil.sample = [];
replaced.components(end + 1) = evil;
check_error(@() aero_sim(probe_model(), replaced), 'aero:comm:busReplaced', ...
    '运行时替换唯一总线');

% ---- 5. 结果测量记录是唯一权威格式 --------------------------------
check_that(~isempty(resOn.measurements), 'result.measurements 记录缺失');
check_that(isfield(resOn.measurements{1}, 'available_time'), ...
    '测量记录缺少 available_time');

info = struct('passed', true, 'summary', sprintf( ...
    '快照可保存/加载；日志 %d 条与实际发布一致且不改变轨迹；容量/真值/冲突/唯一性检查通过', ...
    numel(records)));

end

% ===========================================================================
function model = probe_model()
model = struct('name', 'validation-probe', 'x0', 0, ...
    'dynamics', @(t, x, u, env) u); %#ok<INUSD>
end

function [cfg, capture] = build_config(tspan, logEnabled)
sensorSpec = aero_topic_spec({'value', 'sample_time_s'}, {[1 1], [1 1]}, ...
    {'m', 's'}, struct('max_age_s', 0.05));
valueSpec = aero_topic_spec({'value'}, {[1 1]}, {'-'});

cfg = aero_default_config();
cfg.tspan = [0, tspan];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.solver = struct('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', Inf, 'InitialStep', []);
cfg.communication.topics = { ...
    struct('name', 'sensor/baro', 'spec', sensorSpec), ...
    struct('name', 'command/computed', 'spec', valueSpec), ...
    struct('name', 'command/held', 'spec', valueSpec)};
cfg.communication.command_topic = 'command/computed';
cfg.communication.held_command_topic = 'command/held';
cfg.communication.log.enabled = logEnabled;
cfg.communication.log.topics = {'sensor/baro', 'command/computed'};
cfg.communication.log.max_records = 5000;

capture = containers.Map('KeyType', 'char', 'ValueType', 'any');
cfg.components = struct( ...
    'name', {'baro', 'control', 'latch', 'guard'}, ...
    'kind', {'sensor', 'periodic', 'periodic', 'periodic'}, ...
    'period', {0.005, 0.01, 0.02, 0.05}, ...
    'priority', {10, 30, 40, 60}, ...
    'delay', {0.01, 0, 0, 0}, ...
    'latches', {false, false, true, false}, ...
    'topic', {'sensor/baro', '', '', ''}, ...
    'sample', {@(t, x) x(1) + 10, [], [], []}, ...
    'step', {[], @control_step, @latch_step, @(t, bus) guard_step(t, bus, capture)});
end

function bus = control_step(t, bus)
[msg, status] = bus.topics.tryRead('sensor/baro', t);
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

function bus = replace_bus(bus)
bus.topics = [];
end

function msg = find_message(messages, name)
msg = [];
for k = 1:numel(messages)
    if ~isempty(messages{k}) && strcmp(messages{k}.name, name)
        msg = messages{k};
        return;
    end
end
error('aero:test:message', '快照中找不到 Topic "%s"', name);
end

function delete_if_exists(path)
if isfile(path)
    delete(path);
end
end
