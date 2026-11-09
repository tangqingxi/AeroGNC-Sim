function info = test_communication_integration()
%TEST_COMMUNICATION_INTEGRATION Sensor→Navigation→Control 的 Topic 集成
%
% 频率：Sensor 200 Hz、Navigation 100 Hz、Control 125 Hz、Latched 50 Hz。
% 只使用简单解析模型，不涉及 GNC 算法。

model = struct('name', 'comm-probe', 'x0', 0, ...
    'dynamics', @(t, x, u, env) u); %#ok<INUSD>

sensorSpec = aero_topic_spec({'value', 'sample_time_s'}, {[1 1], [1 1]}, ...
    {'m', 's'}, struct('max_age_s', 0.05));
navSpec = aero_topic_spec({'value'}, {[1 1]}, {'m'});
commandSpec = aero_topic_spec({'value'}, {[1 1]}, {'-'});

cfg = aero_default_config();
cfg.tspan = [0, 0.2];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.solver = struct('RelTol', 1e-8, 'AbsTol', 1e-10, 'MaxStep', Inf, 'InitialStep', []);
cfg.communication.enabled = true;
cfg.communication.topics = { ...
    struct('name', 'sensor/baro', 'spec', sensorSpec), ...
    struct('name', 'nav/state', 'spec', navSpec), ...
    struct('name', 'command/computed', 'spec', commandSpec), ...
    struct('name', 'command/held', 'spec', commandSpec)};
cfg.communication.command_topic = 'command/computed';
cfg.communication.held_command_topic = 'command/held';
capture = containers.Map('KeyType', 'char', 'ValueType', 'any');
cfg.components = struct( ...
    'name', {'baro', 'nav', 'control', 'latch', 'guard'}, ...
    'kind', {'sensor', 'periodic', 'periodic', 'periodic', 'periodic'}, ...
    'period', {0.005, 0.01, 0.008, 0.02, 0.01}, ...
    'priority', {10, 20, 30, 40, 60}, ...
    'delay', {0.01, 0, 0, 0, 0}, ...
    'latches', {false, false, false, true, false}, ...
    'topic', {'sensor/baro', '', '', '', ''}, ...
    'sample', {@(t, x) x(1) + 10, [], [], [], []}, ...
    'step', {[], @nav_step, @control_step, @latch_step, ...
             @(t, bus) guard_step(t, bus, capture)});

result = aero_sim(model, cfg);

check_that(isfield(result, 'communication') && result.communication.enabled, ...
    '通信启用时结果缺少 communication 摘要');
check_that(~isfield(result.communication, 'bus'), ...
    '结果仍保存可变 Bus 句柄');
check_that(isfield(result.communication, 'messages') && ...
    numel(result.communication.messages) == 4, ...
    '结果快照缺少最终消息');
check_that(isfield(result.communication, 'log') && ~result.communication.log.enabled, ...
    '默认应关闭发布日志');
topicBus = capture('bus');
names = topicBus.topicNames();
check_that(numel(names) == 4, '注册 Topic 数不是 4');
check_that(~any(startsWith(names, 'truth/')), '正常反馈通道出现真值 Topic');

% ---- 频率：Sensor 200 Hz / Nav 100 Hz / Control 125 Hz / Latch 50 Hz --
counts = containers.Map();
for k = 1:numel(result.events)
    name = result.events(k).component;
    if strcmp(result.events(k).action, 'sample')
        key = [name '/sample'];
    else
        key = [name '/' result.events(k).action];
    end
    if isKey(counts, key)
        counts(key) = counts(key) + 1;
    else
        counts(key) = 1;
    end
end
check_that(counts('baro/sample') == 40, 'Sensor 采样次数 %d ≠ 40', counts('baro/sample'));
check_that(counts('nav/step') == 20, 'Navigation 更新次数 %d ≠ 20', counts('nav/step'));
check_that(counts('control/step') == 25, 'Control 更新次数 %d ≠ 25', counts('control/step'));
check_that(counts('latch/step') == 10, 'Latched 更新次数 %d ≠ 10', counts('latch/step'));

% ---- 同刻事件顺序：sample → publish → nav → control → latch → guard ---
atT = find(abs([result.events.time] - 0.04) < 1e-12);
seq = {result.events(atT).component};
check_that(numel(atT) == 6, 't=0.04 事件数 %d ≠ 6', numel(atT));
check_that(isequal(seq, {'baro', 'baro', 'nav', 'control', 'latch', 'guard'}), ...
    't=0.04 事件顺序错误：%s', strjoin(seq, '>'));

% ---- 测量只在可用时刻进入 Topic -------------------------------------
% Latest-Value 只保留最后一条消息：检查最终样本的采样/可用关系。
msg = topicBus.read('sensor/baro', 0.2);
check_that(abs(msg.sample_time_s - 0.19) < 1e-12, ...
    '最终样本时刻应为 0.19，实际 %.6f', msg.sample_time_s);
check_that(abs(msg.available_time_s - (msg.sample_time_s + 0.01)) < 1e-12, ...
    'available_time 不等于 sample_time + delay');
check_error(@() topicBus.read('sensor/baro', 0.198), ...
    'aero:bus:notAvailable', '可用时刻之前读取');

% ---- 命令与锁存命令独立 Topic ---------------------------------------
computed = topicBus.latest('command/computed', 'diagnostic');
held = topicBus.latest('command/held', 'diagnostic');
check_that(abs(computed.data.value - result.computed{end}) < 1e-12, ...
    '计算命令 Topic 与结果不一致');
check_that(abs(held.data.value - result.held{end}) < 1e-12, ...
    '锁存命令 Topic 与结果不一致');
check_that(computed.topic_sequence > held.topic_sequence, ...
    '计算命令与锁存命令的 Topic 序号未体现不同频率（%d vs %d）', ...
    computed.topic_sequence, held.topic_sequence);

different = false;
for g = 1:numel(result.held)
    if ~isequaln(result.computed{g}, result.held{g})
        different = true;
        break;
    end
end
check_that(different, '计算命令与锁存命令始终相同，未体现独立锁存');

% ---- 过期消息被拒绝 -------------------------------------------------
check_error(@() topicBus.read('sensor/baro', 0.3), ...
    'aero:bus:staleMessage', '过期消息读取');

% ---- 未注册 Topic 的声明必须失败 ------------------------------------
bad = cfg;
bad.components(1).topic = 'sensor/unknown';
check_error(@() aero_sim(model, bad), 'aero:comm:unknownMeasurementTopic', ...
    '未注册的测量 Topic');

bad = cfg;
bad.communication.command_topic = 'command/unknown';
check_error(@() aero_sim(model, bad), 'aero:comm:unknownCommandTopic', ...
    '未注册的命令 Topic');

bad = cfg;
bad.communication.topics{1}.spec = struct('fields', {{'value'}});
check_error(@() aero_sim(model, bad), 'aero:bus:invalidSpec', ...
    '非法契约');

% ---- 默认关闭时行为与旧版一致 ---------------------------------------
plain = aero_default_config();
plain.tspan = [0, 0.5];
plain.x0 = 0;
plain.initial_command = 1;
resOff = aero_sim(scalar_integrator_model(), plain);
check_that(~isfield(resOff, 'communication'), '关闭通信时结果仍带 communication');
check_that(abs(resOff.state(end) - 0.5) < 1e-9, '关闭通信时数值行为改变');

plainOn = plain;
plainOn.communication.enabled = true;
resOn = aero_sim(scalar_integrator_model(), plainOn);
check_that(isequaln(resOff.state, resOn.state), ...
    '启用空通信配置改变了数值结果');
check_that(resOn.communication.publish_count == 0, ...
    '空通信配置仍发布了消息');

info = struct('passed', true, 'summary', sprintf( ...
    'Sensor→Nav→Control 传递正常；200/100/125/50 Hz 计数正确；命令/锁存 Topic 独立（%d/%d）；默认关闭兼容', ...
    computed.topic_sequence, held.topic_sequence));

end

% ===========================================================================
function bus = nav_step(t, bus)
[m, status] = bus.topics.tryRead('sensor/baro', t);
if ~strcmp(status, 'ok')
    return;   % 尚无可用测量时保持上一状态，不发布
end
check_that(m.available_time_s <= t + 1e-12, ...
    'Navigation 在 t=%.6f 读到不可用测量', t);
bus.topics.publish('nav/state', struct('value', m.data.value), ...
    m.data.sample_time_s, t, t, 'navigation');
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

function bus = guard_step(t, bus, capture)
check_that(isempty(bus.truth), '周期组件读取到真实状态（bus.truth 非空）');
if ~isKey(capture, 'bus')
    capture('bus') = bus.topics;   % 仅用于测试后检查，不进入结果
end
if t == 0
    [~, status] = bus.topics.tryRead('sensor/baro', t);
    check_that(any(strcmp(status, {'missing', 'not_available'})), ...
        't=0 时传感器 Topic 尚不应可读（status=%s）', status);
end
end
