function info = test_topic_bus()
%TEST_TOPIC_BUS Communication-Core：注册、发布、读取、时间与契约校验

bus = aero_topic_bus();

% ---- 契约与注册 ------------------------------------------------------
posSpec = aero_topic_spec({'position', 'velocity'}, {[3 1], [3 1]}, {'m', 'm/s'}, ...
    struct('description', 'NED 位置与速度', 'max_age_s', 0.5));
bus.register('vehicle_local_position', posSpec);
check_that(bus.isRegistered('vehicle_local_position'), '注册后 isRegistered 为假');
check_that(isequal(bus.topicNames(), {'vehicle_local_position'}), 'topicNames 不正确');
check_that(strcmp(bus.getSpec('vehicle_local_position').units{2}, 'm/s'), '单位契约丢失');
check_that(~bus.hasMessage('vehicle_local_position'), '未发布时 hasMessage 为真');

check_error(@() bus.register('vehicle_local_position', posSpec), ...
    'aero:bus:duplicateTopic', '重复注册');
check_error(@() bus.register('Bad-Name', posSpec), 'aero:bus:invalidName', '非法名称');
check_error(@() bus.register('UPPER', posSpec), 'aero:bus:invalidName', '大写名称');
check_error(@() bus.register('truth/a/b', posSpec), 'aero:bus:invalidName', '过深名称');
check_error(@() bus.register('raw_topic', struct('fields', {{'x'}})), ...
    'aero:bus:invalidSpec', '不完整契约');

% ---- 真值 Topic 规则 -------------------------------------------------
defaultSpec = aero_topic_spec({'x'}, {[1 1]}, {'-'});
truthSpec = aero_topic_spec({'x'}, {[1 1]}, {'-'}, ...
    struct('allow_ground_truth', true));
check_error(@() bus.register('truth/state', defaultSpec), ...
    'aero:bus:groundTruthTopic', '真值 Topic 未声明真值标志');
check_error(@() bus.register('vehicle_attitude', truthSpec), ...
    'aero:bus:groundTruthTopic', '普通 Topic 声明真值');
bus.register('truth/state', truthSpec);
check_that(bus.getSpec('truth/state').allow_ground_truth, '真值 Topic 注册失败');

% ---- 发布与读取 ------------------------------------------------------
payload = struct('position', [1; 2; 3], 'velocity', [0.1; 0.2; 0.3]);
bus.publish('vehicle_local_position', payload, 1.0, 1.05, 'navigation');
msg = bus.latest('vehicle_local_position');
check_that(msg.sample_time_s == 1.0 && msg.available_time_s == 1.05, '时间戳记录错误');
check_that(msg.sequence == 1 && msg.topic_sequence == 1, '首次发布序号错误');
check_that(strcmp(msg.source, 'navigation'), '来源未记录');

check_error(@() bus.read('vehicle_local_position', 1.0), ...
    'aero:bus:notAvailable', '未到可用时刻');
msg = bus.read('vehicle_local_position', 1.05);
check_that(isequal(msg.data, payload), '读取的载荷不一致');
[missingMsg, status] = bus.tryRead('vehicle_local_position', 1.04);
check_that(isempty(missingMsg) && strcmp(status, 'not_available'), 'tryRead 可用性状态错误');

% ---- 多 Topic 独立版本与全局顺序 ------------------------------------
bus.register('vehicle_attitude', aero_topic_spec({'q'}, {[4 1]}, {'-'}));
bus.publish('vehicle_attitude', struct('q', [1; 0; 0; 0]), 1.0, 1.0, 'navigation');
check_that(bus.latest('vehicle_attitude').topic_sequence == 1, '第二 Topic 版本号未独立');
check_that(bus.latest('vehicle_attitude').sequence == 2, '全局顺序未递增');
check_that(bus.sequence() == 2, 'sequence() 与全局顺序不一致');

payload2 = struct('position', [1; 2; 4], 'velocity', [0.1; 0.2; 0.4]);
bus.publish('vehicle_local_position', payload2, 1.0, 1.05, 'navigation');
check_that(bus.latest('vehicle_local_position').topic_sequence == 2, '同 Topic 版本未递增');
check_that(bus.latest('vehicle_local_position').sequence == 3, '全局顺序未递增');
check_that(isequal(bus.latest('vehicle_local_position').data, payload2), '同一时刻后发布未覆盖');

% ---- 时间倒退与非法时间 --------------------------------------------
check_error(@() bus.publish('vehicle_local_position', payload, 0.9, 0.9), ...
    'aero:bus:timeRegression', '时间倒退');
check_that(isequal(bus.latest('vehicle_local_position').data, payload2), '时间倒退修改了消息');
check_error(@() bus.publish('vehicle_local_position', payload, NaN, 1.0), ...
    'aero:bus:invalidTime', 'NaN 采样时间');
check_error(@() bus.publish('vehicle_local_position', payload, 2.0, 1.9), ...
    'aero:bus:invalidTime', '可用时间早于采样时间');

% ---- 缺失与过期 ------------------------------------------------------
[noMsg, status] = bus.tryRead('unknown_topic', 0);
check_that(isempty(noMsg) && strcmp(status, 'unknown_topic'), '未知 Topic 状态错误');
check_error(@() bus.read('unknown_topic', 0), 'aero:bus:unknownTopic', '未知 Topic 读取');
bus.register('vehicle_status', aero_topic_spec({'armed'}, {[1 1]}, {'-'}));
[noMsg, status] = bus.tryRead('vehicle_status', 0);
check_that(isempty(noMsg) && strcmp(status, 'missing'), '未发布 Topic 状态错误');
check_error(@() bus.read('vehicle_status', 0), 'aero:bus:missingMessage', '未发布读取');

bus.publish('vehicle_local_position', payload2, 2.0, 2.0);
msg = bus.read('vehicle_local_position', 2.5);
check_that(isequal(msg.data, payload2), '有效期内读取失败');
[staleMsg, status] = bus.tryRead('vehicle_local_position', 2.6);
check_that(isempty(staleMsg) && strcmp(status, 'stale'), '过期状态错误');
check_error(@() bus.read('vehicle_local_position', 2.6), ...
    'aero:bus:staleMessage', '过期读取');

% ---- 载荷契约 --------------------------------------------------------
bad = struct('position', [1; 2; 3]);
check_error(@() bus.publish('vehicle_local_position', bad, 3.0), ...
    'aero:bus:schemaMissingField', '缺少字段');
bad = struct('position', [1; 2; 3], 'velocity', [0; 0; 0], 'extra', 1);
check_error(@() bus.publish('vehicle_local_position', bad, 3.0), ...
    'aero:bus:schemaExtraField', '多余字段');
bad = struct('position', [1 2 3], 'velocity', [0; 0; 0]);
check_error(@() bus.publish('vehicle_local_position', bad, 3.0), ...
    'aero:bus:schemaDimension', '行向量尺寸错误');
bad = struct('position', [1; 2; NaN], 'velocity', [0; 0; 0]);
check_error(@() bus.publish('vehicle_local_position', bad, 3.0), ...
    'aero:bus:schemaValue', '非有限值');
bad = struct('position', 'abc', 'velocity', [0; 0; 0]);
check_error(@() bus.publish('vehicle_local_position', bad, 3.0), ...
    'aero:bus:schemaType', '非数值字段');
check_error(@() bus.publish('vehicle_status', [1 2 3], 0), ...
    'aero:bus:schemaType', '非结构体载荷');

% ---- 现有 Simulation 接口不受影响 ----------------------------------
cfg = aero_default_config();
cfg.tspan = [0, 0.1];
cfg.x0 = 0;
cfg.initial_command = 1;
result = aero_sim(scalar_integrator_model(), cfg);
check_that(abs(result.state(end) - 0.1) < 1e-9, 'aero_sim 行为因通信核心改变');

info = struct('passed', true, 'summary', sprintf( ...
    '注册/发布/读取/多 Topic 版本/时间与可用性/过期/契约/真值策略通过，共 %d 条 Topic', ...
    numel(bus.topicNames())));

end
