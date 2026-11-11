function info = test_velocity_step_guidance()
%TEST_VELOCITY_STEP_GUIDANCE 速度阶跃 Guidance 的分段常值、航向与 Topic 契约

params = struct( ...
    'output_topic', 'guidance/velocity_reference', ...
    'velocity', [0.8; 0; 0], ...
    'acceleration_ff', [0.1; -0.2; 0.05], ...
    'yaw', 0.3, ...
    'yaw_rate', 0.1, ...
    'start_s', 0.5, ...
    'end_s', 2.5);

% 起始前
bus = make_bus();
velocity_step(0.49, bus, params);
[msg, status] = bus.topics.tryRead('guidance/velocity_reference', 0.49);
check_that(strcmp(status, 'ok'), '起始前没有速度参考');
check_that(max(abs(msg.data.velocity)) < 1e-12, '起始前速度不为零');

% 阶跃期间
bus = make_bus();
velocity_step(0.5, bus, params);
[msg, status] = bus.topics.tryRead('guidance/velocity_reference', 0.5);
check_that(strcmp(status, 'ok'), '阶跃起点没有速度参考');
check_that(max(abs(msg.data.velocity - [0.8; 0; 0])) < 1e-12, ...
    '阶跃期间速度不是目标值');
check_that(max(abs(msg.data.acceleration_ff - [0.1; -0.2; 0.05])) < 1e-12, ...
    '前馈加速度错误');
check_that(abs(msg.data.yaw - 0.3) < 1e-12 && abs(msg.data.yaw_rate - 0.1) < 1e-12, ...
    'yaw/yaw_rate 错误');

bus = make_bus();
velocity_step(2.4, bus, params);
[msg, status] = bus.topics.tryRead('guidance/velocity_reference', 2.4);
check_that(strcmp(status, 'ok') && max(abs(msg.data.velocity - [0.8; 0; 0])) < 1e-12, ...
    '阶跃结束前速度提前归零');

% 结束后
bus = make_bus();
velocity_step(2.5, bus, params);
[msg, status] = bus.topics.tryRead('guidance/velocity_reference', 2.5);
check_that(strcmp(status, 'ok'), '结束后没有速度参考');
check_that(max(abs(msg.data.velocity)) < 1e-12, '结束后速度未归零');

% Topic 契约
specs = topic_specs();
names = cellfun(@(x) x.name, specs, 'UniformOutput', false);
idx = find(strcmp(names, 'guidance/velocity_reference'), 1);
check_that(~isempty(idx), 'Topic 契约缺少 guidance/velocity_reference');
spec = specs{idx}.spec;
check_that(isequal(sort(spec.fields), ...
    sort({'velocity', 'acceleration_ff', 'yaw', 'yaw_rate'})), ...
    '速度参考 Topic 字段错误');
check_that(isequal(spec.units, {'m/s', 'm/s^2', 'rad', 'rad/s'}), ...
    '速度参考 Topic 单位错误');
check_that(isequal(spec.dimensions, {[3 1], [3 1], [1 1], [1 1]}), ...
    '速度参考 Topic 尺寸错误');

info = struct('passed', true, ...
    'summary', '速度阶跃前/中/后、yaw/前馈与 Topic 契约通过');

end

% ===========================================================================
function bus = make_bus()
bus = struct('topics', aero_topic_bus());
specs = topic_specs();
idx = find(strcmp(cellfun(@(x) x.name, specs, 'UniformOutput', false), ...
    'guidance/velocity_reference'), 1);
bus.topics.register('guidance/velocity_reference', specs{idx}.spec);
end
