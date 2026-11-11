function info = test_attitude_step_guidance()
%TEST_ATTITUDE_STEP_GUIDANCE 姿态阶跃 Guidance 的前/中/后、四元数与 Topic 契约

roll = deg2rad(5);
qStep = euler_to_quat_bn(roll, 0, 0);
qLevel = euler_to_quat_bn(0, 0, 0);
thrust = 20.5548;

params = struct( ...
    'output_topic', 'guidance/attitude_reference', ...
    'quaternion_level', qLevel, ...
    'quaternion_step', qStep, ...
    'thrust_n', thrust, ...
    'yaw', 0, ...
    'yaw_rate', 0.05, ...
    'start_s', 0.5, ...
    'end_s', 1.5);

% 阶跃前
bus = make_bus();
attitude_step(0.49, bus, params);
[msg, status] = bus.topics.tryRead('guidance/attitude_reference', 0.49);
check_that(strcmp(status, 'ok'), '阶跃前没有姿态参考');
check_that(norm(msg.data.quaternion - qLevel) < 1e-12, '阶跃前不是水平姿态');
check_that(abs(norm(msg.data.quaternion) - 1) < 1e-12, '四元数未单位化');
check_that(abs(msg.data.thrust_n - thrust) < 1e-12, '固定推力错误');
check_that(abs(msg.data.yaw_rate - 0.05) < 1e-12, 'yaw_rate 错误');

% 阶跃期间
bus = make_bus();
attitude_step(0.5, bus, params);
[msg, status] = bus.topics.tryRead('guidance/attitude_reference', 0.5);
check_that(strcmp(status, 'ok'), '阶跃起点没有姿态参考');
check_that(norm(msg.data.quaternion - qStep) < 1e-12, '阶跃姿态四元数错误');
check_that(abs(norm(msg.data.quaternion) - 1) < 1e-12, '阶跃四元数未单位化');
R = quat_to_rotation(msg.data.quaternion);
expected = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
check_that(max(abs(R(:) - expected(:))) < 1e-12, '四元数对应的滚转旋转错误');

% 阶跃结束
bus = make_bus();
attitude_step(1.5, bus, params);
[msg, status] = bus.topics.tryRead('guidance/attitude_reference', 1.5);
check_that(strcmp(status, 'ok') && norm(msg.data.quaternion - qLevel) < 1e-12, ...
    '阶跃结束后未恢复水平姿态');

% Topic 契约
specs = topic_specs();
names = cellfun(@(x) x.name, specs, 'UniformOutput', false);
idx = find(strcmp(names, 'guidance/attitude_reference'), 1);
check_that(~isempty(idx), 'Topic 契约缺少 guidance/attitude_reference');
spec = specs{idx}.spec;
check_that(isequal(sort(spec.fields), ...
    sort({'quaternion', 'thrust_n', 'yaw', 'yaw_rate'})), ...
    '姿态参考 Topic 字段错误');
check_that(isequal(spec.units, {'1', 'N', 'rad', 'rad/s'}), ...
    '姿态参考 Topic 单位错误');
check_that(isequal(spec.dimensions, {[4 1], [1 1], [1 1], [1 1]}), ...
    '姿态参考 Topic 尺寸错误');

info = struct('passed', true, ...
    'summary', '姿态阶跃前/中/后、四元数、推力与 Topic 契约通过');

end

% ===========================================================================
function bus = make_bus()
bus = struct('topics', aero_topic_bus());
specs = topic_specs();
idx = find(strcmp(cellfun(@(x) x.name, specs, 'UniformOutput', false), ...
    'guidance/attitude_reference'), 1);
bus.topics.register('guidance/attitude_reference', specs{idx}.spec);
end
