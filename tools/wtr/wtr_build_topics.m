function [topics, specs] = wtr_build_topics(result, log)
%WTR_BUILD_TOPICS 由结果与发布日志构造 WTR topics/specs
%
% 时间规则：
%   - 估计/参考/控制量使用各自的源采样时钟（日志的 sample_time_s）；
%   - 同一 Topic 的重复时间戳只保留最后一条（例如导航在同一测量上重复
%     发布），不跨 Topic 插值；
%   - 真值直接取 result.time/result.state，不伪装成估计。

layout = result.model_metadata.state_layout;
truthTime = result.time(:);
state = result.state;

truthPosition = state(:, layout.position);
truthVelocity = state(:, layout.velocity);
truthQuaternion = normalize_rows(state(:, layout.quaternion));
truthOmega = state(:, layout.omega);
truthDelta = state(:, layout.servo);

nav = require_topic(log, 'navigation/state');
[navTime, navPos, navVel, navQ, navOmega, navAlpha] = unpack_nav(nav);

rateLog = optional_topic(log, 'control/rate_setpoint');
if isempty(rateLog)
    rateLog = require_topic(log, 'guidance/rate_reference');
end
[rateTime, rateOmega, rateAlpha] = unpack_rates(rateLog);

refLog = optional_topic(log, 'guidance/reference');
if isempty(refLog)
    refTime = rateTime;
    refPosition = zeros(numel(rateTime), 3);
    refVelocity = zeros(numel(rateTime), 3);
    refAcceleration = zeros(numel(rateTime), 3);
    refSource = 'not_applicable (rate-only scenario)';
else
    [refTime, refPosition, refVelocity, refAcceleration] = unpack_reference(refLog);
    refSource = 'guidance/reference';
end

attitudeSetpoint = optional_topic(log, 'control/attitude_setpoint');
if isempty(attitudeSetpoint)
    attitudeTime = rateTime;
    attitudeQ = repmat([1, 0, 0, 0], numel(rateTime), 1);
    attitudeSource = 'not_applicable (rate-only scenario; identity reference)';
else
    [attitudeTime, attitudeQ] = unpack_attitude_setpoint(attitudeSetpoint);
    attitudeSource = 'control/attitude_setpoint (position loop)';
end

rateRefLog = optional_topic(log, 'guidance/rate_reference');
if isempty(rateRefLog)
    rateRefTime = rateTime;
    rateRefOmega = zeros(numel(rateTime), 3);
    rateRefSource = 'not_applicable (hover scenario; zero rate reference)';
else
    [rateRefTime, rateRefOmega, ~] = unpack_rates(rateRefLog);
    rateRefSource = 'guidance/rate_reference';
end

% 锁存舵面：使用真实 latch_servos 事件时刻，不使用状态输出时间轴
latchServoTimes = latch_event_times(result, 'latch_servos');
latchMotorTimes = latch_event_times(result, 'latch_motor');
heldServosAtLatch = held_values(result, latchServoTimes, 'servos');
heldMotorAtLatch = held_values(result, latchMotorTimes, 'motor');

% 状态时间轴上的锁存命令用于 shc09_vanes 的 estimated 对照（零阶保持，
% 不做插值）。这是“指令”而非舵面反馈估计，manifest 中已标注来源。
heldServos = zeros(numel(truthTime), 6);
for k = 1:numel(truthTime)
    if ~isempty(result.held{k}) && isstruct(result.held{k}) && isfield(result.held{k}, 'servos')
        heldServos(k, :) = result.held{k}.servos(:).';
    end
end

topics = struct();
specs = struct();

[topics, specs] = add_topic(topics, specs, 'vehicle_local_position', navTime, ...
    struct('x', navPos(:, 1), 'y', navPos(:, 2), 'z', navPos(:, 3), ...
           'vx', navVel(:, 1), 'vy', navVel(:, 2), 'vz', navVel(:, 3)), ...
    'estimate', 'NED', struct( ...
        'x', unit('m', 'navigation/state (IdealState)'), ...
        'y', unit('m', 'navigation/state (IdealState)'), ...
        'z', unit('m', 'navigation/state (IdealState)'), ...
        'vx', unit('m/s', 'navigation/state (IdealState)'), ...
        'vy', unit('m/s', 'navigation/state (IdealState)'), ...
        'vz', unit('m/s', 'navigation/state (IdealState)')));

[topics, specs] = add_topic(topics, specs, 'vehicle_local_position_groundtruth', truthTime, ...
    struct('x', truthPosition(:, 1), 'y', truthPosition(:, 2), 'z', truthPosition(:, 3), ...
           'vx', truthVelocity(:, 1), 'vy', truthVelocity(:, 2), 'vz', truthVelocity(:, 3)), ...
    'truth', 'NED', struct( ...
        'x', unit('m', 'result.state'), 'y', unit('m', 'result.state'), ...
        'z', unit('m', 'result.state'), ...
        'vx', unit('m/s', 'result.state'), 'vy', unit('m/s', 'result.state'), ...
        'vz', unit('m/s', 'result.state')));

[topics, specs] = add_topic(topics, specs, 'vehicle_attitude', navTime, ...
    struct('q', navQ), 'estimate', 'FRD_to_NED', struct( ...
        'q', unit('1', 'navigation/state (IdealState)')));

[topics, specs] = add_topic(topics, specs, 'vehicle_attitude_groundtruth', truthTime, ...
    struct('q', truthQuaternion), 'truth', 'FRD_to_NED', struct( ...
        'q', unit('1', 'result.state')));

[topics, specs] = add_topic(topics, specs, 'trajectory_setpoint', refTime, ...
    struct('position', refPosition, 'velocity', refVelocity, ...
           'acceleration', refAcceleration), 'reference', 'NED', struct( ...
        'position', unit('m', refSource), 'velocity', unit('m/s', refSource), ...
        'acceleration', unit('m/s^2', refSource)));

[topics, specs] = add_topic(topics, specs, 'vehicle_attitude_reference', attitudeTime, ...
    struct('q', attitudeQ), 'reference', 'FRD_to_NED', struct( ...
        'q', unit('1', attitudeSource)));

[topics, specs] = add_topic(topics, specs, 'vehicle_attitude_setpoint', attitudeTime, ...
    struct('q', attitudeQ), 'controller_setpoint', 'FRD_to_NED', struct( ...
        'q', unit('1', attitudeSource)));

[topics, specs] = add_topic(topics, specs, 'vehicle_rates_setpoint', rateTime, ...
    struct('xyz', rateOmega), 'controller_setpoint', 'FRD', struct( ...
        'xyz', unit('rad/s', 'control/rate_setpoint')));

[topics, specs] = add_topic(topics, specs, 'vehicle_rates_reference', rateRefTime, ...
    struct('xyz', rateRefOmega), 'reference', 'FRD', struct( ...
        'xyz', unit('rad/s', rateRefSource)));

[topics, specs] = add_topic(topics, specs, 'vehicle_angular_velocity', navTime, ...
    struct('xyz', navOmega), 'estimate', 'FRD', struct( ...
        'xyz', unit('rad/s', 'navigation/state (IdealState)')));

[topics, specs] = add_topic(topics, specs, 'vehicle_angular_velocity_groundtruth', truthTime, ...
    struct('xyz', truthOmega), 'truth', 'FRD', struct( ...
        'xyz', unit('rad/s', 'result.state')));

[topics, specs] = add_topic(topics, specs, 'vehicle_angular_acceleration_reference', rateTime, ...
    struct('xyz', rateAlpha), 'reference', 'FRD', struct( ...
        'xyz', unit('rad/s^2', 'control/rate_setpoint alpha feedforward')));

[topics, specs] = add_topic(topics, specs, 'shc09_vanes', truthTime, ...
    struct('actual', truthDelta, 'estimated', heldServos), ...
    'truth_and_estimate_separate', '1-6', struct( ...
        'actual', unit('rad', 'result.state delta'), ...
        'estimated', unit('rad', 'latched command (ideal actuator assumption, not feedback)')));

[topics, specs] = add_topic(topics, specs, 'actuator_servos_pwm', latchServoTimes, ...
    struct('command_rad', heldServosAtLatch), 'pwm_latched', '1-6', struct( ...
        'command_rad', unit('rad', 'latch_servos event held command')));

[topics, specs] = add_topic(topics, specs, 'actuator_motors_pwm', latchMotorTimes, ...
    struct('motor_rad_s', heldMotorAtLatch), 'pwm_latched', 'motor', struct( ...
        'motor_rad_s', unit('rad/s', 'latch_motor event held command')));

end

% ===========================================================================
function [topics, specs] = add_topic(topics, specs, name, time, fields, domain, frame, fieldSpecs)
time = unique_time(time);
for field = fieldnames(fields).'
    fields.(field{1}) = align_rows(fields.(field{1}), numel(time));
end
topic = struct();
topic.time_s = time;
topic.timestamp = uint64(round(time * 1e6));
for field = fieldnames(fields).'
    topic.(field{1}) = fields.(field{1});
end
topics.(name) = topic;
spec = struct('domain', domain, 'frame', frame, 'fields', struct());
for field = fieldnames(fieldSpecs).'
    item = fieldSpecs.(field{1});
    item.width = size(fields.(field{1}), 2);
    spec.fields.(field{1}) = item;
end
specs.(name) = spec;
end

function spec = unit(unitText, sourceText)
spec = struct('unit', unitText, 'source', sourceText, 'width', 1, 'allow_nan', false);
end

function value = align_rows(value, n)
value = reshape(value, n, []);
end

function t = unique_time(t)
t = double(t(:));
if any(~isfinite(t)) || any(t < 0)
    error('wtr:export:time', 'Topic 时间必须有限且非负');
end
if numel(t) < 2 || any(diff(t) <= 0)
    error('wtr:export:time', 'Topic 时间必须严格递增');
end
end

% ===========================================================================
function recs = require_topic(log, name)
recs = optional_topic(log, name);
if isempty(recs)
    error('wtr:export:topic', '缺少必需 Topic 的历史：%s', name);
end
end

function recs = optional_topic(log, name)
mask = cellfun(@(r) strcmp(r.name, name), log.records);
recs = log.records(mask);
if ~isempty(recs)
    times = cellfun(@(r) r.sample_time_s, recs);
    [times, order] = sort(times);
    recs = recs(order);
    [~, ia] = unique(times, 'last');
    recs = recs(sort(ia));
end
end

function [t, position, velocity, quaternion, omega, alpha] = unpack_nav(recs)
n = numel(recs);
t = zeros(n, 1); position = zeros(n, 3); velocity = zeros(n, 3);
quaternion = zeros(n, 4); omega = zeros(n, 3); alpha = zeros(n, 3);
for k = 1:n
    d = recs{k}.data;
    t(k) = recs{k}.sample_time_s;
    position(k, :) = d.position(:).';
    velocity(k, :) = d.velocity(:).';
    quaternion(k, :) = normalize_rows(d.quaternion(:).');
    omega(k, :) = d.omega(:).';
    alpha(k, :) = d.alpha(:).';
end
end

function [t, position, velocity, acceleration] = unpack_reference(recs)
n = numel(recs);
t = zeros(n, 1); position = zeros(n, 3); velocity = zeros(n, 3); acceleration = zeros(n, 3);
for k = 1:n
    d = recs{k}.data;
    t(k) = recs{k}.sample_time_s;
    position(k, :) = d.position(:).';
    velocity(k, :) = d.velocity(:).';
    acceleration(k, :) = d.acceleration(:).';
end
end

function [t, omega, alpha] = unpack_rates(recs)
n = numel(recs);
t = zeros(n, 1); omega = zeros(n, 3); alpha = zeros(n, 3);
for k = 1:n
    d = recs{k}.data;
    t(k) = recs{k}.sample_time_s;
    omega(k, :) = d.omega(:).';
    if isfield(d, 'alpha')
        alpha(k, :) = d.alpha(:).';
    end
end
end

function [t, quaternion] = unpack_attitude_setpoint(recs)
n = numel(recs);
t = zeros(n, 1); quaternion = zeros(n, 4);
for k = 1:n
    t(k) = recs{k}.sample_time_s;
    quaternion(k, :) = normalize_rows(recs{k}.data.quaternion(:).');
end
end

function q = normalize_rows(q)
norms = sqrt(sum(q .^ 2, 2));
if any(norms < 1e-12)
    error('wtr:export:quaternion', '四元数模长为零');
end
q = q ./ norms;
negative = q(:, 1) < 0;
q(negative, :) = -q(negative, :);
end

% ===========================================================================
function times = latch_event_times(result, componentName)
components = {result.events.component};
actions = {result.events.action};
mask = strcmp(components, componentName) & strcmp(actions, 'step');
times = unique([result.events(mask).time].');
if numel(times) < 2
    error('wtr:export:latch', '缺少 %s 的锁存事件', componentName);
end
end

function values = held_values(result, times, field)
if strcmp(field, 'servos')
    values = zeros(numel(times), 6);
else
    values = zeros(numel(times), 1);
end
for k = 1:numel(times)
    [~, g] = min(abs(result.time - times(k)));
    held = result.held{g};
    if ~isempty(held) && isstruct(held) && isfield(held, field)
        if strcmp(field, 'servos')
            values(k, :) = held.servos(:).';
        else
            values(k) = held.motor;
        end
    end
end
end
