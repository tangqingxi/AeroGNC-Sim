function info = test_attitude_step_closed_loop()
%TEST_ATTITUDE_STEP_CLOSED_LOOP 5° 滚转阶跃的姿态跟踪、恢复与执行器边界

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = attitude_step_scenario(struct( ...
    'aero_source', source, ...
    'max_records', 200000));
result = aero_sim(scenario.model, scenario.cfg);
layout = shc09_state_layout();
t = result.time;
q = result.state(:, layout.quaternion);
omega = result.state(:, layout.omega);
position = result.state(:, layout.position);
velocity = result.state(:, layout.velocity);

check_that(all(isfinite(t)) && all(isfinite(result.state(:))), ...
    '状态或时间含非有限值');
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

% 目标姿态与姿态误差
qStep = scenario.cfg.experiment.definition.guidance.quaternion_bn(:).';
qLevel = scenario.cfg.experiment.definition.guidance.quaternion_level_bn(:).';
step = t >= 0.5 & t < 1.5;
ref = repmat(qLevel, numel(t), 1);
ref(step, :) = repmat(qStep, sum(step), 1);
attitudeError = 2 * acos(min(1, abs(sum(q .* ref, 2))));
roll = quat_roll(q);
pitch = quat_pitch(q);

maxRoll = max(abs(roll(step)));
check_that(maxRoll > deg2rad(4) && maxRoll < deg2rad(7), ...
    '阶跃滚转响应异常：%.3f°', rad2deg(maxRoll));
check_that(max(attitudeError(step)) < deg2rad(8), ...
    '阶跃期间姿态误差过大：%.3f°', rad2deg(max(attitudeError(step))));

maxOmega = max(vecnorm(omega, 2, 2));
check_that(maxOmega < 2.0, '最大角速度异常：%.3f rad/s', maxOmega);

% 恢复水平：roll/pitch 回到零；yaw 无位置积分，允许少量漂移
check_that(abs(roll(end)) < deg2rad(0.5), ...
    '结束时滚转未恢复：%.3f°', rad2deg(roll(end)));
check_that(abs(pitch(end)) < deg2rad(0.5), ...
    '结束时俯仰未恢复：%.3f°', rad2deg(pitch(end)));
check_that(attitudeError(end) < deg2rad(3.0), ...
    '结束时整体姿态误差过大：%.3f°', rad2deg(attitudeError(end)));

% 无位置/速度控制，但记录平动响应；不得用位置保持作为指标
displacement = norm(position(end, :) - position(1, :));
check_that(displacement > 0.5, '姿态倾斜未产生预期平动位移');
check_that(max(vecnorm(velocity, 2, 2)) > 0.2, '速度未被姿态倾斜激发');

% 执行器边界
delta = result.state(:, layout.servo);
limit = max(abs(scenario.model.parameters.servo_effective_range_rad));
check_that(max(abs(delta(:))) <= limit + 1e-9, '舵面偏角超出有效行程');

records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
allocation = records(strcmp(names, 'control/allocation_status'));
check_that(~isempty(allocation), '缺少 allocation 历史');
motor = cellfun(@(r) r.data.motor_rad_s, allocation);
thrustRef = scenario.cfg.experiment.definition.guidance.thrust_reference_n;
expectedMotor = sqrt(thrustRef / scenario.model.parameters.duct.k_T0);
check_that(max(abs(motor - expectedMotor)) < 1e-6, ...
    '电机转速不是由固定推力参考推导');
check_that(max(motor) <= scenario.model.parameters.motor_device.omega_range_rad_s(2) + 1e-6, ...
    '电机转速越界');

saturatedCell = cellfun(@(r) r.data.saturated(:).', allocation, ...
    'UniformOutput', false);
saturated = vertcat(saturatedCell{:});
check_that(all(isfinite(saturated(:))) && all(ismember(saturated(:), [0, 1])), ...
    '分配饱和标志非法');

check_that(~any(strcmp(names, 'position')), '姿态场景出现 Position 记录');
check_that(~any(strcmp(names, 'velocity')), '姿态场景出现 Velocity 记录');
check_that(~any(strcmp(names, 'control/velocity_setpoint')), ...
    '姿态场景出现 Velocity setpoint 记录');
check_that(~any(strcmp(names, 'control/attitude_setpoint')), ...
    '姿态场景出现 Velocity 的 attitude setpoint 记录');

info = struct('passed', true, 'summary', sprintf( ...
    'roll max=%.3f°, final=%.3f°/%.3f°, err=%.3f°, omega=%.3f, disp=%.3f m', ...
    rad2deg(maxRoll), rad2deg(roll(end)), rad2deg(pitch(end)), ...
    rad2deg(attitudeError(end)), maxOmega, displacement));

end

% ===========================================================================
function roll = quat_roll(q)
q = q ./ sqrt(sum(q .^ 2, 2));
roll = atan2(2 * (q(:, 1) .* q(:, 2) + q(:, 3) .* q(:, 4)), ...
    1 - 2 * (q(:, 2) .^ 2 + q(:, 3) .^ 2));
end

function pitch = quat_pitch(q)
q = q ./ sqrt(sum(q .^ 2, 2));
pitch = asin(max(-1, min(1, 2 * (q(:, 1) .* q(:, 3) - q(:, 4) .* q(:, 2)))));
end
