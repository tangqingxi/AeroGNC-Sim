function info = test_velocity_step_closed_loop()
%TEST_VELOCITY_STEP_CLOSED_LOOP 无 Position 的 SHC09 速度参考闭环响应与物理边界

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = velocity_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 5], ...
    'start_s', 0.5, ...
    'end_s', 2.5, ...
    'velocity_reference', [0.8; 0; 0], ...
    'max_records', 300000));
result = aero_sim(scenario.model, scenario.cfg);
layout = shc09_state_layout();
t = result.time;
velocity = result.state(:, layout.velocity);
position = result.state(:, layout.position);
quaternion = result.state(:, layout.quaternion);

check_that(all(isfinite(t)) && all(isfinite(result.state(:))), ...
    '状态或时间含非有限值');
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

% 速度方向与响应
step = t >= 0.5 & t < 2.5;
maxSpeed = max(vecnorm(velocity, 2, 2));
check_that(maxSpeed > 0.5 && maxSpeed < 1.0, ...
    '最大速度响应异常：%.4f m/s', maxSpeed);
check_that(max(abs(velocity(:, 2))) < 0.10 && max(abs(velocity(:, 3))) < 0.10, ...
    '出现明显非北向速度：vy=%.4f vz=%.4f', ...
    max(abs(velocity(:, 2))), max(abs(velocity(:, 3))));

errorNorth = velocity(step, 1) - 0.8;
rmse = sqrt(mean(errorNorth .^ 2));
check_that(rmse < 0.55, '速度跟踪 RMSE 过大：%.4f m/s', rmse);

residual = norm(velocity(end, :));
check_that(residual < 0.10, '撤销参考后残余速度过大：%.4f m/s', residual);
slowdown = vecnorm(velocity(t >= 2.5 & t < 3.5, :), 2, 2);
check_that(max(slowdown) <= maxSpeed + 1e-12, '撤销参考后速度继续增大');

% 无位置控制器：只允许累计位移，不用位置回原点指标
displacement = norm(position(end, :) - position(1, :));
check_that(displacement > 0.5, '速度阶跃未产生预期位置累计位移');
check_that(norm(position(end, :)) > 0.5, ...
    '速度场景错误地把位置拉回原点');

tilt = 2 * acos(min(1, abs(quaternion(:, 1))));
check_that(max(tilt) < deg2rad(20), '最大倾角超限：%.2f°', rad2deg(max(tilt)));

% 执行器与推力边界
delta = result.state(:, layout.servo);
limit = max(abs(scenario.model.parameters.servo_effective_range_rad));
check_that(max(abs(delta(:))) <= limit + 1e-9, '舵面偏角超出有效行程');

records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
attitudeRecords = records(strcmp(names, 'control/attitude_setpoint'));
allocationRecords = records(strcmp(names, 'control/allocation_status'));
check_that(~isempty(attitudeRecords) && ~isempty(allocationRecords), ...
    '缺少 Attitude/Allocation 历史');
thrust = cellfun(@(r) r.data.thrust_n, attitudeRecords);
maxThrust = scenario.model.parameters.duct.k_T0 ...
    * scenario.model.parameters.motor_device.omega_range_rad_s(2)^2;
check_that(all(isfinite(thrust)) && all(thrust > 0) ...
    && max(thrust) <= maxThrust + 1e-6, '总推力越界或非有限');
motor = cellfun(@(r) r.data.motor_rad_s, allocationRecords);
check_that(all(isfinite(motor)) && all(motor >= 0) ...
    && max(motor) <= scenario.model.parameters.motor_device.omega_range_rad_s(2) + 1e-6, ...
    '电机转速越界或非有限');
saturated = cellfun(@(r) r.data.saturated(:).', allocationRecords, ...
    'UniformOutput', false);
saturated = vertcat(saturated{:});
check_that(all(isfinite(saturated(:))) && all(ismember(saturated(:), [0, 1])), ...
    '分配饱和标志非法');

check_that(~any(strcmp(names, 'control/velocity_setpoint')), ...
    '速度场景出现 Position 的 setpoint 记录');
check_that(~any(strcmp(names, 'position')), '速度场景出现 Position 组件记录');

info = struct('passed', true, 'summary', sprintf( ...
    '速度 max=%.4f, RMSE=%.4f, residual=%.4f, disp=%.4f, tilt=%.2f°', ...
    maxSpeed, rmse, residual, displacement, rad2deg(max(tilt))));

end
