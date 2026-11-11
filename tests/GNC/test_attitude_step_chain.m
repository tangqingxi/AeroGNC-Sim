function info = test_attitude_step_chain()
%TEST_ATTITUDE_STEP_CHAIN 姿态参考独立控制链、Position/Velocity 绕过与 WTR

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = attitude_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.3], ...
    'start_s', 0.05, ...
    'end_s', 0.2, ...
    'max_records', 30000));
result = aero_sim(scenario.model, scenario.cfg);
def = scenario.cfg.experiment.definition;

% 组件与实验定义：没有 Position/Velocity
componentNames = {scenario.cfg.components.name};
check_that(~any(strcmp(componentNames, 'position')), 'Attitude 场景仍装配 Position');
check_that(~any(strcmp(componentNames, 'velocity')), 'Attitude 场景仍装配 Velocity');
check_that(all(ismember({'sensor_ideal', 'navigation', 'guidance_attitude', ...
    'attitude', 'rate', 'allocation', 'latch_servos', 'latch_motor'}, ...
    componentNames)), '姿态场景组件不完整');
check_that(isequal(def.active_modules, ...
    {'guidance', 'attitude', 'rate', 'allocation'}), 'active_modules 错误');
check_that(~isfield(def.controllers, 'position') ...
    && ~isfield(def.controllers, 'velocity'), ...
    '定义错误包含 Position/Velocity 控制器');
check_that(strcmp(def.method, 'ATT_P_RATE_PID_PINV'), 'method 错误');

% 发布日志：Attitude P 直接读 Guidance，不走 Velocity/Attitude setpoint
records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
check_that(any(strcmp(names, 'guidance/attitude_reference')), ...
    '缺少 guidance/attitude_reference 历史');
check_that(~any(strcmp(names, 'control/velocity_setpoint')), ...
    '姿态场景出现 Velocity setpoint 记录');
check_that(~any(strcmp(names, 'control/attitude_setpoint')), ...
    '姿态场景出现 control/attitude_setpoint（Velocity 环输出）');
check_that(any(strcmp(names, 'control/rate_setpoint')), ...
    'Attitude P 未输出 rate setpoint');
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

% 固定推力：Allocation 电机指令来自 reference
thrustRef = def.guidance.thrust_reference_n;
kT0 = scenario.model.parameters.duct.k_T0;
expectedMotor = sqrt(thrustRef / kT0);
allocation = records(strcmp(names, 'control/allocation_status'));
check_that(~isempty(allocation), '缺少 allocation 历史');
motor = cellfun(@(r) r.data.motor_rad_s, allocation);
check_that(max(abs(motor - expectedMotor)) < 1e-6, ...
    '电机转速不是由固定推力参考推导');

% 多速率与因果性
periods = containers.Map();
for k = 1:numel(scenario.cfg.components)
    periods(scenario.cfg.components(k).name) = scenario.cfg.components(k).period;
end
check_that(abs(periods('guidance_attitude') - 1 / 50) < 1e-12, ...
    'Guidance 周期不是 50 Hz');
check_that(abs(periods('attitude') - 0.005) < 1e-12, 'Attitude 周期不是 200 Hz');
check_that(abs(periods('rate') - 0.0025) < 1e-12, 'Rate 周期不是 400 Hz');
for k = 1:numel(records)
    check_that(records{k}.available_time_s >= records{k}.sample_time_s, ...
        'Topic 输出早于其采样时刻（未来数据泄漏）');
end

% WTR：姿态参考进入 attitude reference/setpoint，位置/速度明确 not_applicable
root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
out = wtr_export(result, scenario.model, struct('output_root', root));
report = wtr_validate(out.wtr_path);
check_that(report.passed, '姿态场景 WTR 校验失败');

manifest = load_manifest(out.wtr_path);
check_that(strcmp(manifest.scenario, 'attitude_step'), 'manifest scenario 错误');
check_that(strcmp(manifest.method, 'ATT_P_RATE_PID_PINV'), 'manifest method 错误');
check_that(contains(manifest.topics.vehicle_attitude_reference.fields.q.source, ...
    'guidance/attitude_reference'), 'WTR attitude reference 来源错误');
check_that(contains(manifest.topics.vehicle_attitude_setpoint.fields.q.source, ...
    'guidance/attitude_reference'), 'WTR attitude setpoint 来源错误');
check_that(contains(manifest.topics.trajectory_setpoint.fields.velocity.source, ...
    'not_applicable'), 'WTR 错误声称存在速度参考');
check_that(strcmp(manifest.topics.vehicle_rates_setpoint.fields.xyz.source, ...
    'control/rate_setpoint'), 'WTR rate setpoint 来源错误');

topics = load_topics(out.wtr_path);
qRef = topics.vehicle_attitude_reference;
stepMask = qRef.time_s >= 0.05 & qRef.time_s < 0.2;
qStep = def.guidance.quaternion_bn(:).';
check_that(any(stepMask) && max(abs(qRef.q(stepMask, :) - qStep), [], 'all') < 1e-12, ...
    'WTR 姿态参考数值错误');

paths = parameter_paths(out.wtr_path);
required = {'experiment.reference.roll_rad', 'experiment.reference.pitch_rad', ...
    'experiment.reference.yaw_rad', 'experiment.reference.quaternion_bn', ...
    'experiment.reference.thrust_reference_n', ...
    'experiment.reference.yaw_rate_rad_s', ...
    'experiment.reference.start_s', 'experiment.reference.end_s'};
missing = setdiff(required, paths);
check_that(isempty(missing), 'WTR 参数缺少：%s', strjoin(missing, ', '));

% MAT roundtrip
loaded = load(out.mat_path).result;
check_that(strcmp(loaded.config.experiment.definition.scenario, 'attitude_step'), ...
    'MAT roundtrip 丢失 attitude_step 定义');
check_that(strcmp(loaded.config.experiment.identity_sha256, ...
    result.config.experiment.identity_sha256), 'MAT roundtrip 改变实验身份');

info = struct('passed', true, 'summary', sprintf( ...
    '无 Position/Velocity、姿态参考 %d 条、固定推力 %.3f N、WTR 校验通过', ...
    sum(strcmp(names, 'guidance/attitude_reference')), thrustRef));

end

% ===========================================================================
function topics = load_topics(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
data = load(fullfile(stage, 'flight.mat'));
topics = data.topics;
end

function paths = parameter_paths(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
params = jsondecode(fileread(fullfile(stage, 'parameters.json')));
paths = {params.entries.path};
end

function manifest = load_manifest(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
manifest = jsondecode(fileread(fullfile(stage, 'manifest.json')));
end

function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
