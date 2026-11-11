function info = test_velocity_step_chain()
%TEST_VELOCITY_STEP_CHAIN 速度参考控制链、Position 绕过与 WTR 参数追溯

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = velocity_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.3], ...
    'start_s', 0.05, ...
    'end_s', 0.2, ...
    'velocity_reference', [0.8; 0; 0], ...
    'max_records', 30000));
result = aero_sim(scenario.model, scenario.cfg);
def = scenario.cfg.experiment.definition;

% 组件与实验定义：没有 Position
componentNames = {scenario.cfg.components.name};
check_that(~any(strcmp(componentNames, 'position')), 'Velocity 场景仍装配 Position');
check_that(all(ismember({'sensor_ideal', 'navigation', 'guidance_velocity', ...
    'velocity', 'attitude', 'rate', 'allocation', 'latch_servos', 'latch_motor'}, ...
    componentNames)), '速度场景组件不完整');
check_that(isequal(def.active_modules, ...
    {'guidance', 'velocity', 'attitude', 'rate', 'allocation'}), ...
    'active_modules 错误');
check_that(~isfield(def.controllers, 'position'), '定义错误包含 Position 控制器');
check_that(strcmp(def.method, 'VEL_PID_P_PID_PINV'), 'method 错误');

% 发布日志：Velocity 直接读 Guidance，不存在 Position 的 setpoint 记录
records = result.communication.log.records;
names = cellfun(@(r) r.name, records, 'UniformOutput', false);
check_that(any(strcmp(names, 'guidance/velocity_reference')), ...
    '缺少 guidance/velocity_reference 历史');
check_that(~any(strcmp(names, 'control/velocity_setpoint')), ...
    'Velocity 场景出现 control/velocity_setpoint（Position 发布）');
check_that(result.communication.log.dropped == 0, '发布日志发生丢弃');

velocityRecords = records(strcmp(names, 'guidance/velocity_reference'));
check_that(~isempty(velocityRecords), '速度参考历史为空');
stepRecords = velocityRecords(cellfun(@(r) ...
    r.sample_time_s >= 0.05 && r.sample_time_s < 0.2, velocityRecords));
check_that(~isempty(stepRecords), '缺少阶跃期间速度参考');
check_that(all(cellfun(@(r) max(abs(r.data.velocity - [0.8; 0; 0])) < 1e-12, ...
    stepRecords)), '阶跃期间速度参考不是目标值');

% 多速率与因果性
periods = containers.Map();
for k = 1:numel(scenario.cfg.components)
    periods(scenario.cfg.components(k).name) = scenario.cfg.components(k).period;
end
check_that(abs(periods('guidance_velocity') - 1 / 50) < 1e-12, ...
    'Guidance 周期不是 50 Hz');
check_that(abs(periods('velocity') - 0.01) < 1e-12, 'Velocity 周期不是 100 Hz');
check_that(abs(periods('attitude') - 0.005) < 1e-12, 'Attitude 周期不是 200 Hz');
check_that(abs(periods('rate') - 0.0025) < 1e-12, 'Rate 周期不是 400 Hz');
for k = 1:numel(records)
    check_that(records{k}.available_time_s >= records{k}.sample_time_s, ...
        'Topic 输出早于其采样时刻（未来数据泄漏）');
end

% WTR：速度参考进入 trajectory_setpoint，参数追溯到速度参考
root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
out = wtr_export(result, scenario.model, struct('output_root', root));
report = wtr_validate(out.wtr_path);
check_that(report.passed, '速度场景 WTR 校验失败');

manifest = load_manifest(out.wtr_path);
check_that(strcmp(manifest.scenario, 'velocity_step'), 'manifest scenario 错误');
check_that(strcmp(manifest.method, 'VEL_PID_P_PID_PINV'), 'manifest method 错误');
check_that(strcmp(manifest.topics.trajectory_setpoint.fields.velocity.source, ...
    'guidance/velocity_reference'), 'WTR 速度参考来源错误');
check_that(contains(manifest.topics.trajectory_setpoint.fields.position.source, ...
    'not_applicable'), 'WTR 错误声称存在位置参考');

topics = load_topics(out.wtr_path);
tp = topics.trajectory_setpoint;
stepMask = tp.time_s >= 0.05 & tp.time_s < 0.2;
check_that(any(stepMask) && max(abs(tp.velocity(stepMask, 1) - 0.8)) < 1e-12 ...
    && max(abs(tp.velocity(stepMask, 2:3)), [], 'all') < 1e-12, ...
    'WTR 速度参考数值错误');
check_that(max(abs(tp.position(:))) < 1e-12, 'WTR 速度场景位置参考不为零占位');

paths = parameter_paths(out.wtr_path);
required = {'experiment.reference.velocity_reference_ned_m_s', ...
    'experiment.reference.acceleration_ff_ned_m_s2', ...
    'experiment.reference.yaw_rad', 'experiment.reference.yaw_rate_rad_s', ...
    'experiment.reference.start_s', 'experiment.reference.end_s'};
missing = setdiff(required, paths);
check_that(isempty(missing), 'WTR 参数缺少：%s', strjoin(missing, ', '));

% MAT 保存/恢复后仍保持同一实验定义与身份
loadedData = load(out.mat_path);
loaded = loadedData.result;
check_that(strcmp(loaded.config.experiment.definition.scenario, 'velocity_step'), ...
    'MAT roundtrip 丢失 velocity_step 定义');
check_that(strcmp(loaded.config.experiment.identity_sha256, ...
    result.config.experiment.identity_sha256), 'MAT roundtrip 改变实验身份');
loadedRoot = tempname();
cleanupLoaded = onCleanup(@() remove_tree(loadedRoot)); %#ok<NASGU>
loadedOut = wtr_export(loaded, scenario.model, struct('output_root', loadedRoot));
check_that(strcmp(loadedOut.run_signature, out.run_signature), ...
    'MAT roundtrip 后 WTR 签名改变');
check_that(wtr_validate(loadedOut.wtr_path).passed, ...
    'MAT roundtrip 后 WTR 校验失败');

info = struct('passed', true, 'summary', sprintf( ...
    '无 Position、速度参考 %d 条、WTR %d topics 校验通过', ...
    numel(velocityRecords), report.topic_count));

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
