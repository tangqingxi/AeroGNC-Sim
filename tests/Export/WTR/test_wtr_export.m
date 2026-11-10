function info = test_wtr_export()
%TEST_WTR_EXPORT WTR 契约、锁存时间轴、参数追溯、容量与覆盖保护

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = rate_step_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.5], ...
    'max_records', 30000));
result = aero_sim(scenario.model, scenario.cfg);

root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
options = struct('scenario', scenario.name, 'output_root', root, ...
    'signature', scenario.signature, 'gains', scenario.gains, ...
    'seed', scenario.cfg.seed);

out = wtr_export(result, scenario.model, options);
check_that(isfile(out.mat_path) && isfile(out.wtr_path), '导出文件缺失');
matHash = wtr_sha256(out.mat_path);
wtrHash = wtr_sha256(out.wtr_path);

report = wtr_validate(out.wtr_path);
check_that(report.passed, 'WTR 独立校验失败');
check_that(strcmp(report.run_id, out.run_id), 'WTR run_id 与导出结果不一致');

% ---- 锁存时间轴：PWM 必须使用真实 latch 事件时刻 --------------------
topics = load_topics(out.wtr_path);
expectedServo = latch_times(result, 'latch_servos');
actualServo = double(topics.actuator_servos_pwm.time_s(:));
check_that(numel(actualServo) == numel(expectedServo), ...
    'PWM 时间点数 %d ≠ 锁存事件数 %d', numel(actualServo), numel(expectedServo));
check_that(max(abs(actualServo - expectedServo)) < 1e-9, ...
    'PWM 时间轴未使用真实锁存时刻');
expectedMotor = latch_times(result, 'latch_motor');
actualMotor = double(topics.actuator_motors_pwm.time_s(:));
check_that(numel(actualMotor) == numel(expectedMotor), '电机锁存时间点数不一致');

% ---- 参数追溯 --------------------------------------------------------
paths = parameter_paths(out.wtr_path);
required = {'vehicle.mass_kg', 'vehicle.inertia_frd_kg_m2', ...
    'aero.trust', 'aero.file_sha256', 'actuator.servo_device', ...
    'actuator.motor_device', 'control.position.Kp', 'control.velocity.Kd', ...
    'control.attitude.Kp', 'control.rate.Kp', 'allocation.trim_delta_rad', ...
    'rates.position_hz', 'rates.velocity_hz', 'solver.RelTol', ...
    'experiment.scenario'};
missing = setdiff(required, paths);
check_that(isempty(missing), 'WTR 参数缺少：%s', strjoin(missing, ', '));

% ---- 容量治理路径（警告分支，不破坏文件） ---------------------------
warnRoot = tempname();
cleanup2 = onCleanup(@() remove_tree(warnRoot)); %#ok<NASGU>
policy = aero_results_policy(struct('warn_bytes', 1, 'limit_bytes', 10e9));
lastwarn('');
warnOut = wtr_export(result, scenario.model, struct( ...
    'scenario', 'capacity_probe', 'output_root', warnRoot, ...
    'signature', scenario.signature, 'gains', scenario.gains, ...
    'seed', scenario.cfg.seed, 'policy', policy));
[~, warnId] = lastwarn();
check_that(strcmp(warnId, 'wtr:export:capacity'), ...
    'WTR 未执行 Results 容量检查（warning id=%s）', warnId);
check_that(isfile(warnOut.wtr_path), '容量警告后 WTR 未写出');

% ---- 覆盖保护：重复导出拒绝且不改变已有文件 -------------------------
check_error(@() wtr_export(result, scenario.model, options), ...
    'wtr:export:exists', '重复导出未拒绝');
check_that(strcmp(wtr_sha256(out.mat_path), matHash), '失败导出修改了原始 .mat');
check_that(strcmp(wtr_sha256(out.wtr_path), wtrHash), '失败导出修改了已有 .wtr');

% ---- 未启用发布日志的结果不能导出 -----------------------------------
noLog = scenario;
noLog.cfg.communication.log.enabled = false;
shortCfg = noLog.cfg;
shortCfg.tspan = [0, 0.3];
shortResult = aero_sim(noLog.model, shortCfg);
check_error(@() wtr_export(shortResult, noLog.model, ...
    struct('scenario', 'no_log', 'output_root', root, ...
    'signature', scenario.signature)), 'wtr:export:log', '无日志结果被错误导出');

info = struct('passed', true, 'summary', sprintf( ...
    'WTR %d topics、锁存时间轴、参数追溯与容量/覆盖保护通过', ...
    report.topic_count));

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

function times = latch_times(result, componentName)
components = {result.events.component};
actions = {result.events.action};
mask = strcmp(components, componentName) & strcmp(actions, 'step');
times = unique([result.events(mask).time].');
end

function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
