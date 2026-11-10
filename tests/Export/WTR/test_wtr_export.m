function info = test_wtr_export()
%TEST_WTR_EXPORT WTR 完全由结果实验定义驱动、拒绝篡改与覆盖保护

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
scenario = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.3], ...
    'yaw_rate', 0.1, ...
    'max_records', 30000));
result = aero_sim(scenario.model, scenario.cfg);

root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
out = wtr_export(result, scenario.model, struct('output_root', root));
check_that(isfile(out.mat_path) && isfile(out.wtr_path), '导出文件缺失');
matHash = wtr_sha256(out.mat_path);
wtrHash = wtr_sha256(out.wtr_path);

report = wtr_validate(out.wtr_path);
check_that(report.passed, 'WTR 独立校验失败');
check_that(strcmp(report.run_id, out.run_id), 'WTR run_id 与导出结果不一致');

exp = result.config.experiment;
identity = wtr_experiment_identity(result, scenario.model);
check_that(strcmp(out.run_signature, identity.input_signature_sha256), ...
    '导出签名不是结果身份哈希');
check_that(strcmp(exp.definition_sha256, ...
    aero_sha256_text(jsonencode(struct('schema', exp.schema, 'definition', exp.definition)))), ...
    '实验定义 SHA256 不正确');
check_that(strcmp(exp.identity_sha256, identity.input_signature_sha256), ...
    '结果身份哈希与重新计算不一致');

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
check_that(max(abs(actualMotor - expectedMotor)) < 1e-9, ...
    '电机 PWM 时间轴未使用真实锁存时刻');

% ---- 参数追溯（全部来自定义/结果快照） ------------------------------
paths = parameter_paths(out.wtr_path);
required = {'vehicle.mass_kg', 'vehicle.inertia_frd_kg_m2', ...
    'aero.trust', 'aero.file_sha256', 'actuator.servo_device', ...
    'actuator.motor_device', 'control.position.Kp', ...
    'control.position.velocity_limit_xy', 'control.velocity.Kd', ...
    'control.velocity.hover_thrust_n', 'control.attitude.Kp', ...
    'control.attitude.yaw_weight', 'control.attitude.ref_model.omega_n', ...
    'control.rate.Kp', 'control.rate.saturation_tolerance', ...
    'allocation.algorithm', 'allocation.B_tau', 'allocation.trim_delta_rad', ...
    'allocation.ve_min_m_s', 'rates.position_hz', 'rates.velocity_hz', ...
    'rates.guidance_hz', 'solver.RelTol', 'solver.InitialStep', ...
    'experiment.definition_sha256', 'experiment.identity_sha256', ...
    'experiment.reference.position', 'experiment.reference.yaw', ...
    'experiment.reference.yaw_rate', 'experiment.scenario'};
missing = setdiff(required, paths);
check_that(isempty(missing), 'WTR 参数缺少：%s', strjoin(missing, ', '));

manifest = load_manifest(out.wtr_path);
check_that(strcmp(manifest.scenario, 'hover_hold'), 'manifest scenario 未从定义派生');
check_that(strcmp(manifest.method, 'P_PID_P_PID_PINV'), 'manifest method 未从定义派生');
check_that(isequal(manifest.seed, result.config.seed), 'manifest seed 未从结果派生');

% ---- 旧身份字段必须拒绝，且不创建目录 -------------------------------
identityRoot = tempname();
cleanupIdentity = onCleanup(@() remove_tree(identityRoot)); %#ok<NASGU>
check_error(@() wtr_export(result, scenario.model, struct( ...
    'output_root', identityRoot, 'signature', struct())), ...
    'wtr:export:identity', 'signature 字段未被拒绝');
check_error(@() wtr_export(result, scenario.model, struct( ...
    'output_root', identityRoot, 'gains', scenario.cfg.experiment.definition.controllers.position)), ...
    'wtr:export:identity', 'gains 字段未被拒绝');
check_that(~isfolder(identityRoot), '身份字段拒绝后留下目录');

% ---- 仿真结果被篡改：定义/运行配置/模型都必须拒绝 -------------------
badRoot = tempname();
cleanupBad = onCleanup(@() remove_tree(badRoot)); %#ok<NASGU>
bad = result;
bad.config.experiment.definition.controllers.rate.Kp(1) = ...
    bad.config.experiment.definition.controllers.rate.Kp(1) + 0.1;
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:definition', '篡改控制器参数未拒绝');

bad = result;
bad.config.experiment.definition.guidance.yaw_rate_rad_s = ...
    bad.config.experiment.definition.guidance.yaw_rate_rad_s + 0.1;
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:definition', '篡改参考值未拒绝');

bad = result;
bad.config.experiment.definition.allocation.l2_m = ...
    bad.config.experiment.definition.allocation.l2_m + 0.001;
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:definition', '篡改分配参数未拒绝');

bad = result;
bad.config.experiment.definition.scenario = 'rate_step';
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:definition', '把 HoverHold 标记为 BodyRate 未拒绝');

bad = result;
bad.config.tspan(2) = bad.config.tspan(2) + 0.1;
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:input', '篡改 tspan 未拒绝');

bad = result;
bad.config.x0(1) = bad.config.x0(1) + 0.1;
check_error(@() wtr_export(bad, scenario.model, struct('output_root', badRoot)), ...
    'wtr:identity:input', '篡改初始状态未拒绝');

badModel = scenario.model;
badModel.name = 'OTHER';
check_error(@() wtr_export(result, badModel, struct('output_root', badRoot)), ...
    'wtr:identity:model', '替换 model 名称未拒绝');

badModel = scenario.model;
badModel.metadata.mass_kg = badModel.metadata.mass_kg + 0.1;
check_error(@() wtr_export(result, badModel, struct('output_root', badRoot)), ...
    'wtr:identity:model', '替换 model 元数据未拒绝');

check_that(~isfolder(badRoot), '篡改拒绝后留下结果目录');

% ---- 容量治理路径（警告分支，不破坏文件） ---------------------------
warnRoot = tempname();
cleanup2 = onCleanup(@() remove_tree(warnRoot)); %#ok<NASGU>
policy = aero_results_policy(struct('warn_bytes', 1, 'limit_bytes', 10e9));
lastwarn('');
warnOut = wtr_export(result, scenario.model, struct( ...
    'output_root', warnRoot, 'policy', policy));
[~, warnId] = lastwarn();
check_that(strcmp(warnId, 'wtr:export:capacity'), ...
    'WTR 未执行 Results 容量检查（warning id=%s）', warnId);
check_that(isfile(warnOut.wtr_path), '容量警告后 WTR 未写出');

% ---- 覆盖保护：重复导出拒绝且不改变已有文件 -------------------------
check_error(@() wtr_export(result, scenario.model, struct('output_root', root)), ...
    'wtr:export:exists', '重复导出未拒绝');
check_that(strcmp(wtr_sha256(out.mat_path), matHash), '失败导出修改了原始 .mat');
check_that(strcmp(wtr_sha256(out.wtr_path), wtrHash), '失败导出修改了已有 .wtr');

% ---- 未启用发布日志的结果不能导出 -----------------------------------
noLog = scenario;
noLog.cfg.communication.log.enabled = false;
shortCfg = noLog.cfg;
shortCfg.tspan = [0, 0.2];
shortResult = aero_sim(noLog.model, shortCfg);
check_error(@() wtr_export(shortResult, noLog.model, ...
    struct('output_root', root)), 'wtr:export:log', '无日志结果被错误导出');

info = struct('passed', true, 'summary', sprintf( ...
    'WTR %d topics、身份/定义/参数追溯、篡改拒绝与覆盖保护通过', ...
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

function manifest = load_manifest(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
manifest = jsondecode(fileread(fullfile(stage, 'manifest.json')));
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
