function info = test_experiment_provenance()
%TEST_EXPERIMENT_PROVENANCE 唯一实验定义、身份哈希、MAT roundtrip 与运行配置

source = struct('type', 'fixture', 'tables', shc09_aero_fixture());
s = hover_scenario(struct( ...
    'aero_source', source, ...
    'tspan', [0, 0.2], ...
    'position_offset', [0.05; 0; 0], ...
    'yaw', 0.1, ...
    'yaw_rate', 0.05, ...
    'max_records', 20000));
cfg = s.cfg;
exp = cfg.experiment;
def = exp.definition;

check_that(strcmp(exp.schema, 'aero-experiment-definition/1'), '定义 schema 错误');
check_that(strcmp(def.scenario, 'hover_hold'), 'scenario 定义错误');
check_that(strcmp(def.method, 'P_PID_P_PID_PINV'), 'hover method 错误');
check_that(isequal(def.active_modules, ...
    {'guidance', 'position', 'velocity', 'attitude', 'rate', 'allocation'}), ...
    'hover active_modules 错误');
check_that(~has_handle(exp), '实验定义含函数/对象句柄');

expectedDefHash = aero_sha256_text(jsonencode(struct( ...
    'schema', exp.schema, 'definition', def)));
check_that(strcmp(exp.definition_sha256, expectedDefHash), '定义 SHA 不正确');

identity = aero_experiment_identity(exp, cfg, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(strcmp(exp.identity_sha256, identity.input_signature_sha256), ...
    '身份 SHA 不正确');

% 相同场景重复构造 → 相同定义/身份
s2 = hover_scenario(struct( ...
    'aero_source', source, 'tspan', [0, 0.2], ...
    'position_offset', [0.05; 0; 0], 'yaw', 0.1, 'yaw_rate', 0.05, ...
    'max_records', 20000));
check_that(strcmp(s2.cfg.experiment.definition_sha256, exp.definition_sha256), ...
    '相同配置的定义 SHA 不同');
check_that(strcmp(s2.cfg.experiment.identity_sha256, exp.identity_sha256), ...
    '相同配置的身份 SHA 不同');

% 修改有效参数 → 定义 SHA 改变
s3 = hover_scenario(struct( ...
    'aero_source', source, 'tspan', [0, 0.2], ...
    'position_offset', [0.05; 0; 0], 'yaw', 0.2, 'yaw_rate', 0.05, ...
    'max_records', 20000));
check_that(~strcmp(s3.cfg.experiment.definition_sha256, exp.definition_sha256), ...
    '修改 yaw 后定义 SHA 未改变');

% 控制器参数必须与定义一致
check_that(isequal(def.controllers.position.Kp, control_defaults().position.Kp), ...
    'Position 参数未从定义派生');

% Rate 场景：只激活实际模块
sr = rate_step_scenario(struct( ...
    'aero_source', source, 'tspan', [0, 0.2], 'max_records', 20000));
check_that(isequal(sr.cfg.experiment.definition.active_modules, ...
    {'guidance', 'rate', 'allocation'}), 'rate active_modules 错误');
check_that(~isfield(sr.cfg.experiment.definition.controllers, 'position'), ...
    'rate 场景错误包含 position 控制器');

% 运行结果保存同一实验定义
result = aero_sim(s.model, cfg);
check_that(isequaln(result.config.experiment, cfg.experiment), ...
    '结果未原样保存 cfg.experiment');
check_that(isequaln(result.model_metadata, s.model.metadata), ...
    '结果未保存实际模型快照');

% 运行配置/模型变化改变身份签名
cfgSeed = cfg; cfgSeed.seed = cfg.seed + 1;
idSeed = aero_experiment_identity(exp, cfgSeed, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idSeed.input_signature_sha256, identity.input_signature_sha256), ...
    'seed 变化未改变身份签名');
cfgTspan = cfg; cfgTspan.tspan(2) = cfgTspan.tspan(2) + 0.01;
idTspan = aero_experiment_identity(exp, cfgTspan, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idTspan.input_signature_sha256, identity.input_signature_sha256), ...
    'tspan 变化未改变身份签名');
cfgX0 = cfg; cfgX0.x0(1) = cfgX0.x0(1) + 0.01;
idX0 = aero_experiment_identity(exp, cfgX0, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idX0.input_signature_sha256, identity.input_signature_sha256), ...
    'x0 变化未改变身份签名');
cfgSolver = cfg; cfgSolver.solver.RelTol = cfgSolver.solver.RelTol / 2;
idSolver = aero_experiment_identity(exp, cfgSolver, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idSolver.input_signature_sha256, identity.input_signature_sha256), ...
    'solver 变化未改变身份签名');
cfgEnv = cfg; cfgEnv.environment.wind_ned_m_s = [1; 0; 0];
idEnv = aero_experiment_identity(exp, cfgEnv, s.model.metadata, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idEnv.input_signature_sha256, identity.input_signature_sha256), ...
    'environment 变化未改变身份签名');
cfgComp = cfg; cfgComp.components(1).period = cfgComp.components(1).period * 1.01;
idComp = aero_experiment_identity(exp, cfgComp, s.model.metadata, ...
    aero_experiment_components(cfgComp.components));
check_that(~strcmp(idComp.input_signature_sha256, identity.input_signature_sha256), ...
    '组件 period 变化未改变身份签名');
badModelMeta = s.model.metadata;
badModelMeta.mass_kg = badModelMeta.mass_kg + 0.1;
idModel = aero_experiment_identity(exp, cfg, badModelMeta, ...
    aero_experiment_components(cfg.components));
check_that(~strcmp(idModel.input_signature_sha256, identity.input_signature_sha256), ...
    '模型快照变化未改变身份签名');

% 真实 MAT save/load，再从加载结果导出 WTR
root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
saveCfg = struct('enabled', true, 'path', 'simulation.mat', 'estimated_bytes', 0);
savedPath = aero_save_result(result, saveCfg, root);
data = load(savedPath);
loaded = data.result;
check_that(isequaln(loaded.config.experiment, cfg.experiment), ...
    'MAT roundtrip 丢失实验定义');
check_that(strcmp(loaded.config.experiment.definition_sha256, ...
    exp.definition_sha256), 'MAT roundtrip 定义 SHA 变化');
check_that(isequaln(loaded.model_metadata, s.model.metadata), ...
    'MAT roundtrip 丢失模型快照');

wtrRoot = tempname();
cleanupWtr = onCleanup(@() remove_tree(wtrRoot)); %#ok<NASGU>
out = wtr_export(loaded, s.model, struct('output_root', wtrRoot));
check_that(isfile(out.wtr_path), '加载结果无法导出 WTR');
check_that(strcmp(out.run_signature, exp.identity_sha256), ...
    '加载结果导出签名与内存结果不一致');

info = struct('passed', true, 'summary', sprintf( ...
    '定义/身份 SHA、运行配置、模型快照、MAT roundtrip 与 WTR 导出通过；modules=%d', ...
    numel(def.active_modules)));

end

% ===========================================================================
function tf = has_handle(value)
tf = isa(value, 'function_handle') || isobject(value);
if isstruct(value)
    for k = 1:numel(value)
        names = fieldnames(value(k));
        for j = 1:numel(names)
            if has_handle(value(k).(names{j}))
                tf = true;
                return;
            end
        end
    end
elseif iscell(value)
    for k = 1:numel(value)
        if has_handle(value{k})
            tf = true;
            return;
        end
    end
end
end

function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
