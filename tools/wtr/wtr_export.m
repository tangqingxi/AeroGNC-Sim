function out = wtr_export(result, model, options)
%WTR_EXPORT 将一次 AeroGNC-Sim 结果导出为 WindTrace windtrace.flight.v1
%
% 用法：
%   out = wtr_export(result, model, struct('output_root', 'results'));
%
% 输入：
%   result  - aero_sim 结果；必须包含 result.config.experiment 与 model_metadata
%   model   - 仅用于已验证的状态布局；名称/版本/元数据必须与结果一致
%   options - output_root、run_id、save_simulation、policy；
%             不接受 gains/signature/method/seed/scenario 等身份字段
%
% 输出：
%   out - struct：run_id、directory、mat_path、wtr_path、sha256、members
%
% 边界：
%   - 身份、参数、参考和模型快照全部从 result 派生；
%   - simulation.mat 是权威原始结果；flight.wtr 是派生可视化交付；
%   - 导出失败只清理本次临时文件，不删除/覆盖已有 .mat 或 .wtr；
%   - 只使用已记录的历史（Topic 发布日志 + 结果状态），不用最新值拼造；
%   - 坐标已经是 NED/FRD，本函数不做任何 SHC09→FRD 变换。
%
% 依赖：MATLAB 基础功能 + Java（SHA-256）。

if nargin < 3
    options = struct();
end
forbidden = {'scenario', 'signature', 'method', 'seed', 'gains'};
for k = 1:numel(forbidden)
    if isfield(options, forbidden{k})
        error('wtr:export:identity', ...
            '实验身份字段 %s 必须从结果派生，禁止由调用者传入', forbidden{k});
    end
end
options = fill_defaults(options);

if ~isfield(result, 'communication') || ~result.communication.log.enabled
    error('wtr:export:log', '导出需要 cfg.communication.log.enabled=true');
end
log = result.communication.log;
if log.dropped ~= 0
    error('wtr:export:dropped', ...
        '发布日志丢弃了 %d 条记录，不能作为完整飞行历史导出', log.dropped);
end

% 身份/定义/模型/运行配置全部从 result 自身恢复并校验；不得信任调用者。
identity = wtr_experiment_identity(result, model);
scenario = sanitize_name(identity.scenario);
signatureHash = identity.input_signature_sha256;

% ---- 1. 身份与路径 ---------------------------------------------------
outputRoot = options.output_root;
if isempty(outputRoot)
    outputRoot = fullfile(aero_project_root(), 'results');
end
if isempty(options.run_id)
    run_id = sprintf('%s_%s', scenario, signatureHash(1:12));
else
    run_id = sanitize_name(options.run_id);
end
relative = fullfile(scenario, run_id);
directory = aero_results_path(outputRoot, relative);
if ~isfolder(directory)
    [ok, message] = mkdir(directory);
    if ~ok
        error('wtr:export:mkdir', '无法创建结果目录：%s', message);
    end
end
matRelative = fullfile(relative, 'simulation.mat');
matPath = aero_results_path(outputRoot, matRelative);
wtrPath = aero_results_path(outputRoot, fullfile(relative, 'flight.wtr'));
if isfile(wtrPath)
    error('wtr:export:exists', 'WTR 已存在，拒绝覆盖：%s', wtrPath);
end

% ---- 2. 组装 topics 与 specs（先验证历史完整） -----------------------
[topics, specs] = wtr_build_topics(result, log);

% ---- 3. 参数与几何（从结果快照派生，不读调用者 model.parameters） ----
parameters = wtr_build_parameters(result, identity);
geometry = wtr_geometry(result.model_metadata);

% ---- 4. 保存权威 .mat（存在则拒绝覆盖） -----------------------------
if options.save_simulation
    if isfile(matPath)
        error('wtr:export:exists', '仿真结果已存在，拒绝覆盖：%s', matPath);
    end
    saveCfg = struct('enabled', true, 'path', matRelative, 'estimated_bytes', 0);
    aero_save_result(result, saveCfg, outputRoot);
else
    matPath = '';
end

% ---- 5. 写容器 -------------------------------------------------------
stage = tempname();
mkdir(stage);
zipTmp = fullfile(directory, unique_temp_name('.zip'));
cleanup = onCleanup(@() clean_stage(stage, zipTmp)); %#ok<NASGU>

flight = struct();
flight.topics = topics;
flight.parameter_arrays = struct();
save(fullfile(stage, 'flight.mat'), '-struct', 'flight', '-v7');

parametersJson = struct('schema', 'windtrace.parameters.v1', ...
    'entries', parameters, 'unknown_fields', {{}});
write_text(fullfile(stage, 'parameters.json'), wtr_json(parametersJson));
write_text(fullfile(stage, 'geometry.json'), wtr_json(geometry));

members = containers.Map();
members('parameters.json') = member_info(fullfile(stage, 'parameters.json'));
members('flight.mat') = member_info(fullfile(stage, 'flight.mat'));
members('geometry.json') = member_info(fullfile(stage, 'geometry.json'));

manifest = struct();
manifest.schema = 'windtrace.flight.v1';
manifest.run_id = run_id;
manifest.method = identity.method;
manifest.seed = identity.seed;
manifest.scenario = scenario;
manifest.duration_s = result.time(end);
manifest.execution = struct('status', 'completed', ...
    'expected_duration_s', result.time(end));
manifest.audit = struct('original', struct('status', 'unknown'), ...
    'saved_checks', struct(), ...
    'policy', 'AeroGNC-Sim 内部检查；unknown 不等于通过，独立审计另存');
manifest.source = struct( ...
    'log_sha256', ternary_str(isempty(matPath), '', wtr_sha256(matPath)), ...
    'filename', ternary_str(isempty(matPath), '', 'simulation.mat'), ...
    'sidecar_sha256', struct(), ...
    'engine', 'AeroGNC-Sim', ...
    'input_signature', signatureHash, ...
    'input_signature_scope', 'experiment definition + runtime config + model snapshot', ...
    'software', software_record(result.environment), ...
    'records', struct('navigation', 'IdealState（真值派生的测试适配器，非 EKF）', ...
        'actuator_estimate', '锁存指令，非舵面反馈估计', ...
        'aero_trust', result.model_metadata.aero.trust, ...
        'aero_file_sha256', result.model_metadata.aero.file_sha256));
manifest.exporter = struct('name', 'AeroGNC-Sim wtr_export', 'version', '1.0.0');
manifest.topics = specs;
manifest.missing_topics = {};
manifest.members = members;
write_text(fullfile(stage, 'manifest.json'), wtr_json(manifest));

zip(zipTmp, {'manifest.json', 'parameters.json', 'flight.mat', 'geometry.json'}, stage);
entry = dir(zipTmp);
incoming = 0;
if ~isempty(entry)
    incoming = entry(1).bytes;
end
policy = aero_results_policy();
if ~isempty(options.policy)
    policy = options.policy;
end
capacity = aero_results_check(outputRoot, policy, incoming, zipTmp);
if strcmp(capacity.status, 'limit')
    error('wtr:export:limit', '%s', capacity.message);
end
if any(strcmp(capacity.status, {'warning', 'file-warning', 'file-limit'}))
    warning('wtr:export:capacity', '%s', capacity.message);
end
if isfile(wtrPath)
    error('wtr:export:exists', 'WTR 在写入期间出现，拒绝覆盖：%s', wtrPath);
end
[ok, message] = movefile(zipTmp, wtrPath);
if ~ok
    error('wtr:export:move', '无法完成 WTR 写入：%s', message);
end

out = struct();
out.run_id = run_id;
out.directory = directory;
out.mat_path = matPath;
out.wtr_path = wtrPath;
out.sha256 = wtr_sha256(wtrPath);
out.run_signature = signatureHash;
out.topic_count = numel(fieldnames(specs));

end

% ===========================================================================
function options = fill_defaults(options)
defaults = struct( ...
    'output_root', '', ...
    'run_id', '', ...
    'save_simulation', true, 'policy', []);
names = fieldnames(defaults);
for k = 1:numel(names)
    if ~isfield(options, names{k})
        options.(names{k}) = defaults.(names{k});
    end
end
end

function name = sanitize_name(text)
name = lower(regexprep(char(text), '[^A-Za-z0-9_]+', '_'));
name = regexprep(name, '^_+|_+$', '');
if isempty(name)
    name = 'run';
end
end

function clean_stage(stage, zipTmp)
if isfolder(stage)
    rmdir(stage, 's');
end
if isfile(zipTmp)
    delete(zipTmp);
end
end

function write_text(path, text)
fid = fopen(path, 'w', 'n', 'UTF-8');
if fid < 0
    error('wtr:export:io', '无法写入：%s', path);
end
fwrite(fid, unicode2native(text, 'UTF-8'), 'uint8');
fclose(fid);
end

function info = member_info(path)
d = dir(path);
info = struct('sha256', wtr_sha256(path), 'bytes', d(1).bytes);
end

function value = ternary_str(condition, a, b)
if condition
    value = a;
else
    value = b;
end
end

function s = software_record(env)
s = struct( ...
    'matlab_version', env.matlab_version, ...
    'matlab_release', env.matlab_release, ...
    'computer', env.computer, ...
    'git_commit', env.git_commit, ...
    'git_dirty', logical(env.git_dirty));
if s.git_dirty
    s.reproducibility_note = 'source tree dirty; commit SHA alone is not an exact reproduction';
end
end

% ===========================================================================
function name = unique_temp_name(extension)
try
    name = ['.aero-tmp-' char(java.util.UUID.randomUUID()) extension];
catch
    name = ['.aero-tmp-' num2str(randi(1e9)) extension];
end
end

% ===========================================================================
