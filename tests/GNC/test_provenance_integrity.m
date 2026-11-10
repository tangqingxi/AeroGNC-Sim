function info = test_provenance_integrity()
%TEST_PROVENANCE_INTEGRITY 跨路径身份、fixture 内容、状态布局和软件来源

% ---- 跨路径：相同气动内容放不同目录 → 相同身份 ---------------------
root = tempname();
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
mkdir(fullfile(root, 'a'));
mkdir(fullfile(root, 'b'));
duct_raw = [0 0 0 0; 1.0 0 0 0];
wing_raw = [-0.3 0 0 0; 1.2 0 0 0];
fileA = fullfile(root, 'a', 'aero.mat');
fileB = fullfile(root, 'b', 'aero.mat');
save(fileA, 'duct_raw', 'wing_raw');
save(fileB, 'duct_raw', 'wing_raw');
sourceA = struct('type', 'external', 'file', fileA);
sourceB = struct('type', 'external', 'file', fileB);
sA = hover_scenario(struct('aero_source', sourceA, 'tspan', [0, 0.05]));
sB = hover_scenario(struct('aero_source', sourceB, 'tspan', [0, 0.05]));
check_that(strcmp(sA.cfg.experiment.identity_sha256, ...
    sB.cfg.experiment.identity_sha256), ...
    '相同气动文件不同目录导致身份 SHA 不同');
check_that(strcmp(sA.model.metadata.aero.content_sha256, ...
    sB.model.metadata.aero.content_sha256), ...
    '相同气动内容不同路径导致内容 SHA 不同');
check_that(~strcmp(sA.model.metadata.aero.source_file, ...
    sB.model.metadata.aero.source_file), '原始模型快照未保留本地路径');
check_that(strcmp(sA.model.metadata.aero.file_sha256, ...
    sB.model.metadata.aero.file_sha256), '相同文件字节的 file_sha256 不同');

% ---- 修改外部文件内容 → 身份改变 --------------------------------
duct_raw(2, 2) = 0.1;
save(fileB, 'duct_raw', 'wing_raw');
sourceB2 = struct('type', 'external', 'file', fileB);
sB2 = hover_scenario(struct('aero_source', sourceB2, 'tspan', [0, 0.05]));
check_that(~strcmp(sA.cfg.experiment.identity_sha256, ...
    sB2.cfg.experiment.identity_sha256), ...
    '修改外部气动文件内容未改变身份 SHA');
check_that(~strcmp(sA.model.metadata.aero.content_sha256, ...
    sB2.model.metadata.aero.content_sha256), ...
    '修改外部气动文件内容未改变内容 SHA');

% ---- 修改 fixture 系数 → 身份改变 --------------------------------
f1 = shc09_aero_fixture();
f2 = shc09_aero_fixture();
f2.dt = spline(f2.duct_domain_rad, [0, 0.2]);
sf1 = hover_scenario(struct('aero_source', ...
    struct('type', 'fixture', 'tables', f1), 'tspan', [0, 0.05]));
sf2 = hover_scenario(struct('aero_source', ...
    struct('type', 'fixture', 'tables', f2), 'tspan', [0, 0.05]));
check_that(~strcmp(sf1.cfg.experiment.identity_sha256, ...
    sf2.cfg.experiment.identity_sha256), ...
    '修改 fixture 系数未改变身份 SHA');
check_that(~strcmp(sf1.model.metadata.aero.content_sha256, ...
    sf2.model.metadata.aero.content_sha256), ...
    '修改 fixture 系数未改变内容 SHA');

% ---- trust / 有效域 → 身份改变 ------------------------------------
sourceTrust = sourceA;
sourceTrust.trust = 'verified';
sTrust = hover_scenario(struct('aero_source', sourceTrust, 'tspan', [0, 0.05]));
check_that(~strcmp(sA.cfg.experiment.identity_sha256, ...
    sTrust.cfg.experiment.identity_sha256), 'trust 变化未改变身份 SHA');

fileDomain = fullfile(root, 'b', 'domain.mat');
duct_raw = [0 0 0 0; 1.4 0 0 0];
save(fileDomain, 'duct_raw', 'wing_raw');
sDomain = hover_scenario(struct('aero_source', ...
    struct('type', 'external', 'file', fileDomain), 'tspan', [0, 0.05]));
check_that(~strcmp(sA.cfg.experiment.identity_sha256, ...
    sDomain.cfg.experiment.identity_sha256), '有效域变化未改变身份 SHA');

% ---- 完整状态布局 ------------------------------------------------
layout = sf1.model.metadata.state_layout;
check_that(layout.nx == 20, 'state_layout.nx 错误');
check_that(isequal(layout.position, 1:3), 'position 索引错误');
check_that(isequal(layout.velocity, 4:6), 'velocity 索引错误');
check_that(isequal(layout.quaternion, 7:10), 'quaternion 索引错误');
check_that(isequal(layout.omega, 11:13), 'omega 索引错误');
check_that(isequal(layout.servo, 14:19), 'servo 索引错误');
check_that(isequal(layout.motor, 20), 'motor 索引错误');
check_that(iscell(layout.units) && numel(layout.units) == 6, 'units 不完整');
check_that(ischar(layout.description) && ~isempty(layout.description), ...
    'description 缺失');

result = aero_sim(sf1.model, sf1.cfg);
wtrRoot = tempname();
cleanupWtr = onCleanup(@() remove_tree(wtrRoot)); %#ok<NASGU>
out = wtr_export(result, sf1.model, struct('output_root', wtrRoot));
check_that(isfile(out.wtr_path), '状态布局结果无法导出 WTR');

badRoot = tempname();
cleanupBad = onCleanup(@() remove_tree(badRoot)); %#ok<NASGU>
badModel = sf1.model;
badModel.state_layout.quaternion = [1 2 3 4];
check_error(@() wtr_export(result, badModel, struct('output_root', badRoot)), ...
    'wtr:identity:layout', '篡改 model quaternion 索引未拒绝');
badModel = sf1.model;
badModel.state_layout.servo = 10:15;
check_error(@() wtr_export(result, badModel, struct('output_root', badRoot)), ...
    'wtr:identity:layout', '篡改 model servo 索引未拒绝');
badModel = sf1.model;
badModel.state_layout.motor = 19;
check_error(@() wtr_export(result, badModel, struct('output_root', badRoot)), ...
    'wtr:identity:layout', '篡改 model motor 索引未拒绝');
badResult = result;
badResult.model_metadata.state_layout.quaternion = [1 2 3 4];
check_error(@() wtr_export(badResult, sf1.model, struct('output_root', badRoot)), ...
    'wtr:identity:input', '篡改结果 quaternion 索引未拒绝');
check_that(~isfolder(badRoot), '状态布局拒绝后留下输出目录');

% ---- 软件来源：WTR v1 可兼容的 source 记录 ------------------------
manifest = load_manifest(out.wtr_path);
check_that(isfield(manifest.source, 'software'), 'manifest.source 缺少 software');
software = manifest.source.software;
check_that(strcmp(software.matlab_version, result.environment.matlab_version), ...
    'MATLAB version 未记录');
check_that(strcmp(software.matlab_release, result.environment.matlab_release), ...
    'MATLAB release 未记录');
check_that(strcmp(software.computer, result.environment.computer), ...
    'computer 未记录');
check_that(strcmp(software.git_commit, result.environment.git_commit) || ...
    strcmp(software.git_commit, 'unknown'), 'git commit 未记录');
check_that(isequal(logical(software.git_dirty), logical(result.environment.git_dirty)), ...
    'git dirty 标志未记录');
check_that(~contains(jsonencode(manifest.source.software), 'project_root'), ...
    '软件来源泄露 project_root');
if result.environment.git_dirty
    check_that(isfield(software, 'reproducibility_note') ...
        && contains(software.reproducibility_note, 'dirty'), ...
        'dirty 源码未标记不可精确复现');
end

paths = parameter_paths(out.wtr_path);
for required = {'software.matlab_version', 'software.matlab_release', ...
        'software.computer', 'software.git_commit', 'software.git_dirty'}
    check_that(any(strcmp(paths, required{1})), ...
        'WTR 参数缺少 %s', required{1});
end
report = wtr_validate(out.wtr_path);
check_that(report.passed, '含 software 来源的 WTR v1 校验失败');

info = struct('passed', true, 'summary', sprintf( ...
    '跨路径/内容身份、fixture、state_layout、software 来源通过；content=%s', ...
    sf1.model.metadata.aero.content_sha256(1:12)));

end

% ===========================================================================
function manifest = load_manifest(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
manifest = jsondecode(fileread(fullfile(stage, 'manifest.json')));
end

function paths = parameter_paths(path)
stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() remove_tree(stage)); %#ok<NASGU>
unzip(path, stage);
params = jsondecode(fileread(fullfile(stage, 'parameters.json')));
paths = {params.entries.path};
end

function remove_tree(path)
if isfolder(path)
    rmdir(path, 's');
end
end
