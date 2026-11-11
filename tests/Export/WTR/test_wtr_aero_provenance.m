function info = test_wtr_aero_provenance()
%TEST_WTR_AERO_PROVENANCE 验证 WTR 气动内容与有效域只来自结果；无输入，输出测试结果。
root = tempname();
mkdir(root);
cleanup = onCleanup(@() remove_tree(root)); %#ok<NASGU>
f1 = shc09_aero_fixture();
f2 = f1;
f2.dt = spline(f1.duct_domain_rad, [0 0.2]);
f3 = f1;
f3.duct_domain_rad = [0 2.1];
f3.wing_domain_rad = [-0.3 2.1];
fixtures = {f1, f2, f3};
values = cell(1,3);
for k = 1:3
    s = hover_scenario(struct('aero_source', struct('type', 'fixture', ...
        'tables', fixtures{k}), 'tspan', [0 0.05]));
    values{k} = export_parameters(s, fullfile(root, sprintf('fixture%d', k)));
    check_that(numel(values{k}.content) == 64, 'synthetic 内容 SHA 为空或非法');
end
check_that(~strcmp(values{1}.content, values{2}.content), 'fixture 系数变化未传到 WTR');
check_that(isequal(values{3}.duct, f3.duct_domain_rad(:)) && ...
    isequal(values{3}.wing, f3.wing_domain_rad(:)), '有效域变化未传到 WTR');
% 相同外部文件原样复制到不同路径，不改变身份或导出内容。
duct_raw = [0 0 0 0; 2.1 0 0 0];
wing_raw = [-0.3 0 0 0; 2.1 0 0 0];
fileA = fullfile(root, 'a.mat');
fileB = fullfile(root, 'b.mat');
save(fileA, 'duct_raw', 'wing_raw');
copyfile(fileA, fileB);
a = hover_scenario(struct('aero_source', struct('type','external','file',fileA), ...
    'tspan', [0 0.05]));
b = hover_scenario(struct('aero_source', struct('type','external','file',fileB), ...
    'tspan', [0 0.05]));
va = export_parameters(a, fullfile(root, 'externalA'));
vb = export_parameters(b, fullfile(root, 'externalB'));
check_that(strcmp(a.cfg.experiment.identity_sha256,b.cfg.experiment.identity_sha256), ...
    '本地路径影响身份');
check_that(isequal(va,vb), '相同外部内容更换路径影响 WTR 气动参数');
info = struct('passed', true, 'summary', 'synthetic/external、系数 SHA、双有效域、跨路径、结果唯一来源通过');
end

function values = export_parameters(s, root)
result = aero_sim(s.model, s.cfg);
% 修改调用者参数，不得影响从结果快照导出的值。
s.model.parameters.aero_source.tables = struct();
out = wtr_export(result, s.model, struct('output_root', root));
report = wtr_validate(out.wtr_path);
check_that(report.passed, '独立 WTR v1 校验失败');
stage = fullfile(root, 'unpacked');
unzip(out.wtr_path, stage);
p = jsondecode(fileread(fullfile(stage, 'parameters.json')));
paths = {'aero.content_sha256', 'aero.duct_domain_rad', 'aero.wing_domain_rad'};
fields = {'content','duct','wing'};
for k = 1:3
    entry = p.entries(strcmp({p.entries.path}, paths{k}));
    check_that(isscalar(entry), 'WTR 缺少或重复参数 %s', paths{k});
    check_that(strcmp(entry.source, 'model_metadata.aero'), '气动来源不符');
    check_that(strcmp(entry.unit, ternary_unit(k)), '气动单位不符');
    if k == 1
        values.(fields{k}) = entry.value;
    else
        values.(fields{k}) = entry.value(:);
    end
    expected = result.model_metadata.aero.(erase(paths{k}, 'aero.'));
    check_that(isequal(entry.value(:), expected(:)), 'WTR 与结果快照不符');
end
end

function unit = ternary_unit(k)
if k == 1
    unit = '-';
else
    unit = 'rad';
end
end

function remove_tree(root)
if isfolder(root)
    rmdir(root, 's');
end
end
