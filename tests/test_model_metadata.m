function info = test_model_metadata()
%TEST_MODEL_METADATA 结果记录完整生效参数、设备型号与气动数据标识

src = struct('type', 'fixture', 'file', '', 'tables', shc09_aero_fixture());
model = shc09_model(struct('aero_source', src, 'overrides', struct('mass_kg', 2.2)));
p = model.parameters;
check_that(p.mass_kg == 2.2, '参数覆盖未生效');
check_that(isfield(p.overrides, 'mass_kg'), '覆盖值未记录在 p.overrides');

cfg = aero_default_config();
cfg.tspan = [0, 0.1];
cfg.x0 = model.x0;
cfg.initial_command = model.initial_command;
cfg.solver = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
result = aero_sim(model, cfg);

check_that(isfield(result, 'model_metadata'), '结果缺少 model_metadata');
md = result.model_metadata;
check_that(strcmp(md.model_name, 'SHC09') && ~isempty(md.model_version), '模型名称/版本未记录');
check_that(md.mass_kg == 2.2, '记录质量 %.3f 不是覆盖后的生效值', md.mass_kg);
check_that(max(abs(md.inertia_frd_kg_m2(:) - p.inertia_frd_kg_m2(:))) < 1e-15, '惯量未记录');
check_that(strcmp(md.servo_device.device_name, 'KST X08H'), '舵机型号未记录');
check_that(strcmp(md.motor_device.device_name, 'MAD M6C06 320KV V2'), '电机型号未记录');
check_that(max(abs(md.servo_effective_range_rad - deg2rad([-40 40]))) < 1e-12, ...
    '生效舵面行程未记录');
check_that(md.overrides.mass_kg == 2.2, '覆盖值未写入元数据');
check_that(md.aero.is_physical == false, '测试系数被错误标记为物理数据');
check_that(isempty(md.aero.file_sha256), '测试系数不应有文件哈希');

% ---- 通用性：任意模型都可以提供 metadata，内核不解释字段 ----------
probe = scalar_integrator_model();
probe.metadata = struct('custom_tag', 'abc', 'value', 42);
cfg2 = aero_default_config();
cfg2.tspan = [0, 0.05];
cfg2.x0 = 0;
cfg2.initial_command = 0;
res2 = aero_sim(probe, cfg2);
check_that(isequal(res2.model_metadata, probe.metadata), ...
    '内核未原样记录自定义 metadata');

% ---- 外部气动数据：记录文件哈希 -----------------------------------
tmpFile = [tempname(), '.mat'];
duct_raw = [0 0 0 0; 2.2 0 0 0];
wing_raw = [-0.3 0 0 0; 2.2 0 0 0];
save(tmpFile, 'duct_raw', 'wing_raw');
cleanup = onCleanup(@() delete(tmpFile)); %#ok<NASGU>

aero = shc09_aero_data(struct('type', 'external', 'file', tmpFile, 'tables', []));
check_that(aero.is_physical == true, '外部数据未标记为物理数据');
extModel = shc09_model(struct('aero_source', ...
    struct('type', 'external', 'file', tmpFile, 'tables', [])));
check_that(strcmp(extModel.metadata.aero.file_sha256, file_hash(tmpFile)), ...
    '外部气动数据哈希与文件不一致');
check_that(numel(extModel.metadata.aero.file_sha256) == 64, '气动数据哈希格式错误');

info = struct('passed', true, 'summary', sprintf( ...
    '记录质量 %.1f kg、惯量、KST/MAD 型号、生效行程 ±%.0f°、覆盖值与气动哈希', ...
    md.mass_kg, rad2deg(md.servo_effective_range_rad(2))));

end

% ===========================================================================
function h = file_hash(path)
fid = fopen(path, 'r');
bytes = fread(fid, inf, 'uint8=>uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
h = sprintf('%02x', typecast(md.digest(bytes), 'uint8'));
end
