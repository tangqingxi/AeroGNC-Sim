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
check_that(strcmp(md.mass_provenance, p.mass_provenance) && contains(md.mass_provenance, '实机'), ...
    '2.1 kg 质量的来源说明不正确：%s', md.mass_provenance);
check_that(abs(md.environment.gravity_m_s2 - p.gravity_m_s2) < 1e-12, '重力未记录');
check_that(abs(md.environment.air_density_kg_m3 - p.air_density_kg_m3) < 1e-15, '大气密度未记录');
check_that(abs(md.geometry.disc_area_m2 - p.disc_area_m2) < 1e-15, '桨盘面积未记录');
check_that(abs(md.geometry.l1_m - p.l1_m) < 1e-15 && abs(md.geometry.l2_m - p.l2_m) < 1e-15, ...
    '力臂未记录');
check_that(isfield(md, 'duct_coefficients') && isfield(md.duct_coefficients, 'k_T0'), ...
    '涵道系数未记录');
check_that(strcmp(md.aero.trust, 'synthetic'), '测试系数可信状态不是 synthetic');
check_that(md.aero.physical_validation_allowed == false, ...
    '合成系数被允许用于物理验证');
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
check_that(strcmp(aero.trust, 'unverified'), '外部数据默认可信状态应为 unverified');
check_that(aero.physical_validation_allowed == false, ...
    '来源未确认的外部文件不得自动允许物理验证');

% 显式声明 verified 才允许物理验证；非法状态必须报错
aeroVerified = shc09_aero_data(struct('type', 'external', 'file', tmpFile, ...
    'tables', [], 'trust', 'verified'));
check_that(aeroVerified.physical_validation_allowed == true, ...
    '显式 verified 未生效');
check_error(@() shc09_aero_data(struct('type', 'external', 'file', tmpFile, ...
    'tables', [], 'trust', 'trusted')), 'shc09:aero:trust', '非法可信状态');

extModel = shc09_model(struct('aero_source', ...
    struct('type', 'external', 'file', tmpFile, 'tables', [])));
check_that(strcmp(extModel.metadata.aero.file_sha256, file_hash(tmpFile)), ...
    '外部气动数据哈希与文件不一致');
check_that(numel(extModel.metadata.aero.file_sha256) == 64, '气动数据哈希格式错误');
check_that(extModel.metadata.aero.physical_validation_allowed == false, ...
    '外部文件默认被错误标记为可物理验证');

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
