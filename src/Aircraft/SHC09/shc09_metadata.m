function m = shc09_metadata(p, aero)
%SHC09_METADATA 生成可复现的模型元数据（通用结构，内核原样记录）
%
% 输出记录：
%   模型名称与版本、机体系、质量与惯量（含来源说明）
%   舵机/电机设备型号与全部动态参数（含覆盖后的生效值）
%   装配参数：生效舵面行程、锁存频率
%   显式参数覆盖 p.overrides
%   气动数据来源：类型、文件、是否物理数据、文件 SHA256（可核查标识）
%
% 该结构不含函数句柄；由 aero_sim 原样写入 result.model_metadata。
% 字段命名是 SHC09 自己的约定，Simulation 不解释它们。

m = struct();
m.model_name = p.name;
m.model_version = p.version;
m.frame = p.frame;
m.state_layout = struct('nx', 20, ...
    'order', 'p_n(3), v_n(3), q_bn(4), omega_b(3), delta(6), Omega(1)');
m.units = 'm, m/s, -, rad/s, rad, rad/s';

m.mass_kg = p.mass_kg;
m.mass_provenance = '参考模型记录值，未实机称重';
m.inertia_frd_kg_m2 = p.inertia_frd_kg_m2;
m.inertia_provenance = p.inertia_provenance;

m.servo_device = p.servo_device;
m.motor_device = p.motor_device;
m.servo_effective_range_rad = p.servo_effective_range_rad;
m.servo_assembly_limit_rad = p.servo_deflection_limit_rad;
m.servo_pwm_rate_hz = p.servo_pwm_rate_hz;
m.servo_latch_rate_hz = p.servo_latch_rate_hz;
m.motor_latch_rate_hz = p.motor_latch_rate_hz;

m.overrides = p.overrides;

m.aero = struct();
m.aero.source_type = aero.source_type;
m.aero.source_file = aero.source_file;
m.aero.is_physical = aero.is_physical;
if strcmpi(aero.source_type, 'external') && ~isempty(aero.source_file)
    m.aero.file_sha256 = sha256_of_file(aero.source_file);
    m.aero.duct_domain_rad = aero.duct_domain_rad;
    m.aero.wing_domain_rad = aero.wing_domain_rad;
else
    m.aero.file_sha256 = '';
    m.aero.note = '测试替身系数（非物理气动数据），不得用于物理验证';
end

end

% ===========================================================================
function digest = sha256_of_file(path)
fid = fopen(path, 'r');
if fid < 0
    error('shc09:metadata:aeroFile', '无法读取气动数据文件：%s', path);
end
bytes = fread(fid, inf, 'uint8=>uint8');
fclose(fid);
md = java.security.MessageDigest.getInstance('SHA-256');
digest = sprintf('%02x', typecast(md.digest(bytes), 'uint8'));
end
