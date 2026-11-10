function m = shc09_metadata(p, aero, trim)
% ...（原有说明）
if nargin < 3 || isempty(trim) || ~isfield(trim, 'delta_rad')
    trim = shc09_hover_equilibrium(p);
end
%SHC09_METADATA 生成可复现的模型元数据（通用结构，内核原样记录）
%
% 记录：
%   模型名称/版本/机体系/状态排列
%   质量与惯量（含来源说明）、重力与大气密度
%   几何与涵道系数、阻尼系数
%   舵机/电机设备型号与全部动态参数（含覆盖后的生效值）
%   装配参数：非对称舵面限制、生效行程、锁存频率
%   显式参数覆盖 p.overrides
%   气动数据：类型、文件、SHA256、**可信状态**与是否允许物理验证
%
% 可信状态说明：
%   trust = 'verified'   调用方显式声明数据可用于物理验证；
%           'unverified' 外部文件存在但来源未确认（默认）；
%           'synthetic'  合成测试系数，永远不得用于物理验证。
%   文件存在与文件哈希只证明“读到的是同一份字节”，不证明物理可信。
%
% 该结构不含函数句柄；由 aero_sim 原样写入 result.model_metadata。
% 字段命名是 SHC09 自己的约定，Simulation 不解释它们。

m = struct();
m.model_name = p.name;
m.model_version = p.version;
m.frame = p.frame;
layout = shc09_state_layout();
m.state_layout = struct( ...
    'nx', layout.nx, ...
    'position', layout.position, ...
    'velocity', layout.velocity, ...
    'quaternion', layout.quaternion, ...
    'omega', layout.omega, ...
    'servo', layout.servo, ...
    'motor', layout.motor, ...
    'units', {layout.units}, ...
    'description', layout.description);
m.units = 'm, m/s, -, rad/s, rad, rad/s';

m.mass_kg = p.mass_kg;
m.mass_provenance = p.mass_provenance;
m.inertia_frd_kg_m2 = p.inertia_frd_kg_m2;
m.inertia_provenance = p.inertia_provenance;
m.environment = struct( ...
    'gravity_m_s2', p.gravity_m_s2, ...
    'air_density_kg_m3', p.air_density_kg_m3);

m.geometry = struct( ...
    'rotor_radius_m', p.rotor_radius_m, ...
    'disc_area_m2', p.disc_area_m2, ...
    'diffuser_ratio', p.diffuser_ratio, ...
    'rotor_direction', p.rotor_direction, ...
    'fan_inertia_kg_m2', p.fan_inertia_kg_m2, ...
    'l1_m', p.l1_m, ...
    'l2_m', p.l2_m, ...
    'control_surface_angle_rad', p.control_surface_angle_rad);
m.duct_coefficients = p.duct;
m.damping = struct('k_my0', p.damping_k_my0, 'k_myv', p.damping_k_myv);
m.servo_torque_coefficient = p.servo_torque_coefficient;
m.fixed_vane_coefficient = p.fixed_vane_coefficient;
m.aero_domain_policy = p.aero_domain_policy;
m.trim = struct( ...
    'delta_rad', trim.delta_rad, ...
    'thrust_n', trim.thrust_n, ...
    'omega_rad_s', trim.omega_rad_s, ...
    'exit_speed_m_s', trim.exit_speed_m_s);

m.servo_device = p.servo_device;
m.motor_device = p.motor_device;
m.servo_assembly_limit_rad = p.servo_deflection_limit_rad;
m.servo_effective_range_rad = p.servo_effective_range_rad;
m.servo_pwm_rate_hz = p.servo_pwm_rate_hz;
m.servo_latch_rate_hz = p.servo_latch_rate_hz;
m.motor_latch_rate_hz = p.motor_latch_rate_hz;

m.overrides = p.overrides;

m.aero = struct();
m.aero.source_type = aero.source_type;
m.aero.source_file = aero.source_file;
m.aero.trust = aero.trust;
m.aero.physical_validation_allowed = aero.physical_validation_allowed;
m.aero.duct_domain_rad = aero.duct_domain_rad;
m.aero.wing_domain_rad = aero.wing_domain_rad;
m.aero.content_sha256 = aero_content_sha256(aero);
if strcmpi(aero.source_type, 'external') && ~isempty(aero.source_file)
    m.aero.file_sha256 = sha256_of_file(aero.source_file);
else
    m.aero.file_sha256 = '';
    m.aero.note = '合成测试系数（非物理气动数据），不得用于物理验证';
end

end

% ===========================================================================
function digest = aero_content_sha256(aero)
%AERO_CONTENT_SHA256 对气动系数内容（非路径）计算稳定 SHA256。
% 同一文件放在不同目录时 spline 系数/有效域相同，摘要相同；修改系数或
% 有效域则摘要改变。source_file 不参与。
payload = struct( ...
    'dt', aero.dt, 'dn', aero.dn, 'dm', aero.dm, ...
    'wl', aero.wl, 'wd', aero.wd, 'wm', aero.wm, ...
    'duct_domain_rad', aero.duct_domain_rad, ...
    'wing_domain_rad', aero.wing_domain_rad);
digest = aero_sha256_text(aero_canonical_json(payload));
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
