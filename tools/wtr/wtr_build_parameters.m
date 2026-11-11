function entries = wtr_build_parameters(result, identity)
%WTR_BUILD_PARAMETERS 从结果快照导出完整生效参数（windtrace.parameters.v1）
%
% 数据来源仅为：
%   result.model_metadata（机型/执行器/气动）
%   result.config.experiment.definition（控制器/参考/分配/频率）
%   result.config（运行配置）
% 不再读取 options.gains 或调用者传入的 model.parameters。

m = result.model_metadata;
def = identity.experiment.definition;
cfg = result.config;
p = parameters_from_metadata(m);
overrides = p.overrides;
rows = {};

% ---- 机型与环境 ------------------------------------------------------
add('vehicle.mass_kg', p.mass_kg, 'kg', 'model_metadata.mass_kg');
add('vehicle.gravity_m_s2', p.gravity_m_s2, 'm/s^2', 'model_metadata.environment');
add('vehicle.inertia_frd_kg_m2', p.inertia_frd_kg_m2, 'kg*m^2', 'model_metadata.inertia');
add('vehicle.rotor_radius_m', p.rotor_radius_m, 'm', 'model_metadata.geometry');
add('vehicle.disc_area_m2', p.disc_area_m2, 'm^2', 'model_metadata.geometry');
add('vehicle.diffuser_ratio', p.diffuser_ratio, '1', 'model_metadata.geometry');
add('vehicle.air_density_kg_m3', p.air_density_kg_m3, 'kg/m^3', 'model_metadata.environment');
add('vehicle.fan_inertia_kg_m2', p.fan_inertia_kg_m2, 'kg*m^2', 'model_metadata.geometry');
add('duct.k_T0', p.duct.k_T0, 'N/(rad/s)^2', 'model_metadata.duct_coefficients');
add('duct.k_Th', p.duct.k_Th, 'N/(rad/s*m/s)', 'model_metadata.duct_coefficients');
add('duct.k_Ts', p.duct.k_Ts, 'N/(rad/s*m/s)', 'model_metadata.duct_coefficients');
add('duct.k_Ns', p.duct.k_Ns, 'N/(rad/s*m/s)', 'model_metadata.duct_coefficients');
add('duct.k_Q0', p.duct.k_Q0, 'N*m/(rad/s)^2', 'model_metadata.duct_coefficients');
add('duct.l_cpz', p.duct.l_cpz, 'm', 'model_metadata.duct_coefficients');
add('duct.k_cpx', p.duct.k_cpx, 'N*m/(rad/s*m/s)', 'model_metadata.duct_coefficients');
add('vehicle.fixed_vane_coefficient', p.fixed_vane_coefficient, 'N*m/(m/s)^2', 'model_metadata');
add('vehicle.damping_k_my0', p.damping_k_my0, 'N*m/(rad/s)', 'model_metadata.damping');
add('vehicle.damping_k_myv', p.damping_k_myv, 'N*m/(rad/s*m/s)', 'model_metadata.damping');
add('vehicle.rotor_direction', p.rotor_direction, '1', 'model_metadata.geometry');
add('vehicle.servo_deflection_limit_rad', p.servo_deflection_limit_rad, 'rad', 'model_metadata.servo_assembly_limit');
add('vehicle.servo_pwm_rate_hz', p.servo_pwm_rate_hz, 'Hz', 'model_metadata');
add('vehicle.servo_latch_rate_hz', p.servo_latch_rate_hz, 'Hz', 'model_metadata');
add('vehicle.motor_latch_rate_hz', p.motor_latch_rate_hz, 'Hz', 'model_metadata');

% ---- 气动来源与可信等级 ---------------------------------------------
add('aero.source_type', m.aero.source_type, '-', 'model_metadata.aero');
add('aero.trust', m.aero.trust, '-', 'model_metadata.aero');
add('aero.file_sha256', m.aero.file_sha256, '-', 'model_metadata.aero');
add('aero.content_sha256', m.aero.content_sha256, '-', 'model_metadata.aero');
add('aero.duct_domain_rad', m.aero.duct_domain_rad, 'rad', 'model_metadata.aero');
add('aero.wing_domain_rad', m.aero.wing_domain_rad, 'rad', 'model_metadata.aero');
add('aero.physical_validation_allowed', ...
    double(m.aero.physical_validation_allowed), '1', 'model_metadata.aero');

% ---- 执行器 ----------------------------------------------------------
add('actuator.servo_device', p.servo_device.device_name, '-', 'model_metadata.servo_device');
add('actuator.servo_tau_s', p.servo_device.tau_s, 's', 'model_metadata.servo_device');
add('actuator.servo_rate_limit_rad_s', p.servo_device.rate_limit_rad_s, 'rad/s', 'model_metadata.servo_device');
add('actuator.servo_effective_range_rad', p.servo_effective_range_rad, 'rad', 'model_metadata.servo_effective_range');
add('actuator.motor_device', p.motor_device.device_name, '-', 'model_metadata.motor_device');
add('actuator.motor_tau_s', p.motor_device.tau_s, 's', 'model_metadata.motor_device');
add('actuator.motor_range_rad_s', p.motor_device.omega_range_rad_s, 'rad/s', 'model_metadata.motor_device');
add('actuator.motor_stop_rad_s', p.motor_device.omega_stop_rad_s, 'rad/s', 'model_metadata.motor_device');

% ---- 控制参数（来自 experiment.definition） --------------------------
if isfield(def.controllers, 'position')
    g = def.controllers.position;
    add('control.position.Kp', g.Kp, '1/s', 'experiment.definition.controllers.position');
    add('control.position.velocity_limit_xy', g.velocity_limit_xy, 'm/s', 'experiment.definition.controllers.position');
    add('control.position.velocity_limit_up', g.velocity_limit_up, 'm/s', 'experiment.definition.controllers.position');
    add('control.position.velocity_limit_down', g.velocity_limit_down, 'm/s', 'experiment.definition.controllers.position');
end
if isfield(def.controllers, 'velocity')
    g = def.controllers.velocity;
    add('control.velocity.Kp', g.Kp, '1/s', 'experiment.definition.controllers.velocity');
    add('control.velocity.Ki', g.Ki, '1/s^2', 'experiment.definition.controllers.velocity');
    add('control.velocity.Kd', g.Kd, '1', 'experiment.definition.controllers.velocity');
    add('control.velocity.integral_limit', g.integral_limit, 'm/s^2', 'experiment.definition.controllers.velocity');
    add('control.velocity.max_thrust_n', g.max_thrust_n, 'N', 'experiment.definition.controllers.velocity');
    add('control.velocity.hover_thrust_n', g.hover_thrust_n, 'N', 'experiment.definition.controllers.velocity');
    add('control.velocity.min_thrust_n', g.min_thrust_n, 'N', 'experiment.definition.controllers.velocity');
    add('control.velocity.horizontal_thrust_margin_n', ...
        g.horizontal_thrust_margin_n, 'N', 'experiment.definition.controllers.velocity');
    add('control.velocity.max_tilt_rad', g.max_tilt_rad, 'rad', 'experiment.definition.controllers.velocity');
end
if isfield(def.controllers, 'attitude')
    g = def.controllers.attitude;
    add('control.attitude.Kp', g.Kp, '1/s', 'experiment.definition.controllers.attitude');
    add('control.attitude.rate_limit', g.rate_limit, 'rad/s', 'experiment.definition.controllers.attitude');
    add('control.attitude.yaw_weight', g.yaw_weight, '1', 'experiment.definition.controllers.attitude');
    add('control.attitude.ref_model.enabled', double(g.ref_model.enabled), '1', 'experiment.definition.controllers.attitude');
    add('control.attitude.ref_model.omega_n', g.ref_model.omega_n, 'rad/s', 'experiment.definition.controllers.attitude');
    add('control.attitude.ref_model.ff_gain', g.ref_model.ff_gain, '1', 'experiment.definition.controllers.attitude');
    add('control.attitude.ref_model.ff_limit', g.ref_model.ff_limit, 'rad/s', 'experiment.definition.controllers.attitude');
end
g = def.controllers.rate;
add('control.rate.Kp', g.Kp, 'N*m/(rad/s)', 'experiment.definition.controllers.rate');
add('control.rate.Ki', g.Ki, 'N*m/rad', 'experiment.definition.controllers.rate');
add('control.rate.Kd', g.Kd, 'N*m/(rad/s^2)', 'experiment.definition.controllers.rate');
add('control.rate.Kff', g.Kff, 'N*m/(rad/s)', 'experiment.definition.controllers.rate');
add('control.rate.integral_limit', g.integral_limit, 'N*m', 'experiment.definition.controllers.rate');
add('control.rate.saturation_tolerance', g.saturation_tolerance, 'N*m', 'experiment.definition.controllers.rate');

% ---- 分配与配平 ------------------------------------------------------
a = def.allocation;
add('allocation.algorithm', a.algorithm, '-', 'experiment.definition.allocation');
add('allocation.B_M', a.B_M, '1', 'experiment.definition.allocation');
add('allocation.B_tau', a.B_tau, 'N*m/rad', 'experiment.definition.allocation');
add('allocation.servo_torque_coefficient', a.servo_torque_coefficient, 'N*m/rad', 'experiment.definition.allocation');
add('allocation.l1_m', a.l1_m, 'm', 'experiment.definition.allocation');
add('allocation.l2_m', a.l2_m, 'm', 'experiment.definition.allocation');
add('allocation.trim_delta_rad', a.trim_delta_rad, 'rad', 'experiment.definition.allocation');
add('allocation.delta_limit_rad', a.delta_limit_rad, 'rad', 'experiment.definition.allocation');
add('allocation.ve_min_m_s', a.ve_min_m_s, 'm/s', 'experiment.definition.allocation');
add('allocation.hover_thrust_n', a.hover_thrust_n, 'N', 'experiment.definition.allocation');
add('allocation.max_thrust_n', a.max_thrust_n, 'N', 'experiment.definition.allocation');
add('allocation.min_thrust_n', a.min_thrust_n, 'N', 'experiment.definition.allocation');
add('allocation.motor_min_rad_s', a.motor_min_rad_s, 'rad/s', 'experiment.definition.allocation');
add('allocation.motor_max_rad_s', a.motor_max_rad_s, 'rad/s', 'experiment.definition.allocation');
add('allocation.motor_model', a.motor_model, '-', 'experiment.definition.allocation');

% ---- 频率、求解器、场景 ---------------------------------------------
r = def.rates;
add('rates.position_hz', r.position_hz, 'Hz', 'experiment.definition.rates');
add('rates.velocity_hz', r.velocity_hz, 'Hz', 'experiment.definition.rates');
add('rates.guidance_hz', r.guidance_hz, 'Hz', 'experiment.definition.rates');
add('rates.attitude_hz', r.attitude_hz, 'Hz', 'experiment.definition.rates');
add('rates.rate_hz', r.rate_hz, 'Hz', 'experiment.definition.rates');
add('rates.servo_latch_hz', r.servo_latch_hz, 'Hz', 'experiment.definition.rates');
add('rates.motor_latch_hz', r.motor_latch_hz, 'Hz', 'experiment.definition.rates');
add('rates.sensor_hz', r.sensor_hz, 'Hz', 'experiment.definition.rates');
add('rates.navigation_hz', r.navigation_hz, 'Hz', 'experiment.definition.rates');
add('solver.RelTol', cfg.solver.RelTol, '1', 'result.config.solver');
add('solver.AbsTol', cfg.solver.AbsTol, '1', 'result.config.solver');
add('solver.MaxStep', cfg.solver.MaxStep, 's', 'result.config.solver');
if isempty(cfg.solver.InitialStep)
    add('solver.InitialStep', 'auto', '-', 'result.config.solver');
else
    add('solver.InitialStep', cfg.solver.InitialStep, 's', 'result.config.solver');
end
add('experiment.scenario', identity.scenario, '-', 'experiment.definition');
add('experiment.method', identity.method, '-', 'experiment.definition');
add('experiment.seed', cfg.seed, '1', 'result.config.seed');
add('experiment.tspan', cfg.tspan, 's', 'result.config.tspan');
add('experiment.model_version', m.model_version, '-', 'model_metadata');
add('experiment.definition_sha256', identity.experiment.definition_sha256, '-', 'experiment.definition');
add('experiment.identity_sha256', identity.input_signature_sha256, '-', 'experiment.identity');

% ---- 软件执行环境（不属于实验输入签名） -----------------------------
env = result.environment;
add('software.matlab_version', env.matlab_version, '-', 'result.environment');
add('software.matlab_release', env.matlab_release, '-', 'result.environment');
add('software.computer', env.computer, '-', 'result.environment');
add('software.git_commit', env.git_commit, '-', 'result.environment');
add('software.git_dirty', double(env.git_dirty), '1', 'result.environment');
if env.git_dirty
    add('software.reproducibility_note', ...
        'source tree dirty; commit SHA alone is not an exact reproduction', ...
        '-', 'result.environment');
end

% ---- 参考条件（来自 definition.guidance） ---------------------------
guidance = def.guidance;
switch guidance.type
    case 'HoverHold'
        add('experiment.reference.position', guidance.position_m, 'm', 'experiment.definition.guidance');
        add('experiment.reference.velocity', guidance.velocity_m_s, 'm/s', 'experiment.definition.guidance');
        add('experiment.reference.acceleration', guidance.acceleration_m_s2, 'm/s^2', 'experiment.definition.guidance');
        add('experiment.reference.yaw', guidance.yaw_rad, 'rad', 'experiment.definition.guidance');
        add('experiment.reference.yaw_rate', guidance.yaw_rate_rad_s, 'rad/s', 'experiment.definition.guidance');
    case 'VelocityStep'
        add('experiment.reference.velocity_reference_ned_m_s', ...
            guidance.velocity_reference_ned_m_s, 'm/s', 'experiment.definition.guidance');
        add('experiment.reference.acceleration_ff_ned_m_s2', ...
            guidance.acceleration_ff_ned_m_s2, 'm/s^2', 'experiment.definition.guidance');
        add('experiment.reference.yaw_rad', guidance.yaw_rad, 'rad', 'experiment.definition.guidance');
        add('experiment.reference.yaw_rate_rad_s', guidance.yaw_rate_rad_s, 'rad/s', 'experiment.definition.guidance');
        add('experiment.reference.start_s', guidance.start_s, 's', 'experiment.definition.guidance');
        add('experiment.reference.end_s', guidance.end_s, 's', 'experiment.definition.guidance');
    case 'BodyRateStep'
        add('experiment.reference.axis', guidance.axis, '1', 'experiment.definition.guidance');
        add('experiment.reference.amplitude', guidance.amplitude_rad_s, 'rad/s', 'experiment.definition.guidance');
        add('experiment.reference.start_s', guidance.start_s, 's', 'experiment.definition.guidance');
        add('experiment.reference.end_s', guidance.end_s, 's', 'experiment.definition.guidance');
        add('experiment.reference.alpha_ff', guidance.alpha_ff_rad_s2, 'rad/s^2', 'experiment.definition.guidance');
end

% ---- 显式参数覆盖（如有） -------------------------------------------
names = fieldnames(overrides);
for k = 1:numel(names)
    add(['overrides.' names{k}], overrides.(names{k}), '-', 'explicit override');
end

entries = struct('path', cell(1, numel(rows)), 'value', cell(1, numel(rows)), ...
    'type', cell(1, numel(rows)), 'unit', cell(1, numel(rows)), ...
    'source', cell(1, numel(rows)), 'overridden', cell(1, numel(rows)));
for k = 1:numel(rows)
    entries(k) = rows{k};
end

    function add(path, value, unitText, sourceText)
        item = struct();
        item.path = path;
        if ischar(value) || isstring(value)
            item.type = 'string';
            item.value = char(value);
        elseif isscalar(value)
            item.type = 'float';
            item.value = double(value);
        else
            item.type = 'array';
            item.value = double(value);
        end
        item.unit = unitText;
        item.source = sourceText;
        item.overridden = isfield(overrides, strrep(path, 'overrides.', ''));
        rows{end + 1} = item; %#ok<AGROW>
    end
end

% ===========================================================================
function p = parameters_from_metadata(m)
p = struct();
p.mass_kg = m.mass_kg;
p.gravity_m_s2 = m.environment.gravity_m_s2;
p.air_density_kg_m3 = m.environment.air_density_kg_m3;
p.inertia_frd_kg_m2 = m.inertia_frd_kg_m2;
p.rotor_radius_m = m.geometry.rotor_radius_m;
p.disc_area_m2 = m.geometry.disc_area_m2;
p.diffuser_ratio = m.geometry.diffuser_ratio;
p.fan_inertia_kg_m2 = m.geometry.fan_inertia_kg_m2;
p.rotor_direction = m.geometry.rotor_direction;
p.l1_m = m.geometry.l1_m;
p.l2_m = m.geometry.l2_m;
p.control_surface_angle_rad = m.geometry.control_surface_angle_rad;
p.duct = m.duct_coefficients;
p.damping_k_my0 = m.damping.k_my0;
p.damping_k_myv = m.damping.k_myv;
p.servo_torque_coefficient = m.servo_torque_coefficient;
p.fixed_vane_coefficient = m.fixed_vane_coefficient;
p.servo_device = m.servo_device;
p.motor_device = m.motor_device;
p.servo_effective_range_rad = m.servo_effective_range_rad;
p.servo_deflection_limit_rad = m.servo_assembly_limit_rad;
p.servo_pwm_rate_hz = m.servo_pwm_rate_hz;
p.servo_latch_rate_hz = m.servo_latch_rate_hz;
p.motor_latch_rate_hz = m.motor_latch_rate_hz;
p.aero_source = struct('type', m.aero.source_type, 'file', m.aero.source_file, 'tables', []);
p.aero_domain_policy = m.aero_domain_policy;
p.overrides = m.overrides;
p.version = m.model_version;
p.name = m.model_name;
end
