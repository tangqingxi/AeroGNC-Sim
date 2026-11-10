function entries = wtr_build_parameters(model, options)
%WTR_BUILD_PARAMETERS 导出本次实验的完整生效参数（WindTrace parameters.v1）
%
% 只记录模型/控制实际使用的数值与来源，不把未知字段补成默认值，也不把
% 名义值标成实测值。字符串参数（设备型号、气动来源/可信等级）以文本记录。

p = model.parameters;
sig = options.signature;
overrides = model.parameters.overrides;
rows = {};

% ---- 机型与环境 ------------------------------------------------------
add('vehicle.mass_kg', p.mass_kg, 'kg', 'shc09_parameters');
add('vehicle.gravity_m_s2', p.gravity_m_s2, 'm/s^2', 'shc09_parameters');
add('vehicle.inertia_frd_kg_m2', p.inertia_frd_kg_m2, 'kg*m^2', 'shc09_parameters');
add('vehicle.rotor_radius_m', p.rotor_radius_m, 'm', 'shc09_parameters');
add('vehicle.disc_area_m2', p.disc_area_m2, 'm^2', 'shc09_parameters');
add('vehicle.diffuser_ratio', p.diffuser_ratio, '1', 'shc09_parameters');
add('vehicle.air_density_kg_m3', p.air_density_kg_m3, 'kg/m^3', 'shc09_parameters');
add('vehicle.fan_inertia_kg_m2', p.fan_inertia_kg_m2, 'kg*m^2', 'shc09_parameters');
add('duct.k_T0', p.duct.k_T0, 'N/(rad/s)^2', 'shc09_parameters');
add('duct.k_Th', p.duct.k_Th, 'N/(rad/s*m/s)', 'shc09_parameters');
add('duct.k_Ts', p.duct.k_Ts, 'N/(rad/s*m/s)', 'shc09_parameters');
add('duct.k_Ns', p.duct.k_Ns, 'N/(rad/s*m/s)', 'shc09_parameters');
add('duct.k_Q0', p.duct.k_Q0, 'N*m/(rad/s)^2', 'shc09_parameters');
add('duct.l_cpz', p.duct.l_cpz, 'm', 'shc09_parameters');

% ---- 气动来源与可信等级 ---------------------------------------------
add('aero.source_type', p.aero_source.type, '-', 'shc09_parameters.aero_source');
add('aero.trust', model.metadata.aero.trust, '-', 'shc09_metadata');
add('aero.file_sha256', model.metadata.aero.file_sha256, '-', 'shc09_metadata');
add('aero.physical_validation_allowed', ...
    double(model.aero.physical_validation_allowed), '1', 'shc09_metadata');

% ---- 执行器 ----------------------------------------------------------
add('actuator.servo_device', p.servo_device.device_name, '-', 'KST device');
add('actuator.servo_tau_s', p.servo_device.tau_s, 's', 'KST nominal');
add('actuator.servo_rate_limit_rad_s', p.servo_device.rate_limit_rad_s, 'rad/s', 'KST nominal');
add('actuator.servo_effective_range_rad', p.servo_effective_range_rad, 'rad', 'assembly limit');
add('actuator.motor_device', p.motor_device.device_name, '-', 'MAD device');
add('actuator.motor_tau_s', p.motor_device.tau_s, 's', 'MAD nominal');
add('actuator.motor_range_rad_s', p.motor_device.omega_range_rad_s, 'rad/s', 'MAD nominal');
add('actuator.motor_stop_rad_s', p.motor_device.omega_stop_rad_s, 'rad/s', 'MAD definition');

% ---- 控制参数 --------------------------------------------------------
g = options.gains;
add('control.position.Kp', g.position.Kp, '1/s', 'control_defaults');
add('control.velocity.Kp', g.velocity.Kp, '1/s', 'control_defaults');
add('control.velocity.Ki', g.velocity.Ki, '1/s^2', 'control_defaults');
add('control.velocity.Kd', g.velocity.Kd, 's', 'control_defaults');
add('control.velocity.integral_limit', g.velocity.integral_limit, 'm/s^2', 'control_defaults');
add('control.velocity.limit_xy', g.velocity.limit_xy, 'm/s', 'control_defaults');
add('control.velocity.limit_up', g.velocity.limit_up, 'm/s', 'control_defaults');
add('control.velocity.limit_down', g.velocity.limit_down, 'm/s', 'control_defaults');
add('control.acceleration_limit', g.acceleration_limit, 'm/s^2', 'control_defaults');
add('control.max_tilt_rad', g.max_tilt_rad, 'rad', 'control_defaults');
add('control.attitude.Kp', g.attitude.Kp, '1/s', 'control_defaults');
add('control.attitude.rate_limit', g.attitude.rate_limit, 'rad/s', 'control_defaults');
add('control.attitude.omega_ff', g.attitude.omega_ff, 'rad/s', 'control_defaults');
add('control.rate.Kp', g.rate.Kp, 'N*m/(rad/s)', 'control_defaults');
add('control.rate.Ki', g.rate.Ki, 'N*m/rad', 'control_defaults');
add('control.rate.Kd', g.rate.Kd, 'N*m/(rad/s^2)', 'control_defaults');
add('control.rate.Kff', g.rate.Kff, 'N*m/(rad/s)', 'control_defaults');
add('control.rate.integral_limit', g.rate.integral_limit, 'N*m', 'control_defaults');
add('control.rate.torque_limit', g.rate.torque_limit, 'N*m', 'control_defaults');

% ---- 分配与配平 ------------------------------------------------------
geometry = shc09_control_surface_geometry(p);
add('allocation.B_M', geometry.B_M, '1', 'shc09_control_surface_geometry');
add('allocation.servo_torque_coefficient', p.servo_torque_coefficient, 'N*m/rad', 'shc09_parameters');
add('allocation.l1_m', p.l1_m, 'm', 'shc09_parameters');
add('allocation.l2_m', p.l2_m, 'm', 'shc09_parameters');
add('allocation.trim_delta_rad', model.trim.delta_rad, 'rad', 'shc09_hover_equilibrium');
add('allocation.trim_omega_rad_s', model.trim.omega_rad_s, 'rad/s', 'shc09_hover_equilibrium');
add('allocation.trim_ve_m_s', model.trim.exit_speed_m_s, 'm/s', 'shc09_hover_equilibrium');

% ---- 频率、求解器、场景 ---------------------------------------------
add('rates.position_hz', g.rates.position_hz, 'Hz', 'control_defaults');
add('rates.velocity_hz', g.rates.velocity_hz, 'Hz', 'control_defaults');
add('rates.attitude_hz', g.rates.attitude_hz, 'Hz', 'control_defaults');
add('rates.rate_hz', g.rates.rate_hz, 'Hz', 'control_defaults');
add('rates.servo_latch_hz', g.rates.servo_latch_hz, 'Hz', 'control_defaults');
add('rates.motor_latch_hz', g.rates.motor_latch_hz, 'Hz', 'control_defaults');
add('rates.sensor_hz', g.rates.sensor_hz, 'Hz', 'control_defaults');
add('rates.navigation_hz', g.rates.navigation_hz, 'Hz', 'control_defaults');
add('solver.RelTol', sig.solver.RelTol, '1', 'cfg.solver');
add('solver.AbsTol', sig.solver.AbsTol, '1', 'cfg.solver');
add('solver.MaxStep', sig.solver.MaxStep, 's', 'cfg.solver');
add('experiment.scenario', sig.scenario, '-', 'scenario');
add('experiment.method', sig.method, '-', 'scenario');
add('experiment.seed', sig.seed, '1', 'cfg.seed');
add('experiment.tspan', sig.tspan, 's', 'cfg.tspan');
add('experiment.model_version', sig.model.version, '-', 'model');

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
