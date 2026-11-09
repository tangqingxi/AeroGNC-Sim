function entries = wtr_build_parameters(model, options)
%WTR_BUILD_PARAMETERS 导出本次实验的生效参数（WindTrace parameters.v1）
%
% 只记录模型/控制实际使用的数值，不把未知字段补成默认值；数组按原值内联
% 保存（均小于 128 元素），不做类型改名或伪造 PX4 参数。

p = model.parameters;
rows = {};

add('vehicle.mass_kg', p.mass_kg, 'kg', 'shc09_parameters');
add('vehicle.gravity_m_s2', p.gravity_m_s2, 'm/s^2', 'shc09_parameters');
add('vehicle.inertia_frd_kg_m2', p.inertia_frd_kg_m2, 'kg*m^2', 'shc09_parameters');
add('vehicle.rotor_radius_m', p.rotor_radius_m, 'm', 'shc09_parameters');
add('actuator.servo_range_rad', p.servo_effective_range_rad, 'rad', 'assembly limit');
add('actuator.motor_range_rad_s', p.motor_device.omega_range_rad_s, 'rad/s', 'motor device');
add('duct.k_T0', p.duct.k_T0, 'N/(rad/s)^2', 'shc09_parameters');
add('control.trim_delta_rad', model.trim.delta_rad, 'rad', 'shc09_hover_equilibrium');
add('control.ve_nominal_m_s', model.trim.exit_speed_m_s, 'm/s', 'shc09_hover_equilibrium');

if isfield(options, 'gains') && isstruct(options.gains)
    g = options.gains;
    add('control.position.Kp', g.position.Kp, '1/s', 'gnc_control_defaults');
    add('control.velocity.Kp', g.velocity.Kp, '1/s', 'gnc_control_defaults');
    add('control.velocity.Ki', g.velocity.Ki, '1/s^2', 'gnc_control_defaults');
    add('control.velocity.Kd', g.velocity.Kd, 's', 'gnc_control_defaults');
    add('control.attitude.Kp', g.attitude.Kp, '1/s', 'gnc_control_defaults');
    add('control.rate.Kp', g.rate.Kp, 'N*m/(rad/s)', 'gnc_control_defaults');
    add('control.rate.Ki', g.rate.Ki, 'N*m/rad', 'gnc_control_defaults');
    add('control.rate.Kd', g.rate.Kd, 'N*m/(rad/s^2)', 'gnc_control_defaults');
    add('control.max_tilt_rad', g.max_tilt_rad, 'rad', 'gnc_control_defaults');
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
        if isscalar(value)
            item.type = 'float';
        else
            item.type = 'array';
        end
        item.value = double(value);
        item.unit = unitText;
        item.source = sourceText;
        item.overridden = false;
        rows{end + 1} = item; %#ok<AGROW>
    end
end
