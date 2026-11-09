function info = test_shc09_outputs()
%TEST_SHC09_OUTPUTS 纯气动力、总作用力、比力与外部扰动的语义一致性

src = struct('type', 'fixture', 'file', '', 'tables', shc09_aero_fixture());
model = shc09_model(struct('aero_source', src));
p = model.parameters;
x = model.x0;

% ---- 无外部扰动：总力等于气动力 ------------------------------------
truth = shc09_outputs(0, x, p, model.aero, struct());
check_that(max(abs(truth.external_force_body_n)) == 0, '零扰动时外部力非零');
check_that(max(abs(truth.total_force_body_n - truth.aero_force_body_n)) < 1e-15, ...
    '零扰动时总力不等于气动力');
check_that(max(abs(truth.specific_force_body_m_s2 - truth.aero_force_body_n / p.mass_kg)) < 1e-15, ...
    '比力不等于总力除以质量');

% ---- 非零外部力与力矩 ----------------------------------------------
forceNed = [1; 2; -3];
torqueBody = [0.01; -0.02; 0.03];
env = struct('force_ned_n', forceNed, 'torque_body_nm', torqueBody);
truth = shc09_outputs(0, x, p, model.aero, env);

R = truth.rotation_nb;
expectedExternalBody = R' * forceNed;
expectedTotal = truth.aero_force_body_n + expectedExternalBody;

check_that(max(abs(truth.external_force_body_n - expectedExternalBody)) < 1e-12, ...
    '外部力未正确旋转到机体系');
check_that(max(abs(truth.total_force_body_n - expectedTotal)) < 1e-12, ...
    '总作用力不等于气动力 + 外部力');
check_that(max(abs(truth.specific_force_body_m_s2 - expectedTotal / p.mass_kg)) < 1e-12, ...
    '比力不等于总作用力除以质量（不应含重力）');
check_that(max(abs(truth.acceleration_ned_m_s2 - ...
    (R * expectedTotal / p.mass_kg + [0; 0; p.gravity_m_s2]))) < 1e-12, ...
    'NED 加速度未按总力 + 重力合成');

check_that(max(abs(truth.total_torque_body_nm - (truth.aero_torque_body_nm + torqueBody))) < 1e-12, ...
    '总力矩不等于气动力矩 + 外部力矩');
check_that(max(abs(truth.external_torque_body_nm - torqueBody)) < 1e-12, ...
    '外部力矩记录错误');

% ---- 比力与重力的区别：平飞配平时应近似为零 ------------------------
check_that(norm(truth.specific_force_body_m_s2) > 0, '非零总力下比力为零');

info = struct('passed', true, 'summary', sprintf( ...
    '气动力/外力/总力/比力/NED 加速度/力矩语义一致（外力量级 %.2f N）', norm(forceNed)));

end
