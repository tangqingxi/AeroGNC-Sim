function info = test_shc09_geometry()
%TEST_SHC09_GEOMETRY 验证 SHC09 六舵面几何、冻结效能矩阵与力矩映射

p = shc09_parameters();
g = shc09_control_surface_geometry(p);

expectedPhi = deg2rad(0:60:300);
check_that(max(abs(g.phi_rad - expectedPhi)) < 1e-12, ...
    '舵面方位角不是 0,60,...,300°');

% 位置与旋转轴
expectedPos = [p.l2_m * cos(expectedPhi); p.l2_m * sin(expectedPhi); p.l1_m * ones(1, 6)];
check_that(max(abs(g.position_m(:) - expectedPos(:))) < 1e-12, '舵面位置几何不正确');
check_that(max(abs(g.axis(3, :))) < 1e-15, '舵面旋转轴不应有 z 分量');

% 冻结的 B_M
B_frozen = [ -1, -1/2,  1/2, 1,  1/2, -1/2;
              0, -sqrt(3)/2, -sqrt(3)/2, 0, sqrt(3)/2, sqrt(3)/2;
              1,  1,    1,   1,  1,    1 ];
check_that(max(abs(g.B_M(:) - B_frozen(:))) < 1e-12, ...
    'B_M 与冻结定义不一致');

% 由 r_i × 正偏角力方向导出的列必须与 B_M 一致
arm = diag([p.l1_m, p.l1_m, p.l2_m]);
for i = 1:6
    torque = cross(g.position_m(:, i), g.force_dir(:, i));
    check_that(max(abs(torque - arm * g.B_M(:, i))) < 1e-12, ...
        '第 %d 个舵面的几何力矩与 B_M 不符', i);
end

% 秩与正交性
check_that(rank(g.B_M) == 3, 'B_M 秩不是 3');
gram = g.B_M * g.B_M.';
for i = 1:3
    for j = 1:3
        if i ~= j
            check_that(abs(gram(i, j)) < 1e-12, 'B_M B_M^T 非对角元素非零');
        end
    end
end

% 单舵正偏角：B_tau 列与几何力矩方向一致；随 Ve² 缩放
Ve = 10;
[M, B_tau, B_M] = shc09_control_surfaces(zeros(6, 1), Ve, p);
check_that(norm(M) == 0, '零舵角仍产生力矩');
check_that(max(abs(B_M(:) - B_frozen(:))) < 1e-12, 'B_tau 中的 B_M 不一致');

for i = 1:6
    delta = zeros(6, 1);
    delta(i) = deg2rad(1);
    torque = B_tau * delta;
    geometric = cross(g.position_m(:, i), p.servo_torque_coefficient * Ve^2 * g.force_dir(:, i)) * deg2rad(1);
    check_that(max(abs(torque - geometric)) < 1e-10, ...
        '第 %d 个舵面单舵力矩方向不一致', i);
end

[M10] = shc09_control_surfaces(deg2rad([1 0 0 0 0 0]), 10, p);
[M20] = shc09_control_surfaces(deg2rad([1 0 0 0 0 0]), 20, p);
check_that(max(abs(M20 - 4 * M10)) < 1e-12, '舵面力矩未随 Ve² 缩放');

info = struct('passed', true, 'summary', sprintf( ...
    '六舵几何/冻结 B_M/秩 3/正交/单舵方向/Ve² 缩放均通过（单舵 1° 力矩 %.4f N·m）', norm(M10)));

end
