function info = test_project_layout()
%TEST_PROJECT_LAYOUT 全项目目录、入口、命名与路径治理检查

root = aero_project_root();
aero_setup();

% ---- 主要入口位置 ----------------------------------------------------
entries = { ...
    'aero_sim',              fullfile('src', 'Simulation', 'aero_sim.m'); ...
    'aero_integrate',        fullfile('src', 'Simulation', 'aero_integrate.m'); ...
    'shc09_model',           fullfile('src', 'Aircraft', 'SHC09', 'shc09_model.m'); ...
    'shc09_hover_equilibrium', fullfile('src', 'Aircraft', 'SHC09', 'shc09_hover_equilibrium.m'); ...
    'shc09_forces_and_moments', fullfile('src', 'Aircraft', 'SHC09', 'Aerodynamics', 'shc09_forces_and_moments.m'); ...
    'aero_servo_dynamics',   fullfile('src', 'Components', 'Actuators', 'Servo', 'aero_servo_dynamics.m'); ...
    'aero_motor_dynamics',   fullfile('src', 'Components', 'Actuators', 'Motor', 'aero_motor_dynamics.m')};

for k = 1:size(entries, 1)
    expected = fullfile(root, entries{k, 2});
    actual = which(entries{k, 1});
    check_that(same_path(actual, expected), ...
        '%s 的入口位置错误：%s（期望 %s）', entries{k, 1}, actual, expected);
end

% ---- 旧名称与重复实现不得存在 --------------------------------------
legacyFunctions = {'shc09_trim', 'shc09_aero_force', ...
    'gnc_position_velocity_step', 'gnc_attitude_step', 'gnc_rate_step', ...
    'gnc_allocate_pinv_step', 'gnc_hover_scenario', 'gnc_rate_step_scenario', ...
    'gnc_ideal_state_step', 'gnc_hover_hold_step', 'gnc_body_rate_step', ...
    'gnc_control_defaults', 'gnc_topic_specs'};
for k = 1:numel(legacyFunctions)
    check_that(isempty(which(legacyFunctions{k})), ...
        '旧接口 %s 仍可解析', legacyFunctions{k});
end

% 四级控制器目录与入口必须真实存在
controllerFiles = { ...
    fullfile('src', 'GNC', 'Control', 'Position', 'position_p.m'); ...
    fullfile('src', 'GNC', 'Control', 'Velocity', 'velocity_pid.m'); ...
    fullfile('src', 'GNC', 'Control', 'Attitude', 'attitude_p.m'); ...
    fullfile('src', 'GNC', 'Control', 'Rate', 'rate_pid.m'); ...
    fullfile('src', 'GNC', 'Control', 'Allocation', 'pinv_allocator.m')};
for k = 1:size(controllerFiles, 1)
    check_that(isfile(fullfile(root, controllerFiles{k, 1})), ...
        '缺少控制器文件：%s', controllerFiles{k, 1});
end

keyFunctions = {'aero_sim', 'aero_schedule', 'aero_validate_config', ...
    'shc09_model', 'shc09_dynamics', 'shc09_hover_equilibrium', ...
    'shc09_forces_and_moments', 'aero_servo_dynamics', 'aero_motor_dynamics', ...
    'position_p', 'velocity_pid', 'attitude_p', 'rate_pid', ...
    'pinv_allocator', 'constrain_xy', 'quat_error_default', ...
    'attitude_from_thrust_direction', 'hover_scenario', 'rate_step_scenario', ...
    'experiment_definition', 'aero_experiment_identity', ...
    'wtr_export', 'wtr_experiment_identity', 'wtr_validate'};
for k = 1:numel(keyFunctions)
    hits = which(keyFunctions{k}, '-all');
    check_that(numel(hits) == 1, ...
        '%s 存在 %d 处定义（路径遮蔽或重复实现）：%s', ...
        keyFunctions{k}, numel(hits), strjoin(hits, '; '));
end

% ---- V1.0：已删除的旧接口不得回归 ----------------------------------
legacyFunctions = {'aero_measurement'};
for k = 1:numel(legacyFunctions)
    check_that(isempty(which(legacyFunctions{k})), ...
        '旧接口 %s 仍可解析', legacyFunctions{k});
end

legacyPatterns = {'bus\.measurements', 'aero_measurement', 'is_physical', 'bus\.truth', 'gnc_'};
sourceFiles = dir(fullfile(root, 'src', '**', '*.m'));
for k = 1:numel(sourceFiles)
    text = fileread(fullfile(sourceFiles(k).folder, sourceFiles(k).name));
    for p = 1:numel(legacyPatterns)
        check_that(isempty(regexp(text, legacyPatterns{p}, 'once')), ...
            '生产代码 %s 仍含旧接口 %s', sourceFiles(k).name, legacyPatterns{p});
    end
end

defaultCfg = aero_default_config();
check_that(~isfield(defaultCfg.communication, 'enabled'), ...
    'V1.0 配置仍保留 communication.enabled 双模式开关');

% ---- 禁止按开发轮次命名的目录 --------------------------------------
forbidden = '^(phase\d+|step\d+|temp|newcode|old|backup|v\d+)$';
directories = dir(fullfile(root, 'src', '**'));
for k = 1:numel(directories)
    if ~directories(k).isdir
        continue;
    end
    name = directories(k).name;
    if strcmp(name, '.') || strcmp(name, '..')
        continue;
    end
    check_that(isempty(regexpi(name, forbidden, 'once')), ...
        '发现按轮次/临时命名的目录：%s', name);
end

% ---- 生产代码文件基名唯一（MATLAB 包内函数由命名空间区分） --------
files = dir(fullfile(root, 'src', '**', '*.m'));
folders = {files.folder};
inPackage = cellfun(@(p) contains(p, [filesep '+']), folders);
files = files(~inPackage);
baseNames = cell(1, numel(files));
for k = 1:numel(files)
    [~, baseNames{k}] = fileparts(files(k).name);
end
[uniqueNames, ~, groupIndex] = unique(lower(baseNames));
check_that(numel(uniqueNames) == numel(baseNames), ...
    'src/ 中存在重复文件基名');
counts = accumarray(groupIndex(:), 1);
duplicated = uniqueNames(counts > 1);
check_that(isempty(duplicated), '重复基名：%s', strjoin(duplicated, ', '));

% ---- 生产路径不得依赖测试目录 --------------------------------------
setupText = fileread(fullfile(root, 'aero_setup.m'));
check_that(isempty(regexp(setupText, '''fixtures''', 'once')), ...
    'aero_setup 无条件加载测试 fixtures');
check_that(isempty(regexpi(setupText, 'genpath', 'once')), ...
    'aero_setup 使用了 genpath');
check_that(isempty(regexp(setupText, '''tests''', 'once')), ...
    'aero_setup 加载了 tests 目录');

% ---- run_tests 必须覆盖所有测试文件 --------------------------------
runTestsText = fileread(fullfile(root, 'tests', 'run_tests.m'));
testFiles = dir(fullfile(root, 'tests', '**', 'test_*.m'));
check_that(numel(testFiles) >= 20, '测试文件数量异常：%d', numel(testFiles));
for k = 1:numel(testFiles)
    [~, name] = fileparts(testFiles(k).name);
    check_that(contains(runTestsText, name), ...
        '测试 %s 未登记到 run_tests', name);
end

info = struct('passed', true, 'summary', sprintf( ...
    '%d 个入口位置正确，无旧接口/旧名称/重复定义/轮次目录；%d 个测试文件均被 run_tests 覆盖', ...
    size(entries, 1), numel(testFiles)));

end

% ===========================================================================
function tf = same_path(a, b)
a = strrep(char(a), '/', filesep);
b = strrep(char(b), '/', filesep);
if ispc
    tf = strcmpi(a, b);
else
    tf = strcmp(a, b);
end
end
