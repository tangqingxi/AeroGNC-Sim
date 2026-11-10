function run_tests(pattern)
%RUN_TESTS 运行 AeroGNC-Sim 回归测试集（唯一测试入口）
%
% 用法：
%   run_tests              运行全部测试
%   run_tests('multirate') 只运行名称包含该片段的测试
%
% 路径：
%   生产代码由 aero_setup 登记；测试辅助、fixtures 与各测试分组目录由本函数
%   显式加入路径。生产入口不加载 tests/。
%
% 完整性：
%   本函数在运行前核对“登记的测试名”与“磁盘上的 test_*.m 文件”完全一致，
%   防止迁移目录后测试被静默遗漏或出现未登记测试。
%
% 任一测试失败时抛出 aero:tests:failed，使
% `matlab -batch "run('tests/run_tests.m')"` 返回非零退出码。
% 被跳过的测试（例如系统不允许创建 Junction）单独计数，不伪装成通过。

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);
aero_setup();

groupDirectories = { ...
    here, ...
    fullfile(here, 'Simulation', 'Core'), ...
    fullfile(here, 'Simulation', 'Events'), ...
    fullfile(here, 'Simulation', 'Communication'), ...
    fullfile(here, 'Simulation', 'Results'), ...
    fullfile(here, 'Aircraft', 'SHC09'), ...
    fullfile(here, 'GNC'), ...
    fullfile(here, 'Export', 'WTR')};
supportDirectories = {fullfile(here, 'helpers'), fullfile(here, 'fixtures')};

for k = 1:numel(supportDirectories)
    addpath(supportDirectories{k});
end
for k = 1:numel(groupDirectories)
    addpath(groupDirectories{k});
end

names = { ...
    'test_scalar_integrator', ...
    'test_first_order', ...
    'test_state_dimension', ...
    'test_segment_boundaries', ...
    'test_multirate', ...
    'test_simultaneous_events', ...
    'test_measurement_delay', ...
    'test_command_latch', ...
    'test_solver_tolerance', ...
    'test_reproducibility', ...
    'test_results_policy', ...
    'test_results_safety', ...
    'test_invalid_input', ...
    'test_adaptive_integration', ...
    'test_shc09_geometry', ...
    'test_shc09_actuators', ...
    'test_shc09_dynamics', ...
    'test_shc09_operating_states', ...
    'test_shc09_outputs', ...
    'test_model_metadata', ...
    'test_command_container', ...
    'test_shc09_integration', ...
    'test_topic_bus', ...
    'test_communication_integration', ...
    'test_communication_validation', ...
    'test_communication_performance', ...
    'test_position_p', ...
    'test_velocity_pid', ...
    'test_velocity_px4', ...
    'test_attitude_p', ...
    'test_rate_pid', ...
    'test_pinv_allocator', ...
    'test_control_chain', ...
    'test_control_multirate', ...
    'test_guidance_yaw', ...
    'test_rate_step', ...
    'test_hover_hold', ...
    'test_wtr_export', ...
    'test_project_layout'};

% ---- 完整性检查：登记列表必须与磁盘文件一致 ------------------------
found = {};
for k = 1:numel(groupDirectories)
    entries = dir(fullfile(groupDirectories{k}, 'test_*.m'));
    for j = 1:numel(entries)
        found{end + 1} = entries(j).name; %#ok<AGROW>
    end
end
listed = strcat(names, '.m');
missingFiles = setdiff(listed, found);
unlistedFiles = setdiff(found, listed);
if ~isempty(missingFiles)
    error('aero:tests:inventory', '登记的测试缺少文件：%s', strjoin(missingFiles, ', '));
end
if ~isempty(unlistedFiles)
    error('aero:tests:inventory', '存在未登记的测试文件：%s', strjoin(unlistedFiles, ', '));
end

if nargin >= 1 && ~isempty(pattern)
    keep = false(size(names));
    for k = 1:numel(names)
        keep(k) = contains(names{k}, pattern);
    end
    names = names(keep);
    if isempty(names)
        error('aero:tests:filter', '没有匹配 "%s" 的测试', pattern);
    end
end

fprintf('AeroGNC-Sim 回归测试\n');
fprintf('MATLAB %s (%s) on %s\n', version, version('-release'), computer);
fprintf('项目根目录：%s\n\n', root);

nPass = 0;
nSkip = 0;
failures = {};
tAll = tic;
for k = 1:numel(names)
    fprintf('[%2d/%2d] %s\n', k, numel(names), names{k});
    t0 = tic;
    try
        info = feval(names{k});
        if ~isstruct(info) || ~isfield(info, 'passed') || ~info.passed
            error('aero:test:result', '测试未报告通过结果');
        end
        if isfield(info, 'skipped') && info.skipped
            nSkip = nSkip + 1;
            fprintf('        SKIP  %s  (%.2fs)\n', info.summary, toc(t0));
        else
            nPass = nPass + 1;
            fprintf('        PASS  %s  (%.2fs)\n', info.summary, toc(t0));
        end
    catch err
        failures{end + 1} = sprintf('%s: %s', names{k}, err.message); %#ok<AGROW>
        fprintf('        FAIL  %s\n', err.message);
        if ~isempty(err.stack)
            fprintf('              at %s:%d\n', err.stack(1).name, err.stack(1).line);
        end
    end
end

fprintf('\n%d/%d 项测试通过，用时 %.2fs', nPass, numel(names), toc(tAll));
if nSkip > 0
    fprintf('（%d 项跳过）', nSkip);
end
fprintf('\n');
if ~isempty(failures)
    fprintf('\n失败列表\n');
    for k = 1:numel(failures)
        fprintf('  - %s\n', failures{k});
    end
    error('aero:tests:failed', '%d/%d 项测试失败', numel(failures), numel(names));
end

end
