function run_tests(pattern)
%RUN_TESTS 运行 Simulation-Kernel 回归测试集
%
% 用法：
%   run_tests              运行全部测试
%   run_tests('multirate') 只运行名称包含该片段的测试
%
% 任一测试失败时抛出 aero:tests:failed，使
% `matlab -batch "run('tests/run_tests.m')"` 返回非零退出码。
% 被跳过的测试（例如系统不允许创建 Junction）单独计数，不伪装成通过。

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(root);      % aero_setup 位于仓库根目录
aero_setup();
addpath(here);

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
    'test_adaptive_integration'};

if nargin >= 1 && ~isempty(pattern)
    keep = false(size(names));
    for k = 1:numel(names)
        keep(k) = contains(names{k}, pattern);
    end
    names = names(keep);
    if isempty(names)
        error('aero:tests:filter', 'no test matches "%s"', pattern);
    end
end

fprintf('AeroGNC-Sim Simulation-Kernel tests\n');
fprintf('MATLAB %s (%s) on %s\n', version, version('-release'), computer);
fprintf('Project root: %s\n\n', root);

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
            error('aero:test:result', 'test did not report a passing result');
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

fprintf('\n%d/%d tests passed in %.2fs', nPass, numel(names), toc(tAll));
if nSkip > 0
    fprintf(' (%d skipped)', nSkip);
end
fprintf('\n');
if ~isempty(failures)
    fprintf('\nFAILURES\n');
    for k = 1:numel(failures)
        fprintf('  - %s\n', failures{k});
    end
    error('aero:tests:failed', '%d of %d tests failed', numel(failures), numel(names));
end

end
