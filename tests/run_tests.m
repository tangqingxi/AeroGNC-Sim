function run_tests(pattern)
%RUN_TESTS Run the Simulation-Kernel regression suite.
%
%   run_tests            run every test
%   run_tests('multirate') run tests whose name contains the pattern
%
%   The script exits with an error when any test fails, so that
%   `matlab -batch "run_tests"` returns a non-zero exit code.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(fullfile(root, 'src'));
addpath(fullfile(root, 'models'));
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
        nPass = nPass + 1;
        fprintf('        PASS  %s  (%.2fs)\n', info.summary, toc(t0));
    catch err
        failures{end + 1} = sprintf('%s: %s', names{k}, err.message); %#ok<AGROW>
        fprintf('        FAIL  %s\n', err.message);
        if ~isempty(err.stack)
            fprintf('              at %s:%d\n', err.stack(1).name, err.stack(1).line);
        end
    end
end

fprintf('\n%d/%d tests passed in %.2fs\n', nPass, numel(names), toc(tAll));
if ~isempty(failures)
    fprintf('\nFAILURES\n');
    for k = 1:numel(failures)
        fprintf('  - %s\n', failures{k});
    end
    error('aero:tests:failed', '%d of %d tests failed', numel(failures), numel(names));
end

end
