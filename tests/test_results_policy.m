function info = test_results_policy()
%TEST_RESULTS_POLICY 10 GB cap, 8 GB warning, file budget and safe refusal.

p = aero_results_policy();
check_that(p.limit_bytes == 10e9, 'default limit is %.17g, expected 10e9', p.limit_bytes);
check_that(p.warn_bytes == 8e9, 'default warning is %.17g, expected 8e9', p.warn_bytes);
check_that(p.max_files == 5000 && p.warn_files == 4000, ...
    'default file budget is %d/%d, expected 4000/5000', p.warn_files, p.max_files);

scaled = aero_results_policy(struct('limit_bytes', 1000, 'warn_bytes', 800, ...
    'max_files', 10, 'warn_files', 8));

root = tempname();
mkdir(root);
cleanupRoot = onCleanup(@() remove_dir(root)); %#ok<NASGU>

write_bytes(fullfile(root, 'dummy.bin'), 900);

check = aero_results_check(root, scaled, 0);
check_that(strcmp(check.status, 'warning'), 'expected warning, got "%s"', check.status);
check_that(abs(check.projected_bytes - 900) < 1, 'projected bytes %.1f != 900', check.projected_bytes);

check = aero_results_check(root, scaled, 200);
check_that(strcmp(check.status, 'limit'), 'expected limit, got "%s"', check.status);

% refusal at the limit must not leave partial evidence behind
dummy = struct('payload', 1);
check_error(@() aero_save_result(dummy, struct('path', 'x.mat'), root, scaled), ...
    'aero:results:limit', 'refuse at limit');
check_that(~isfile(fullfile(root, 'x.mat')), 'refused save left a result file');
leftovers = dir(fullfile(root, '*.tmp'));
check_that(isempty(leftovers), 'refused save left %d temporary file(s)', numel(leftovers));

% writing outside the results root is rejected
outside = fullfile(tempdir(), 'aero_outside_result.mat');
check_error(@() aero_save_result(dummy, struct('path', outside), root, scaled), ...
    'aero:results:outside', 'outside results root');

% a normal save under the limit succeeds inside the root
root2 = tempname();
mkdir(root2);
cleanup2 = onCleanup(@() remove_dir(root2)); %#ok<NASGU>
okPolicy = aero_results_policy(struct('limit_bytes', 1e7, 'warn_bytes', 8e6, ...
    'max_files', 100, 'warn_files', 80));
[savedPath, check] = aero_save_result(dummy, struct('path', 'ok.mat'), root2, okPolicy);
check_that(isfile(savedPath), 'save did not produce %s', savedPath);
check_that(strncmp(savedPath, root2, numel(root2)), 'saved path is outside the results root');
check_that(strcmp(check.status, 'ok'), 'save status "%s" != ok', check.status);

% a memory-only run writes nothing
before = aero_results_usage(root2);
cfg = aero_default_config();
cfg.tspan = [0, 0.05];
cfg.x0 = 0;
cfg.initial_command = 0;
cfg.results_root = root2;
cfg.save.enabled = false;
aero_sim(scalar_integrator_model(), cfg);
after = aero_results_usage(root2);
check_that(after.files == before.files, ...
    'memory-only run changed the results file count (%d -> %d)', before.files, after.files);

% file-count thresholds warn at 80% and report the temporary budget limit
root3 = tempname();
mkdir(root3);
cleanup3 = onCleanup(@() remove_dir(root3)); %#ok<NASGU>
for k = 1:9
    write_bytes(fullfile(root3, sprintf('f%02d.bin', k)), 1);
end
check = aero_results_check(root3, scaled, 0);
check_that(strcmp(check.status, 'file-warning'), 'expected file-warning, got "%s"', check.status);
write_bytes(fullfile(root3, 'f10.bin'), 1);
check = aero_results_check(root3, scaled, 0);
check_that(strcmp(check.status, 'file-limit'), 'expected file-limit, got "%s"', check.status);

info = struct('passed', true, 'summary', sprintf( ...
    'defaults 10e9/8e9/5000/4000; warning, limit refusal, outside-root rejection and file budget verified'));

end

function write_bytes(path, n)
fid = fopen(path, 'w');
check_that(fid >= 0, 'cannot create %s', path);
fwrite(fid, zeros(1, n, 'uint8'), 'uint8');
fclose(fid);
end

function remove_dir(path)
if isfolder(path)
    try
        rmdir(path, 's');
    catch
    end
end
end
