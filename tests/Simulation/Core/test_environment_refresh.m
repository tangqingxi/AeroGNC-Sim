function info = test_environment_refresh()
%TEST_ENVIRONMENT_REFRESH 隔离 Git 仓库中验证同一会话的运行入口来源；无输入，输出测试结果。
originalPath = path;
root = tempname();
mkdir(root);
cleanup = onCleanup(@() restore(originalPath, root)); %#ok<NASGU>
source = fileparts(which('aero_env_info'));
copyfile(fullfile(source, 'aero_env_info.m'), root);
copyfile(fullfile(source, 'aero_project_root.m'), root);
write_text(fullfile(root, 'ARCHITECTURE.md'), '测试仓库');
git(root, 'init');
git(root, 'add .');
git(root, '-c user.name=Test -c user.email=test@example.invalid commit -m baseline');
addpath(root, '-begin');
clear aero_env_info aero_project_root
model = scalar_integrator_model();
cfg = aero_default_config();
cfg.tspan = [0 0.01];
cfg.x0 = 0;
cfg.initial_command = 0;
first = aero_sim(model, cfg);
check_that(~first.environment.git_dirty, '首次运行未识别 clean');
head = git(root, 'rev-parse HEAD');
check_that(strcmp(first.environment.git_commit, head), '首次 HEAD 不符');
marker = fullfile(root, 'source_change.m');
write_text(marker, '% 测试源码变更');
second = aero_sim(model, cfg);
third = aero_sim(model, cfg);
check_that(second.environment.git_dirty && third.environment.git_dirty, ...
    '同一会话重复运行沿用了 clean 缓存');
git(root, 'add .');
git(root, '-c user.name=Test -c user.email=test@example.invalid commit -m change');
fourth = aero_sim(model, cfg);
check_that(~fourth.environment.git_dirty, '提交后未刷新 clean');
check_that(~strcmp(first.environment.git_commit, fourth.environment.git_commit), ...
    '同一会话 HEAD 未刷新');
% 在积分回调中创建源码，结果仍必须保存仿真开始时的 clean 状态。
model.dynamics = @(t,x,u,env) change_source(t,x,u,env,marker);
fifth = aero_sim(model, cfg);
check_that(~fifth.environment.git_dirty, '记录了结束状态而非开始状态');
sixth = aero_sim(model, cfg);
check_that(sixth.environment.git_dirty, '下一次开始未识别回调产生的 dirty');
check_that(~first.environment.git_dirty, '已保存的软件来源被后续运行修改');
fprintf('来源刷新：clean=%d dirty=%d repeat=%d committed=%d entry=%d next=%d\n', ...
    first.environment.git_dirty, second.environment.git_dirty, third.environment.git_dirty, ...
    fourth.environment.git_dirty, fifth.environment.git_dirty, sixth.environment.git_dirty);
info = struct('passed', true, 'summary', '同一会话 clean/dirty/重复/HEAD/运行入口快照通过');
end

function dx = change_source(~, x, ~, ~, marker)
% 模拟运行过程中修改源码，检验入口快照时序。
write_text(marker, '% 运行过程中修改源码');
dx = zeros(size(x));
end

function out = git(root, args)
% 命令只作用于本测试创建的临时仓库。
[status, out] = system(sprintf('git -C "%s" %s', root, args));
check_that(status == 0, '隔离 Git 命令失败：%s', out);
out = strtrim(out);
end

function write_text(file, content)
fid = fopen(file, 'w');
check_that(fid >= 0, '无法创建测试源码');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', content);
end

function restore(originalPath, root)
path(originalPath);
clear aero_env_info aero_project_root
if isfolder(root)
    rmdir(root, 's');
end
end
