function cfg = aero_default_config()
%AERO_DEFAULT_CONFIG 返回仿真默认配置
%
% 字段：
%   tspan           [t0 tf] 虚拟仿真区间（s）
%   x0              连续状态初值；为空时使用 model.x0
%   initial_command 初始锁存输入
%   components      离散组件结构数组（字段见 aero_empty_component）
%   solver          ode45 设置：RelTol、AbsTol、MaxStep、InitialStep
%   environment     原样传给 model.dynamics 的环境结构
%   seed            随机种子（内核本身不消耗随机数，仅记录以复现实验）
%   time_tolerance  同时事件合并容差；为空时取 1e-12*max(1,|tf|)
%   results_root    结果根目录；为空时使用 <项目根>/results
%   save            持久化设置：enabled、path、estimated_bytes
%   verbose         是否打印运行摘要
%
% 该函数是纯构造器，不读取任何全局状态。

cfg = struct();
cfg.tspan = [0, 1];
cfg.x0 = [];
cfg.initial_command = 0;
cfg.components = aero_empty_component();
cfg.solver = struct('RelTol', 1e-9, 'AbsTol', 1e-11, 'MaxStep', Inf, 'InitialStep', []);
cfg.environment = struct();
cfg.seed = 0;
cfg.time_tolerance = [];
cfg.results_root = '';
cfg.save = struct('enabled', false, 'path', '', 'estimated_bytes', 0);
cfg.verbose = false;

end
