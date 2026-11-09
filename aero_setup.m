function aero_setup(mode)
%AERO_SETUP 初始化 AeroGNC-Sim 生产代码的 MATLAB 搜索路径
%
% 用法：
%   aero_setup();            % 添加已实现模块的目录
%   aero_setup('verbose');   % 同时打印实际加入的目录
%
% 原则：
%   - 只显式登记 src/ 下已实现模块的目录，不对整个仓库递归 addpath，
%     避免隐藏模块边界或把未完成代码混入搜索路径；
%   - **不登记 tests/**：测试替身与 fixtures 只能在测试运行时加入路径，
%     生产代码不得依赖测试数据；
%   - Aircraft/Components 的子目录按“入口 + 物理职责 + 内部辅助”登记；
%     将来新增机型或模块时在此显式添加其目录。
%
% 该函数可重复调用（addpath 不会产生重复条目）。

verbose = nargin >= 1 && strcmpi(char(mode), 'verbose');

root = fileparts(mfilename('fullpath'));

directories = { ...
    fullfile(root, 'src', 'Simulation'), ...
    fullfile(root, 'src', 'Simulation', 'Events'), ...
    fullfile(root, 'src', 'Simulation', 'Configuration'), ...
    fullfile(root, 'src', 'Simulation', 'Results'), ...
    fullfile(root, 'src', 'Simulation', 'Internal'), ...
    fullfile(root, 'src', 'Aircraft', 'SHC09'), ...
    fullfile(root, 'src', 'Aircraft', 'SHC09', 'Aerodynamics'), ...
    fullfile(root, 'src', 'Aircraft', 'SHC09', 'Internal'), ...
    fullfile(root, 'src', 'Components', 'Actuators', 'Servo'), ...
    fullfile(root, 'src', 'Components', 'Actuators', 'Motor')};

for k = 1:numel(directories)
    if isfolder(directories{k})
        addpath(directories{k});
        if verbose
            fprintf('aero_setup: added %s\n', directories{k});
        end
    end
end

end
