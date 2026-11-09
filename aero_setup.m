function aero_setup(mode)
%AERO_SETUP 初始化 AeroGNC-Sim 的 MATLAB 搜索路径
%
% 用法：
%   aero_setup();            % 添加内核与测试替身所在目录
%   aero_setup('verbose');   % 同时打印实际加入的目录
%
% 原则：
%   - 只显式添加已实现模块的目录，不对整个仓库递归 addpath，
%     避免隐藏模块边界或把未完成代码混入搜索路径；
%   - Aircraft/Components/GNC/Scenarios 尚未实现，因此当前不在列表中；
%     将来新增模块时在此显式登记其目录。
%
% 该函数可重复调用（addpath 不会产生重复条目）。

verbose = nargin >= 1 && strcmpi(char(mode), 'verbose');

root = fileparts(mfilename('fullpath'));

directories = { ...
    fullfile(root, 'src', 'Simulation'), ...
    fullfile(root, 'src', 'Simulation', 'Results'), ...
    fullfile(root, 'src', 'Aircraft', 'SHC09'), ...
    fullfile(root, 'src', 'Components', 'Actuators', 'Servo'), ...
    fullfile(root, 'src', 'Components', 'Actuators', 'Motor'), ...
    fullfile(root, 'tests', 'fixtures')};

for k = 1:numel(directories)
    if isfolder(directories{k})
        addpath(directories{k});
        if verbose
            fprintf('aero_setup: added %s\n', directories{k});
        end
    end
end

end
