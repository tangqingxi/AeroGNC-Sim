function root = aero_project_root()
%AERO_PROJECT_ROOT 返回 AeroGNC-Sim 仓库根目录
%
% 通过向上查找 ARCHITECTURE.md 定位根目录，避免因目录调整而硬编码层级。
% 找不到时抛出 aero:path:projectRoot，提示当前目录结构异常。

here = fileparts(mfilename('fullpath'));
root = here;
for level = 1:8
    if isfile(fullfile(root, 'ARCHITECTURE.md'))
        return;
    end
    parent = fileparts(root);
    if strcmp(parent, root)
        break;
    end
    root = parent;
end

error('aero:path:projectRoot', ...
    '无法定位项目根目录：从 "%s" 向上未找到 ARCHITECTURE.md', here);

end
