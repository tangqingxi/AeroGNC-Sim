function root = aero_project_root()
%AERO_PROJECT_ROOT Repository root (the parent of src/).

root = fileparts(fileparts(mfilename('fullpath')));

end
