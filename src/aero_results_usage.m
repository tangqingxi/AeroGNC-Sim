function usage = aero_results_usage(root)
%AERO_RESULTS_USAGE Measure bytes and file count below a results root.

usage = struct('root', '', 'exists', false, 'bytes', 0, 'files', 0);
if nargin < 1 || isempty(root)
    return;
end
usage.root = char(root);
if ~isfolder(usage.root)
    return;
end

entries = dir(fullfile(usage.root, '**', '*'));
entries = entries(~[entries.isdir]);
usage.exists = true;
usage.files = numel(entries);
usage.bytes = sum([entries.bytes]);

end
