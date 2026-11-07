function check = aero_results_check(root, policy, incoming_bytes)
%AERO_RESULTS_CHECK Evaluate the results policy for a prospective write.
%
%   status:
%     ok            within every threshold
%     warning       projected usage >= warn_bytes
%     file-warning  file count >= warn_files (review prompt only)
%     file-limit    file count >= max_files (temporary budget; still warns)
%     limit         projected usage > limit_bytes; persistence must be refused

if nargin < 2 || isempty(policy)
    policy = aero_results_policy();
end
if nargin < 3 || isempty(incoming_bytes)
    incoming_bytes = 0;
end
if ~isnumeric(incoming_bytes) || ~isscalar(incoming_bytes) ...
        || ~isfinite(incoming_bytes) || incoming_bytes < 0
    error('aero:results:policy', 'incoming_bytes must be a non-negative scalar');
end

usage = aero_results_usage(root);
projected = usage.bytes + incoming_bytes;

if projected > policy.limit_bytes
    status = 'limit';
    message = sprintf(['active results would reach %.3f GB, above the %.3f GB limit ' ...
        '(root: %s)'], projected / 1e9, policy.limit_bytes / 1e9, usage.root);
elseif usage.files >= policy.max_files
    status = 'file-limit';
    message = sprintf(['%d files in results exceed the temporary budget of %d; ' ...
        'review the storage plan before adding more'], usage.files, policy.max_files);
elseif projected >= policy.warn_bytes
    status = 'warning';
    message = sprintf('active results would reach %.3f GB (warning at %.3f GB)', ...
        projected / 1e9, policy.warn_bytes / 1e9);
elseif usage.files >= policy.warn_files
    status = 'file-warning';
    message = sprintf('%d files in results reached 80%% of the temporary budget of %d', ...
        usage.files, policy.max_files);
else
    status = 'ok';
    message = '';
end

check = struct();
check.status = status;
check.message = message;
check.usage = usage;
check.projected_bytes = projected;
check.policy = policy;

end
