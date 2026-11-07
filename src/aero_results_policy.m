function p = aero_results_policy(overrides)
%AERO_RESULTS_POLICY Storage policy for persisted results.
%
%   Active results are capped at 10 GB (10e9 bytes, the conservative decimal
%   reading) with a warning threshold at 8 GB. The file budget of 5000 files
%   is a temporary planning value, not a frozen project standard; reaching 80%
%   only asks for a review. Byte limits block new persistence; file limits warn.
%
%   p = aero_results_policy() returns the defaults.
%   p = aero_results_policy(struct('limit_bytes', 1000, ...)) overrides fields
%   (used by tests to exercise the policy without writing gigabytes).

p = struct();
p.limit_bytes = 10e9;
p.warn_bytes = 8e9;
p.max_files = 5000;
p.warn_files = 4000;

if nargin >= 1 && ~isempty(overrides)
    if ~isstruct(overrides) || ~isscalar(overrides)
        error('aero:results:policy', 'policy overrides must be a scalar struct');
    end
    names = fieldnames(overrides);
    for k = 1:numel(names)
        if ~isfield(p, names{k})
            error('aero:results:policy', 'unknown policy field "%s"', names{k});
        end
        p.(names{k}) = overrides.(names{k});
    end
end

if ~isnumeric(p.limit_bytes) || ~isscalar(p.limit_bytes) || ~isfinite(p.limit_bytes) ...
        || p.limit_bytes <= 0
    error('aero:results:policy', 'limit_bytes must be a positive finite scalar');
end
if ~isnumeric(p.warn_bytes) || ~isscalar(p.warn_bytes) || ~isfinite(p.warn_bytes) ...
        || p.warn_bytes <= 0 || p.warn_bytes >= p.limit_bytes
    error('aero:results:policy', 'warn_bytes must be positive and below limit_bytes');
end
if ~isnumeric(p.max_files) || ~isscalar(p.max_files) || p.max_files < 1 ...
        || p.max_files ~= fix(p.max_files)
    error('aero:results:policy', 'max_files must be a positive integer');
end
if ~isnumeric(p.warn_files) || ~isscalar(p.warn_files) || p.warn_files < 1 ...
        || p.warn_files >= p.max_files || p.warn_files ~= fix(p.warn_files)
    error('aero:results:policy', 'warn_files must be a positive integer below max_files');
end

end
