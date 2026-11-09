function check_error(fn, identifier_prefix, context)
%CHECK_ERROR Assert that calling fn raises an error with the given identifier prefix.

if nargin < 3
    context = '';
end
try
    fn();
catch err
    check_that(strncmp(err.identifier, identifier_prefix, numel(identifier_prefix)), ...
        '%s: expected error "%s" but got "%s" (%s)', ...
        context, identifier_prefix, err.identifier, err.message);
    return;
end
error('aero:test:assert', '%s: expected error "%s" but no error was raised', ...
    context, identifier_prefix);

end
