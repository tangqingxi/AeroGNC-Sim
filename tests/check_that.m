function check_that(condition, format, varargin)
%CHECK_THAT Fail the current test when condition is false.

if ~condition
    error('aero:test:assert', format, varargin{:});
end

end
