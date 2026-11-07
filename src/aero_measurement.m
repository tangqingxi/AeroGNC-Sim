function m = aero_measurement(bus, name)
%AERO_MEASUREMENT Latest published measurement with the given name.
%
%   m = aero_measurement(bus, name) returns the most recent entry of
%   bus.measurements whose name matches, or [] when none has been published.
%   Only published (already available) measurements are visible to components.

m = [];
if ~isstruct(bus) || ~isfield(bus, 'measurements') || isempty(bus.measurements)
    return;
end
entries = bus.measurements;
for k = numel(entries):-1:1
    if strcmp(entries{k}.name, char(name))
        m = entries{k};
        return;
    end
end

end
