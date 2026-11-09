function [times, actions] = event_times(result, name)
%EVENT_TIMES Event times and actions recorded for one component name.

names = {result.events.component};
actions = {result.events.action};
mask = strcmp(names, char(name));
times = [result.events(mask).time];
actions = actions(mask);

end
