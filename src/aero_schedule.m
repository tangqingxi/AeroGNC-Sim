function sched = aero_schedule(cfg)
%AERO_SCHEDULE Build the deterministic event schedule for a configuration.
%
%   Event times of a component are t0 + offset + k*period, computed directly
%   from k so that no drift accumulates. Component updates occur on the
%   half-open interval [t0, tf): the terminal boundary tf is a schedule
%   boundary without component updates. Times that differ by less than
%   cfg.time_tolerance are treated as one simultaneous group; grouping never
%   changes a declared component period, it only fixes the execution order of
%   coincident events. Within a group, events run by (priority, component
%   order, action) and the group executes at the earliest declared time.
%
%   Action codes: 1 = sensor sample, 2 = measurement publish, 3 = periodic step.

t0 = cfg.tspan(1);
tf = cfg.tspan(2);
tol = cfg.time_tolerance;
n = numel(cfg.components);

compTimes = cell(n, 1);
maxEvents = 0;
for i = 1:n
    c = cfg.components(i);
    usable = tf - t0 - c.offset - tol;
    if usable < 0
        compTimes{i} = zeros(0, 1);
        continue;
    end
    npts = floor(usable / c.period) + 1;
    t = t0 + c.offset + (0:npts-1)' * c.period;
    compTimes{i} = t;
    maxEvents = maxEvents + 2 * numel(t);
end

evTime = zeros(maxEvents, 1);
evPrio = zeros(maxEvents, 1);
evComp = zeros(maxEvents, 1);
evAct = zeros(maxEvents, 1);
evSid = zeros(maxEvents, 1);
e = 0;
sid = 0;
for i = 1:n
    c = cfg.components(i);
    t = compTimes{i};
    for k = 1:numel(t)
        if strcmp(c.kind, 'sensor')
            sid = sid + 1;
            e = e + 1;
            evTime(e) = t(k);
            evPrio(e) = c.priority;
            evComp(e) = i;
            evAct(e) = 1;
            evSid(e) = sid;
            tp = t(k) + c.delay;
            if tp <= tf
                e = e + 1;
                evTime(e) = tp;
                evPrio(e) = c.priority + 1;
                evComp(e) = i;
                evAct(e) = 2;
                evSid(e) = sid;
            end
        else
            e = e + 1;
            evTime(e) = t(k);
            evPrio(e) = c.priority;
            evComp(e) = i;
            evAct(e) = 3;
            evSid(e) = 0;
        end
    end
end

evTime = evTime(1:e);
evPrio = evPrio(1:e);
evComp = evComp(1:e);
evAct = evAct(1:e);
evSid = evSid(1:e);

% ---- group simultaneous events -------------------------------------------
grp = zeros(e, 1);
if e > 0
    [evTime, order] = sort(evTime);
    evPrio = evPrio(order);
    evComp = evComp(order);
    evAct = evAct(order);
    evSid = evSid(order);

    g = 1;
    anchor = evTime(1);
    for k = 1:e
        if evTime(k) - anchor > tol
            g = g + 1;
            anchor = evTime(k);
        end
        grp(k) = g;
    end

    % deterministic order inside each group
    [~, order] = sortrows([grp, evPrio, evComp, evAct]);
    evTime = evTime(order);
    evPrio = evPrio(order);
    evComp = evComp(order);
    evAct = evAct(order);
    evSid = evSid(order);
    grp = grp(order);
    G = g;
    groupTime = accumarray(grp, evTime, [G, 1], @min);
else
    G = 0;
    groupTime = zeros(0, 1);
end

% ---- clamp and add boundaries --------------------------------------------
if G > 0
    if abs(groupTime(1) - t0) <= tol
        groupTime(1) = t0;
    end
    if abs(groupTime(end) - tf) <= tol
        groupTime(end) = tf;
    end
end

counts = accumarray(grp, 1, [max(G, 0), 1]);
if G > 0
    groupStart = cumsum([1; counts(1:end-1)]);
    groupEnd = cumsum(counts);
else
    groupStart = zeros(0, 1);
    groupEnd = zeros(0, 1);
end

if G == 0 || groupTime(1) > t0 + tol
    groupTime = [t0; groupTime];
    groupStart = [e + 1; groupStart];
    groupEnd = [e; groupEnd];
end
if groupTime(end) < tf - tol
    groupTime = [groupTime; tf];
    groupStart = [groupStart; e + 1];
    groupEnd = [groupEnd; e];
end

sched = struct();
sched.times = groupTime(:);
sched.event_time = evTime;
sched.event_priority = evPrio;
sched.event_component = evComp;
sched.event_action = evAct;
sched.event_sample_id = evSid;
sched.group = grp;
sched.group_start = groupStart;
sched.group_end = groupEnd;
sched.tolerance = tol;

end
