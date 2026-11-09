function sched = aero_schedule(cfg)
%AERO_SCHEDULE 生成确定性事件调度表
%
% 输入：
%   cfg - 已通过 aero_validate_config 的配置结构
%
% 输出 sched：
%   times            各事件组的执行时刻（含 t0 与 tf 边界）
%   event_time       事件声明时刻，用于诊断与频率漂移验证
%   event_priority   同一时刻的执行优先级，数值越小越先执行
%   event_component  事件所属组件下标
%   event_action     动作名称：'sample' | 'publish' | 'step'
%   event_sample_id  sensor 采样编号，publish 事件引用同一次采样
%   group            事件所属组
%   group_start/end  每组在事件数组中的起止下标
%   tolerance        判定“同时事件”的时间容差
%
% 时刻规则：
%   组件事件时刻 = t0 + offset + k*period，直接由 k 计算，不经过固定网格，
%   因此不会累积舍入漂移；组件更新区间为半开区间 [t0, tf)，tf 只作为积分
%   终点边界，不触发组件更新。
%
% 同时事件：
%   相差小于 tolerance 的事件视为同一物理时刻，执行组内按
%   (priority, 组件声明顺序, 动作) 排序；合并只改变执行顺序，不改变组件频率。

SAMPLE = 1;      % 传感器采样：读取真值
PUBLISH = 2;     % 测量发布：样本到达可用时刻
STEP = 3;        % 周期组件更新
ACTION_NAMES = {'sample', 'publish', 'step'};

t0 = cfg.tspan(1);
tf = cfg.tspan(2);
tol = cfg.time_tolerance;
nComponents = numel(cfg.components);

% ---- 1. 逐个组件生成事件时刻 ------------------------------------------
componentTimes = cell(nComponents, 1);
maxEvents = 0;
for i = 1:nComponents
    c = cfg.components(i);
    % 半开区间：最后一个事件必须严格早于 tf（预留一个容差量）
    usableSpan = tf - t0 - c.offset - tol;
    if usableSpan < 0
        componentTimes{i} = zeros(0, 1);
        continue;
    end
    nTicks = floor(usableSpan / c.period) + 1;
    componentTimes{i} = t0 + c.offset + (0:nTicks - 1)' * c.period;
    maxEvents = maxEvents + 2 * nTicks;   % 上限：每个采样最多再产生一次发布
end

% ---- 2. 展开为事件记录 -------------------------------------------------
eventTime = zeros(maxEvents, 1);
eventPriority = zeros(maxEvents, 1);
eventComponent = zeros(maxEvents, 1);
eventCode = zeros(maxEvents, 1);
eventSampleId = zeros(maxEvents, 1);

nEvents = 0;
sampleCounter = 0;
for i = 1:nComponents
    c = cfg.components(i);
    times = componentTimes{i};
    for k = 1:numel(times)
        t = times(k);
        if strcmp(c.kind, 'sensor')
            sampleCounter = sampleCounter + 1;
            nEvents = nEvents + 1;
            eventTime(nEvents) = t;
            eventPriority(nEvents) = c.priority;
            eventComponent(nEvents) = i;
            eventCode(nEvents) = SAMPLE;
            eventSampleId(nEvents) = sampleCounter;

            % 延迟测量：采样后 delay 秒才发布，发布事件本身不改变采样时间
            publishTime = t + c.delay;
            if publishTime <= tf
                nEvents = nEvents + 1;
                eventTime(nEvents) = publishTime;
                eventPriority(nEvents) = c.priority + 1;
                eventComponent(nEvents) = i;
                eventCode(nEvents) = PUBLISH;
                eventSampleId(nEvents) = sampleCounter;
            end
        else
            nEvents = nEvents + 1;
            eventTime(nEvents) = t;
            eventPriority(nEvents) = c.priority;
            eventComponent(nEvents) = i;
            eventCode(nEvents) = STEP;
            eventSampleId(nEvents) = 0;
        end
    end
end

eventTime = eventTime(1:nEvents);
eventPriority = eventPriority(1:nEvents);
eventComponent = eventComponent(1:nEvents);
eventCode = eventCode(1:nEvents);
eventSampleId = eventSampleId(1:nEvents);

% ---- 3. 合并同时事件并排序 --------------------------------------------
group = zeros(nEvents, 1);
if nEvents > 0
    [eventTime, order] = sort(eventTime);
    eventPriority = eventPriority(order);
    eventComponent = eventComponent(order);
    eventCode = eventCode(order);
    eventSampleId = eventSampleId(order);

    groupIndex = 1;
    anchor = eventTime(1);
    for k = 1:nEvents
        if eventTime(k) - anchor > tol
            groupIndex = groupIndex + 1;
            anchor = eventTime(k);
        end
        group(k) = groupIndex;
    end

    % 组内固定顺序：优先级 -> 组件声明顺序 -> 动作（采样先于发布）
    [~, order] = sortrows([group, eventPriority, eventComponent, eventCode]);
    eventTime = eventTime(order);
    eventPriority = eventPriority(order);
    eventComponent = eventComponent(order);
    eventCode = eventCode(order);
    eventSampleId = eventSampleId(order);
    group = group(order);

    nGroups = groupIndex;
    groupTime = accumarray(group, eventTime, [nGroups, 1], @min);
else
    nGroups = 0;
    groupTime = zeros(0, 1);
    group = zeros(0, 1);
end

% ---- 4. 边界修正：首尾对齐 t0/tf --------------------------------------
if nGroups > 0
    if abs(groupTime(1) - t0) <= tol
        groupTime(1) = t0;
    end
    if abs(groupTime(end) - tf) <= tol
        groupTime(end) = tf;
    end
end

counts = accumarray(group, 1, [max(nGroups, 0), 1]);
if nGroups > 0
    groupStart = cumsum([1; counts(1:end - 1)]);
    groupEnd = cumsum(counts);
else
    groupStart = zeros(0, 1);
    groupEnd = zeros(0, 1);
end

% 起点边界 t0：即使没有组件事件也记录初始状态
if nGroups == 0 || groupTime(1) > t0 + tol
    groupTime = [t0; groupTime];
    groupStart = [nEvents + 1; groupStart];
    groupEnd = [nEvents; groupEnd];
end
% 终点边界 tf：只积分不触发组件
if groupTime(end) < tf - tol
    groupTime = [groupTime; tf];
    groupStart = [groupStart; nEvents + 1];
    groupEnd = [groupEnd; nEvents];
end

sched = struct();
sched.times = groupTime(:);
sched.event_time = eventTime;
sched.event_priority = eventPriority;
sched.event_component = eventComponent;
sched.event_action = reshape(ACTION_NAMES(eventCode), [], 1);
sched.event_sample_id = eventSampleId;
sched.group = group;
sched.group_start = groupStart;
sched.group_end = groupEnd;
sched.tolerance = tol;

end
