classdef aero_topic_bus < handle
%AERO_TOPIC_BUS 轻量、确定的 Latest-Value Topic 总线
%
% 定位：
%   为 Guidance / Navigation / Control 等模块提供统一的按名称通信底座。
%   总线只保存每个 Topic 的最新已发布消息，不实现时钟、调度或历史队列；
%   时间由调用方（Simulation）提供，消息可用性按 available_time_s 判断。
%
% 主要接口：
%   bus = aero_topic_bus();
%   bus.register(name, spec);                  % 注册 Topic 与数据契约
%   bus.publish(name, data, sample, avail, now, src) % now = 当前仿真时间
%   msg = bus.read(name, now);                 % 读取（检查可用/过期）
%   [msg, status] = bus.tryRead(name, now);    % 非抛出式读取
%   msg = bus.latest(name, 'diagnostic');      % 仅诊断，不做时间门限
%   tf = bus.isRegistered(name); tf = bus.hasMessage(name);
%   names = bus.topicNames(); spec = bus.getSpec(name); seq = bus.sequence();
%
% 确定性与约束：
%   - 同一 Topic 的 sample_time_s 不允许倒退；相等允许，后者覆盖前者并递增序号；
%   - sequence 为全局发布顺序，topic_sequence 为该 Topic 的独立版本号；
%   - 数据必须匹配注册契约（字段、尺寸、有限数值、单位仅声明不换算）；
%   - publish 必须携带当前时间 now，且 now >= available_time_s；提前发布未来
%     消息会覆盖仍有效的旧样本，因此被直接拒绝；
%   - "truth/" 前缀只是命名约定，不等于访问控制：真实状态仍只允许 Sensor
%     回调与明确授权的测试适配器读取，生产 GNC 不得读取真值；
%   - 总线不创建 Timer、线程、网络或文件 I/O。
%
% 未来接入 Simulation 时，应由 Simulation 持有唯一实例并作为权威通信状态，
% 不允许再建立第二套并行总线。

properties (Access = private)
    specs               % containers.Map: name -> spec
    messages            % containers.Map: name -> latest message
    registeredNames     % cellstr，按注册顺序
    globalSequence      % uint64，全局发布顺序
    topicVersions       % containers.Map: name -> uint64
    logEnabled          % 是否记录发布日志（默认 false）
    logTopics           % 需要记录的 Topic 名称 cellstr
    logMaxRecords       % 日志总容量（正标量）
    logRecords          % 1×N cell，每条为一次实际发布
    logDropped          % uint64，因容量上限未记录的条数
end

methods
    function obj = aero_topic_bus()
        obj.specs = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.messages = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.topicVersions = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.registeredNames = cell(1, 0);
        obj.globalSequence = uint64(0);
        obj.logEnabled = false;
        obj.logTopics = cell(1, 0);
        obj.logMaxRecords = 10000;
        obj.logRecords = cell(1, 0);
        obj.logDropped = uint64(0);
    end

    function enableLog(obj, topics, maxRecords)
        % 开启轻量发布日志（默认关闭）。只记录显式选择的 Topic，且拒绝真值 Topic。
        if nargin < 3 || isempty(maxRecords)
            maxRecords = 10000;
        end
        names = normalize_topic_list(topics);
        if isempty(names)
            error('aero:bus:logConfig', '日志至少需要一个 Topic');
        end
        for k = 1:numel(names)
            spec = obj.requireRegistered(names{k});
            if spec.allow_ground_truth
                error('aero:bus:logGroundTruth', ...
                    'Topic "%s" 承载真值，禁止进入发布日志', names{k});
            end
        end
        if ~isnumeric(maxRecords) || ~isscalar(maxRecords) || ~isfinite(maxRecords) ...
                || maxRecords < 1 || maxRecords ~= fix(maxRecords)
            error('aero:bus:logConfig', 'maxRecords 必须是正整数');
        end
        obj.logEnabled = true;
        obj.logTopics = names(:).';
        obj.logMaxRecords = double(maxRecords);
        obj.logRecords = cell(1, 0);
        obj.logDropped = uint64(0);
    end

    function snap = logSnapshot(obj)
        % 返回不可变的日志快照（普通值结构，可安全写入结果或 .mat）
        snap = struct();
        snap.enabled = obj.logEnabled;
        snap.topics = obj.logTopics;
        snap.max_records = obj.logMaxRecords;
        snap.records = obj.logRecords;
        snap.dropped = obj.logDropped;
    end

    function register(obj, name, spec)
        name = obj.normalizeName(name);
        if obj.specs.isKey(name)
            error('aero:bus:duplicateTopic', 'Topic "%s" 已注册', name);
        end
        spec = canonicalizeSpec(spec);
        if spec.allow_ground_truth && ~startsWith(name, 'truth/')
            error('aero:bus:groundTruthTopic', ...
                'Topic "%s" 声明承载真值，但名称必须以 "truth/" 开头', name);
        end
        if ~spec.allow_ground_truth && startsWith(name, 'truth/')
            error('aero:bus:groundTruthTopic', ...
                '真值 Topic "%s" 必须在契约中声明 allow_ground_truth=true', name);
        end
        obj.specs(name) = spec;
        obj.registeredNames{end + 1} = name;
    end

    function publish(obj, name, data, sampleTime, availableTime, now, source)
        name = obj.normalizeName(name);
        spec = obj.requireRegistered(name);

        if nargin < 5 || isempty(availableTime)
            availableTime = sampleTime;
        end
        if nargin < 6 || isempty(now)
            error('aero:bus:invalidTime', ...
                'publish 需要显式 now（当前仿真时间），以阻止提前发布未来消息');
        end
        if nargin < 7 || isempty(source)
            source = '';
        end

        obj.requireTime(sampleTime, 'sample_time_s');
        obj.requireTime(availableTime, 'available_time_s');
        obj.requireTime(now, 'now');
        if availableTime < sampleTime
            error('aero:bus:invalidTime', ...
                'Topic "%s" 的 available_time_s 不能早于 sample_time_s', name);
        end
        % Latest-Value 安全：消息必须在可用时刻之后才能发布，否则会覆盖
        % 仍在有效期内的旧样本，造成消费者读到“未来”数据。
        if now < availableTime
            error('aero:bus:notYetAvailable', ...
                'Topic "%s" 在 t=%.9f 提前发布（available=%.9f），禁止覆盖仍有效的旧样本', ...
                name, now, availableTime);
        end

        if obj.messages.isKey(name)
            previous = obj.messages(name);
            if sampleTime < previous.sample_time_s
                error('aero:bus:timeRegression', ...
                    'Topic "%s" 时间倒退：%.9f < 上一次 %.9f', ...
                    name, sampleTime, previous.sample_time_s);
            end
            version = obj.topicVersions(name) + uint64(1);
        else
            version = uint64(1);
        end

        obj.validatePayload(name, spec, data);
        if ~spec.allow_ground_truth && isfield(data, 'is_ground_truth') ...
                && logical(data.is_ground_truth)
            error('aero:bus:groundTruthForbidden', ...
                'Topic "%s" 不允许通过普通 GNC Topic 发布真值', name);
        end

        obj.globalSequence = obj.globalSequence + uint64(1);

        message = struct();
        message.name = name;
        message.data = data;
        message.sample_time_s = sampleTime;
        message.available_time_s = availableTime;
        message.sequence = obj.globalSequence;
        message.topic_sequence = version;
        message.source = char(source);

        obj.messages(name) = message;
        obj.topicVersions(name) = version;
        obj.recordLog(message);
    end

    function msg = read(obj, name, now)
        [msg, status] = obj.tryRead(name, now);
        switch status
            case 'ok'
                return;
            case 'unknown_topic'
                error('aero:bus:unknownTopic', 'Topic "%s" 未注册', char(name));
            case 'missing'
                error('aero:bus:missingMessage', 'Topic "%s" 尚未发布任何消息', char(name));
            case 'not_available'
                error('aero:bus:notAvailable', ...
                    'Topic "%s" 的消息在 t=%.9f 尚不可用', char(name), now);
            case 'stale'
                error('aero:bus:staleMessage', ...
                    'Topic "%s" 的消息在 t=%.9f 已超过有效期', char(name), now);
            otherwise
                error('aero:bus:read', '未知读取状态：%s', status);
        end
    end

    function [msg, status] = tryRead(obj, name, now)
        name = char(string(name));
        msg = [];
        if ~obj.specs.isKey(name)
            status = 'unknown_topic';
            return;
        end
        if ~obj.messages.isKey(name)
            status = 'missing';
            return;
        end
        obj.requireTime(now, 'now');
        current = obj.messages(name);
        spec = obj.specs(name);
        if now < current.available_time_s
            status = 'not_available';
            return;
        end
        if isfinite(spec.max_age_s) && (now - current.sample_time_s) > spec.max_age_s
            status = 'stale';
            return;
        end
        msg = current;
        status = 'ok';
    end

    function msg = latest(obj, name, mode)
        % 仅诊断用途：不检查可用时间，正式 GNC 消费者必须使用 read/tryRead。
        if nargin < 3 || ~(ischar(mode) || (isstring(mode) && isscalar(mode))) ...
                || ~strcmpi(char(mode), 'diagnostic')
            error('aero:bus:diagnosticRequired', ...
                'latest 仅用于诊断；正式读取请使用 read/tryRead（需传入 ''diagnostic'' 才允许）');
        end
        name = obj.normalizeName(name);
        obj.requireRegistered(name);
        if ~obj.messages.isKey(name)
            error('aero:bus:missingMessage', 'Topic "%s" 尚未发布任何消息', name);
        end
        msg = obj.messages(name);
    end

    function tf = isRegistered(obj, name)
        tf = obj.specs.isKey(char(string(name)));
    end

    function tf = hasMessage(obj, name)
        tf = obj.messages.isKey(char(string(name)));
    end

    function names = topicNames(obj)
        names = obj.registeredNames(:);
    end

    function spec = getSpec(obj, name)
        name = obj.normalizeName(name);
        spec = obj.requireRegistered(name);
    end

    function value = sequence(obj)
        value = obj.globalSequence;
    end
end

methods (Access = private)
    function recordLog(obj, message)
        % 只记录显式选择的 Topic 的“实际发布”，不做定时采样。
        if ~obj.logEnabled || ~any(strcmp(obj.logTopics, message.name))
            return;
        end
        if numel(obj.logRecords) >= obj.logMaxRecords
            obj.logDropped = obj.logDropped + uint64(1);
            return;
        end
        obj.logRecords{end + 1} = struct( ...
            'name', message.name, ...
            'data', message.data, ...
            'sample_time_s', message.sample_time_s, ...
            'available_time_s', message.available_time_s, ...
            'sequence', message.sequence, ...
            'topic_sequence', message.topic_sequence);
    end

    function name = normalizeName(~, name)
        if isstring(name) && isscalar(name)
            name = char(name);
        end
        if ~ischar(name) || isempty(name) || size(name, 1) ~= 1
            error('aero:bus:invalidName', 'Topic 名称必须是单行字符/字符串');
        end
        pattern = '^[a-z][a-z0-9_]{0,63}(/[a-z][a-z0-9_]{0,63})?$';
        if isempty(regexp(name, pattern, 'once'))
            error('aero:bus:invalidName', ...
                'Topic 名称 "%s" 非法：只允许小写字母、数字、下划线，最多一级 "/"', name);
        end
    end

    function spec = requireRegistered(obj, name)
        if ~obj.specs.isKey(name)
            error('aero:bus:unknownTopic', 'Topic "%s" 未注册', name);
        end
        spec = obj.specs(name);
    end

    function requireTime(~, value, label)
        if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ~isfinite(value)
            error('aero:bus:invalidTime', '%s 必须是有限实数标量', label);
        end
    end

    function validatePayload(obj, name, spec, data)
        if ~isstruct(data) || ~isscalar(data)
            error('aero:bus:schemaType', 'Topic "%s" 的载荷必须是标量结构体', name);
        end
        actual = fieldnames(data);
        missing = setdiff(spec.fields, actual);
        extra = setdiff(actual, spec.fields);
        if ~isempty(missing)
            error('aero:bus:schemaMissingField', ...
                'Topic "%s" 缺少字段：%s', name, strjoin(missing, ', '));
        end
        if ~isempty(extra)
            error('aero:bus:schemaExtraField', ...
                'Topic "%s" 含未声明字段：%s', name, strjoin(extra, ', '));
        end
        for k = 1:numel(spec.fields)
            field = spec.fields{k};
            value = data.(field);
            if ~isnumeric(value) || ~isreal(value)
                error('aero:bus:schemaType', ...
                    'Topic "%s" 字段 "%s" 必须是实数数值', name, field);
            end
            if ~isequal(size(value), spec.dimensions{k})
                error('aero:bus:schemaDimension', ...
                    'Topic "%s" 字段 "%s" 尺寸为 %s，契约为 %s', ...
                    name, field, mat2str(size(value)), mat2str(spec.dimensions{k}));
            end
            if any(~isfinite(value(:)))
                error('aero:bus:schemaValue', ...
                    'Topic "%s" 字段 "%s" 含非有限值', name, field);
            end
        end
    end
end
end

% ===========================================================================
function names = normalize_topic_list(topics)
if ischar(topics)
    names = {topics};
elseif isstring(topics)
    names = cellstr(topics(:).');
elseif iscell(topics)
    if ~all(cellfun(@(v) ischar(v) || (isstring(v) && isscalar(v)), topics))
        error('aero:bus:logConfig', '日志 Topic 必须是字符/字符串');
    end
    names = cellfun(@char, topics(:).', 'UniformOutput', false);
else
    error('aero:bus:logConfig', '日志 Topic 必须是字符/字符串或 cell 数组');
end
names = names(~cellfun(@isempty, names));
end

% ===========================================================================
function spec = canonicalizeSpec(spec)
% 手工构造的 spec 也必须经过 aero_topic_spec 的完整校验，避免绕过契约规范。
allowed = {'fields', 'dimensions', 'units', 'allow_ground_truth', 'max_age_s', 'description'};
if ~isstruct(spec) || ~isscalar(spec)
    error('aero:bus:invalidSpec', 'spec 必须是 aero_topic_spec 生成的标量结构体');
end
unknown = setdiff(fieldnames(spec), allowed);
if ~isempty(unknown)
    error('aero:bus:invalidSpec', 'spec 含未知字段：%s', strjoin(unknown, ', '));
end
required = {'fields', 'dimensions', 'units', 'allow_ground_truth', 'max_age_s'};
for k = 1:numel(required)
    if ~isfield(spec, required{k})
        error('aero:bus:invalidSpec', 'spec 缺少字段：%s', required{k});
    end
end

options = struct();
options.allow_ground_truth = spec.allow_ground_truth;
options.max_age_s = spec.max_age_s;
if isfield(spec, 'description')
    options.description = spec.description;
end
spec = aero_topic_spec(spec.fields, spec.dimensions, spec.units, options);
end
