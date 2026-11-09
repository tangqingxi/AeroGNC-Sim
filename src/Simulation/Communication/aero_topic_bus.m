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
%   bus.publish(name, data, sample, avail, src) % 发布最新值
%   msg = bus.read(name, now);                 % 读取（检查可用/过期）
%   [msg, status] = bus.tryRead(name, now);    % 非抛出式读取
%   msg = bus.latest(name);                    % 最近一次发布（不做时间门限）
%   tf = bus.isRegistered(name); tf = bus.hasMessage(name);
%   names = bus.topicNames(); spec = bus.getSpec(name); seq = bus.sequence();
%
% 确定性与约束：
%   - 同一 Topic 的 sample_time_s 不允许倒退；相等允许，后者覆盖前者并递增序号；
%   - sequence 为全局发布顺序，topic_sequence 为该 Topic 的独立版本号；
%   - 数据必须匹配注册契约（字段、尺寸、有限数值、单位仅声明不换算）；
%   - 普通 Topic 不允许承载真值；真值 Topic 必须以 "truth/" 开头并在注册时声明；
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
end

methods
    function obj = aero_topic_bus()
        obj.specs = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.messages = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.topicVersions = containers.Map('KeyType', 'char', 'ValueType', 'any');
        obj.registeredNames = cell(1, 0);
        obj.globalSequence = uint64(0);
    end

    function register(obj, name, spec)
        name = obj.normalizeName(name);
        if obj.specs.isKey(name)
            error('aero:bus:duplicateTopic', 'Topic "%s" 已注册', name);
        end
        validateSpec(spec);
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

    function publish(obj, name, data, sampleTime, availableTime, source)
        name = obj.normalizeName(name);
        spec = obj.requireRegistered(name);

        if nargin < 5 || isempty(availableTime)
            availableTime = sampleTime;
        end
        if nargin < 6 || isempty(source)
            source = '';
        end

        obj.requireTime(sampleTime, 'sample_time_s');
        obj.requireTime(availableTime, 'available_time_s');
        if availableTime < sampleTime
            error('aero:bus:invalidTime', ...
                'Topic "%s" 的 available_time_s 不能早于 sample_time_s', name);
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

    function msg = latest(obj, name)
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
function validateSpec(spec)
required = {'fields', 'dimensions', 'units', 'allow_ground_truth', 'max_age_s'};
if ~isstruct(spec) || ~isscalar(spec)
    error('aero:bus:invalidSpec', 'spec 必须是 aero_topic_spec 生成的标量结构体');
end
for k = 1:numel(required)
    if ~isfield(spec, required{k})
        error('aero:bus:invalidSpec', 'spec 缺少字段：%s', required{k});
    end
end
end
