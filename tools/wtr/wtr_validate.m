function report = wtr_validate(path)
%WTR_VALIDATE 独立校验 windtrace.flight.v1 容器（MATLAB 侧）
%
% 输入：path 为 .wtr 文件。
% 输出：report（passed、run_id、topic_count、duration_s、sha256）。
%
% 校验项与 WindTrace core/flight.py 保持一致的最小集合：
%   - 容器仅含 4 个成员，成员长度/SHA256 与 manifest 一致；
%   - schema/run_id/duration/核心 topic 齐全；
%   - 每个 topic 的时间非负严格递增，timestamp=round(time*1e6) 为 uint64；
%   - 字段宽度、有限性、四元数单位模长与声明一致；
%   - geometry 为 FRD，六舵面网格齐全。
% 该函数只读文件，不修改容器；失败即报错，不返回“猜测通过”。

if ~isfile(path)
    error('wtr:validate:file', 'WTR 文件不存在：%s', path);
end

stage = tempname();
mkdir(stage);
cleanup = onCleanup(@() rmdir_if_exists(stage)); %#ok<NASGU>
unzip(path, stage);

required = {'manifest.json', 'parameters.json', 'flight.mat', 'geometry.json'};
present = dir(stage);
present = present(~[present.isdir]);
names = {present.name};
if numel(names) ~= 4 || ~isempty(setdiff(required, names)) || ~isempty(setdiff(names, required))
    error('wtr:validate:members', '容器成员不合法：%s', strjoin(names, ', '));
end

manifest = jsondecode(fileread(fullfile(stage, 'manifest.json')));
if ~strcmp(manifest.schema, 'windtrace.flight.v1')
    error('wtr:validate:schema', 'schema 不是 windtrace.flight.v1');
end
if ~ischar(manifest.run_id) || isempty(manifest.run_id)
    error('wtr:validate:run_id', '缺少 run_id');
end
if ~isfield(manifest, 'method') || ~isfield(manifest, 'execution') ...
        || ~isfield(manifest, 'audit') || ~isfield(manifest, 'source')
    error('wtr:validate:meta', 'manifest 缺少 method/execution/audit/source');
end

% 成员哈希与长度（jsondecode 把点替换为下划线）
memberNames = {'parameters.json', 'flight.mat', 'geometry.json'};
for k = 1:numel(memberNames)
    name = memberNames{k};
    key = strrep(name, '.', '_');
    if ~isfield(manifest.members, key)
        error('wtr:validate:members', 'manifest.members 缺少 %s', name);
    end
    info = manifest.members.(key);
    actualHash = wtr_sha256(fullfile(stage, name));
    d = dir(fullfile(stage, name));
    if ~strcmpi(info.sha256, actualHash) || info.bytes ~= d(1).bytes
        error('wtr:validate:hash', '%s 的 SHA256/长度不匹配', name);
    end
end

flight = load(fullfile(stage, 'flight.mat'));
topics = flight.topics;
if ~isfield(flight, 'parameter_arrays')
    error('wtr:validate:mat', 'flight.mat 缺少 parameter_arrays');
end

topicSpecs = manifest.topics;
requiredTopics = {'vehicle_local_position', 'vehicle_local_position_groundtruth', ...
    'vehicle_attitude', 'vehicle_attitude_groundtruth', ...
    'trajectory_setpoint', 'vehicle_attitude_reference', 'shc09_vanes'};
if ~all(isfield(topicSpecs, requiredTopics))
    error('wtr:validate:topics', 'manifest 缺少核心 topic');
end
if ~isequal(sort(fieldnames(topics)), sort(fieldnames(topicSpecs)))
    error('wtr:validate:topics', 'flight.mat 与 manifest 的 topic 集合不一致');
end

duration = double(manifest.duration_s);

topicNames = fieldnames(topics);
for t = 1:numel(topicNames)
    name = topicNames{t};
    topic = topics.(name);
    spec = topicSpecs.(name);
    if ~isfield(topic, 'time_s') || ~isfield(topic, 'timestamp')
        error('wtr:validate:time', '%s 缺少 time_s/timestamp', name);
    end
    time = double(topic.time_s(:));
    stamp = uint64(topic.timestamp(:));
    if numel(time) < 2 || any(~isfinite(time)) || any(time < 0) || any(diff(time) <= 0)
        error('wtr:validate:time', '%s 时间必须有限、非负、严格递增', name);
    end
    if ~isequal(stamp, uint64(round(time * 1e6)))
        error('wtr:validate:time', '%s 的 microsecond timestamp 不匹配', name);
    end
    if time(end) > duration + 1e-9
        error('wtr:validate:time', '%s 时间超出 duration_s', name);
    end
    fields = fieldnames(spec.fields);
    if ~isequal(sort(fields), sort(setdiff(fieldnames(topic), {'time_s', 'timestamp'})))
        error('wtr:validate:fields', '%s 字段集合与声明不一致', name);
    end
    for f = 1:numel(fields)
        field = fields{f};
        value = topic.(field);
        desc = spec.fields.(field);
        value = reshape(value, numel(time), []);
        if size(value, 2) ~= desc.width
            error('wtr:validate:width', '%s.%s 宽度不匹配', name, field);
        end
        if any(~isfinite(value))
            error('wtr:validate:finite', '%s.%s 含非有限值', name, field);
        end
        if strcmp(field, 'q')
            norms = sqrt(sum(value .^ 2, 2));
            if any(abs(norms - 1) > 1e-8)
                error('wtr:validate:quat', '%s.q 未归一化', name);
            end
        end
    end
end

geometry = jsondecode(fileread(fullfile(stage, 'geometry.json')));
if ~strcmp(geometry.schema, 'windtrace.geometry.v1') || ~strcmp(geometry.frame, 'FRD')
    error('wtr:validate:geometry', '几何 schema/frame 不合法');
end
if ~isfield(geometry, 'meshes') || numel(geometry.meshes.vanes) ~= 6
    error('wtr:validate:geometry', '缺少六舵面几何');
end

report = struct();
report.passed = true;
report.run_id = manifest.run_id;
report.topic_count = numel(topicNames);
report.duration_s = duration;
report.sha256 = wtr_sha256(path);

end

% ===========================================================================
function rmdir_if_exists(path)
if isfolder(path)
    rmdir(path, 's');
end
end
