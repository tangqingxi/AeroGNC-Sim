function text = wtr_json(value)
%WTR_JSON 将 MATLAB 值序列化为 JSON 文本（WindTrace 导出专用）
%
% 支持：char/string、logical、数值标量/向量/矩阵、struct 标量与数组、
% cell 数组、空值。Inf/NaN 按 WindTrace 契约写成
% {"special_float":"+Inf"|"-Inf"|"NaN"}，不静默转 0。
%
% 该函数只做序列化，不做协议校验；字段顺序保持 struct 的 fieldnames 顺序。

text = encode(value);

end

% ===========================================================================
function out = encode(value)
if ischar(value)
    out = ['"', escape(value), '"'];
elseif isstring(value)
    if ~isscalar(value)
        error('wtr:json:string', '只支持标量字符串');
    end
    out = ['"', escape(char(value)), '"'];
elseif islogical(value)
    if isscalar(value)
        out = ternary(value, 'true', 'false');
    else
        out = array_text(value);
    end
elseif isnumeric(value)
    if isempty(value)
        out = '[]';
    elseif isscalar(value)
        if isnan(value)
            out = '{"special_float":"NaN"}';
        elseif isinf(value)
            out = ['{"special_float":"', ternary(value > 0, '+Inf', '-Inf'), '"}'];
        else
            out = sprintf('%.17g', double(value));
        end
    else
        out = array_text(value);
    end
elseif isa(value, 'containers.Map')
    keys = value.keys;
    parts = cell(1, numel(keys));
    for k = 1:numel(keys)
        parts{k} = ['"', escape(keys{k}), '":', encode(value(keys{k}))];
    end
    out = ['{', strjoin(parts, ','), '}'];
elseif isstruct(value)
    if isscalar(value)
        out = struct_text(value);
    else
        parts = cell(1, numel(value));
        for k = 1:numel(value)
            parts{k} = struct_text(value(k));
        end
        out = ['[', strjoin(parts, ','), ']'];
    end
elseif iscell(value)
    parts = cell(1, numel(value));
    for k = 1:numel(value)
        parts{k} = encode(value{k});
    end
    out = ['[', strjoin(parts, ','), ']'];
else
    error('wtr:json:type', '不支持的 JSON 值类型：%s', class(value));
end
end

% ===========================================================================
function out = struct_text(value)
names = fieldnames(value);
parts = cell(1, numel(names));
for k = 1:numel(names)
    parts{k} = ['"', escape(names{k}), '":', encode(value.(names{k}))];
end
out = ['{', strjoin(parts, ','), '}'];
end

% ===========================================================================
function out = array_text(value)
if isvector(value)
    parts = cell(1, numel(value));
    for k = 1:numel(value)
        parts{k} = encode(value(k));
    end
    out = ['[', strjoin(parts, ','), ']'];
else
    rows = cell(1, size(value, 1));
    for r = 1:size(value, 1)
        rows{r} = array_text(value(r, :));
    end
    out = ['[', strjoin(rows, ','), ']'];
end
end

% ===========================================================================
function out = escape(text)
text = char(text);
text = strrep(text, '\', '\\');
text = strrep(text, '"', '\"');
text = strrep(text, newline, '\n');
text = strrep(text, sprintf('\r'), '\r');
text = strrep(text, sprintf('\t'), '\t');
out = text;
end

function out = ternary(condition, a, b)
if condition
    out = a;
else
    out = b;
end
end
