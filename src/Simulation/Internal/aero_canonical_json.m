function text = aero_canonical_json(value)
%AERO_CANONICAL_JSON 稳定、可复现的递归 JSON 序列化（实验身份专用）
%
% 规则：
%   - struct 字段按字母排序；struct 数组元素顺序保持；
%   - cell 元素顺序保持；
%   - 数值/逻辑保留维度：{"type":"numeric","dims":[r,c],"data":[...]}；
%   - NaN/Inf 写成字符串，不静默转 0；
%   - 函数句柄/对象/未知类型直接报错；实验定义不得含此类值；
%   - 字符串按 JSON 转义；数值按 %.17g。
%
% 这是实验身份哈希的唯一序列化入口，不用于 WTR 文件编码。

if isa(value, 'function_handle') || isobject(value)
    error('aero:json:unsupported', '实验身份不能包含函数/对象句柄');
end
text = encode(value);

end

% ===========================================================================
function out = encode(value)
if isa(value, 'function_handle') || isobject(value)
    error('aero:json:unsupported', '实验身份不能包含函数/对象句柄');
end
if ischar(value)
    if size(value, 1) > 1
        out = struct_text(struct('type', 'char_matrix', ...
            'dims', size(value), 'data', cellstr(value)));
    else
        out = ['"', escape(value), '"'];
    end
elseif isstring(value)
    if ~isscalar(value)
        error('aero:json:string', '只支持标量字符串');
    end
    out = ['"', escape(char(value)), '"'];
elseif islogical(value)
    out = typed_array('logical', value);
elseif isnumeric(value)
    if ~isreal(value)
        error('aero:json:complex', '实验身份不能包含复数');
    end
    out = typed_array('numeric', value);
elseif iscell(value)
    parts = cell(1, numel(value));
    for k = 1:numel(value)
        parts{k} = encode(value{k});
    end
    out = ['[', strjoin(parts, ','), ']'];
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
else
    error('aero:json:type', '不支持的实验身份类型：%s', class(value));
end
end

function out = struct_text(value)
names = sort(fieldnames(value));
parts = cell(1, numel(names));
for k = 1:numel(names)
    parts{k} = ['"', escape(names{k}), '":', encode(value.(names{k}))];
end
out = ['{', strjoin(parts, ','), '}'];
end

function out = typed_array(kind, value)
if isempty(value)
    out = ['{"type":"', kind, '","dims":[', num2str(size(value, 1)), ',', ...
        num2str(size(value, 2)), '],"data":[]}'];
    return;
end
dims = size(value);
data = value(:);
parts = cell(1, numel(data));
for k = 1:numel(data)
    if islogical(data)
        parts{k} = ternary(data(k), 'true', 'false');
    else
        v = double(data(k));
        if isnan(v)
            parts{k} = '"NaN"';
        elseif isinf(v)
            parts{k} = ternary(v > 0, '"+Inf"', '"-Inf"');
        else
            parts{k} = sprintf('%.17g', v);
        end
    end
end
out = ['{"type":"', kind, '","dims":[', num2str(dims(1)), ',', ...
    num2str(dims(2)), '],"data":[', strjoin(parts, ','), ']}'];
end

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
