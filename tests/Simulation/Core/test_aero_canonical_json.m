function info = test_aero_canonical_json()
%TEST_AERO_CANONICAL_JSON 实验身份规范化序列化的顺序/维度/特殊值行为

% 字段创建顺序不影响序列化结果
a = struct('b', 2, 'a', 1);
b = struct('a', 1, 'b', 2);
check_that(strcmp(aero_canonical_json(a), aero_canonical_json(b)), ...
    '字段创建顺序改变了规范 JSON');

% 数值变化必须改变结果
c = struct('a', 1, 'b', 3);
check_that(~strcmp(aero_canonical_json(a), aero_canonical_json(c)), ...
    '数值变化未改变规范 JSON');

% 数组元素顺序必须保留
v1 = struct('a', [1 2 3], 'b', 2);
v2 = struct('a', [3 2 1], 'b', 2);
check_that(~strcmp(aero_canonical_json(v1), aero_canonical_json(v2)), ...
    '数组元素顺序未进入规范 JSON');

% 数值维度必须保留（行/列向量不能哈希相同）
r = struct('a', [1 2], 'b', 2);
col = struct('a', [1; 2], 'b', 2);
check_that(~strcmp(aero_canonical_json(r), aero_canonical_json(col)), ...
    '数值维度未进入规范 JSON');

% cell 顺序必须保留
c1 = struct('a', {{1, 2}}, 'b', 2);
c2 = struct('a', {{2, 1}}, 'b', 2);
check_that(~strcmp(aero_canonical_json(c1), aero_canonical_json(c2)), ...
    'cell 顺序未进入规范 JSON');

% struct 数组元素顺序必须保留
s1 = struct('a', {1, 2});
s2 = struct('a', {2, 1});
check_that(~strcmp(aero_canonical_json(s1), aero_canonical_json(s2)), ...
    'struct 数组顺序未进入规范 JSON');

% NaN/Inf 明确编码，不静默转为 0
special = aero_canonical_json(struct('x', [NaN, Inf, -Inf]));
check_that(contains(special, '"NaN"') && contains(special, '"+Inf"') ...
    && contains(special, '"-Inf"'), 'NaN/Inf 未被显式编码');

% 函数句柄必须被拒绝
check_error(@() aero_canonical_json(struct('f', @sin)), ...
    'aero:json:unsupported', '函数句柄未被拒绝');

info = struct('passed', true, ...
    'summary', '规范 JSON：字段排序、维度/顺序保留、NaN/Inf 与句柄拒绝通过');

end
