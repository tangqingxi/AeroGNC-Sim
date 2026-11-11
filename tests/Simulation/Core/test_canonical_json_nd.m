function info = test_canonical_json_nd()
%TEST_CANONICAL_JSON_ND 验证完整 N 维尺寸及稳定身份；无输入，输出测试结果。
a = reshape(1:24, [2 3 4]);
b = reshape(1:24, [2 3 2 2]);
j = jsondecode(aero_canonical_json(a));
check_that(isequal(j.dims(:)', [2 3 4]), '三维尺寸未完整编码');
check_that(isequal(j.data(:), a(:)), '列主序数据被改变');
check_that(~strcmp(identity(a), identity(b)), '不同 N 维尺寸出现相同身份');
left = struct('z', b, 'a', a);
right = struct('a', a, 'z', b);
check_that(strcmp(identity(left), identity(right)), 'N 维字段排序影响身份');
for shape = {[0 3 4], [2 0 3 4], [0 0], [1 0 2]}
    empty = zeros(shape{1});
    parsed = jsondecode(aero_canonical_json(empty));
    check_that(isequal(parsed.dims(:)', size(empty)), '空数组尺寸丢失');
    check_that(isempty(parsed.data), '空数组数据不为空');
end
check_that(~strcmp(identity(zeros(2,0,3)), identity(zeros(2,0,4))), ...
    '不同空数组尺寸出现相同身份');
logicalArray = false(2,3,4);
parsed = jsondecode(aero_canonical_json(logicalArray));
check_that(isequal(parsed.dims(:)', size(logicalArray)), '逻辑数组尺寸丢失');
info = struct('passed', true, 'summary', '3D/4D、字段排序、列主序、空数组和逻辑数组尺寸通过');
end

function sha = identity(value)
% 复用生产身份算法，验证序列化结果的 SHA256。
sha = aero_sha256_text(aero_canonical_json(value));
end
