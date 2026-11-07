function m = aero_measurement(bus, name)
%AERO_MEASUREMENT 读取指定名称的最新可用测量
%
% 输入：
%   bus  - 内核传给组件的总线结构
%   name - 测量名称（对应 sensor 组件的 name）
%
% 输出：
%   m    - 最新一条测量结构（name/sample_time/available_time/value）；
%          尚未发布过时返回 []
%
% 只有已经发布（available_time <= 当前时刻）的测量会出现在 bus 中，
% 因此该函数不会返回“未来”的数据。

m = [];
if ~isstruct(bus) || ~isfield(bus, 'measurements') || isempty(bus.measurements)
    return;
end
entries = bus.measurements;
for k = numel(entries):-1:1
    if strcmp(entries{k}.name, char(name))
        m = entries{k};
        return;
    end
end

end
