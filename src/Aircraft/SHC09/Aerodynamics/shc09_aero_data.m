function aero = shc09_aero_data(source)
%SHC09_AERO_DATA 载入 SHC09 涵道/机翼气动系数（外部数据接口）
%
% 输入 source：
%   source.type = 'external'：从 MAT 文件读取原始系数表，要求包含
%                 变量 duct_raw（N×4）与 wing_raw（M×4），第一列为弧度角，
%                 其余列为系数。文件路径由 source.file 指定；为空时读取
%                 环境变量 AEROGNC_SHC09_AERO_FILE。
%   source.type = 'fixture' ：直接使用 source.tables 中的样条结构
%                 （仅用于测试替身，trust 固定为 'synthetic'）。
%
% 输出 aero：
%   dt, dn, dm       涵道样条（pp 形式）
%   wl, wd, wm       机翼样条（pp 形式）
%   duct_domain_rad  涵道迎角有效域
%   wing_domain_rad  机翼迎角有效域
%   trust            外部数据默认 'unverified'，调用方显式声明 'verified'
%                    （仅表示允许用于物理验证，来源仍需另行核对）；
%                    fixture 固定为 'synthetic'，永不可用于物理验证
%   source_type/source_file
%
% 气动原始表的再分发许可尚未确认，因此真实表不进入本仓库，只能由用户
% 在本地提供；缺失时必须显式失败，不得用合成数据冒充实测气动数据。

if nargin < 1
    source = struct('type', 'external', 'file', '');
end
if ~isstruct(source) || ~isfield(source, 'type')
    error('shc09:aero:source', 'aero source 必须是含 type 字段的结构体');
end

switch lower(char(source.type))
    case 'external'
        aero = load_external(source);
        aero = apply_trust(aero, source, 'unverified');
    case 'fixture'
        if ~isfield(source, 'tables') || isempty(source.tables)
            error('shc09:aero:source', 'fixture 模式必须提供 source.tables');
        end
        aero = source.tables;
        aero.source_type = 'fixture';
        aero.source_file = '';
        % 合成系数永远不是物理数据，且不接受调用方改写可信状态
        aero.trust = 'synthetic';
        aero.physical_validation_allowed = false;
    otherwise
        error('shc09:aero:source', '未知的 aero source.type：%s', char(source.type));
end

required = {'dt', 'dn', 'dm', 'wl', 'wd', 'wm', 'duct_domain_rad', 'wing_domain_rad'};
for k = 1:numel(required)
    if ~isfield(aero, required{k})
        error('shc09:aero:source', '气动数据缺少字段：%s', required{k});
    end
end

end

% ===========================================================================
function aero = load_external(source)
file = '';
if isfield(source, 'file')
    file = char(source.file);
end
if isempty(file)
    file = getenv('AEROGNC_SHC09_AERO_FILE');
end
if isempty(file)
    error('shc09:aero:dataMissing', [ ...
        '缺少 SHC09 气动数据文件。请设置 cfg 的 aero 数据路径或环境变量 ' ...
        'AEROGNC_SHC09_AERO_FILE（MAT 文件需含 duct_raw / wing_raw）。']);
end
if ~isfile(file)
    error('shc09:aero:dataMissing', '气动数据文件不存在：%s', file);
end

data = load(file);
if ~isfield(data, 'duct_raw') || ~isfield(data, 'wing_raw')
    error('shc09:aero:dataFormat', ...
        '文件 %s 必须包含 duct_raw 与 wing_raw', file);
end
duct = double(data.duct_raw);
wing = double(data.wing_raw);

aero = struct();
aero.dt = make_spline(duct, 'duct_raw');
aero.dn = make_spline(struct('data', duct(:, [1 3])), 'duct_raw');
aero.dm = make_spline(struct('data', duct(:, [1 4])), 'duct_raw');
aero.wl = make_spline(wing, 'wing_raw');
aero.wd = make_spline(struct('data', wing(:, [1 3])), 'wing_raw');
aero.wm = make_spline(struct('data', wing(:, [1 4])), 'wing_raw');
aero.duct_domain_rad = [duct(1, 1), duct(end, 1)];
aero.wing_domain_rad = [wing(1, 1), wing(end, 1)];
aero.source_type = 'external';
aero.source_file = file;
end

% ===========================================================================
function aero = apply_trust(aero, source, defaultTrust)
% 外部文件存在 ≠ 物理可信。默认标记为 'unverified'，只有调用方显式声明
% trust='verified' 才允许用于物理验证；来源与哈希由 metadata 单独记录。
trust = defaultTrust;
if isfield(source, 'trust') && ~isempty(source.trust)
    trust = lower(char(source.trust));
end
if ~any(strcmp(trust, {'unverified', 'verified'}))
    error('shc09:aero:trust', ...
        'aero source.trust 只能是 unverified 或 verified，收到 "%s"', trust);
end
aero.trust = trust;
aero.physical_validation_allowed = strcmp(trust, 'verified');
end

function pp = make_spline(table, name)
% 第一列角度必须严格递增；样条使用列 1 与列 2
if isstruct(table), table = table.data; end
if size(table, 1) < 2 || size(table, 2) < 2
    error('shc09:aero:dataFormat', '%s 至少需要 2 行 2 列', name);
end
angle = table(:, 1);
if any(diff(angle) <= 0)
    error('shc09:aero:dataFormat', '%s 的角度列必须严格递增', name);
end
pp = spline(angle, table(:, 2));
end
