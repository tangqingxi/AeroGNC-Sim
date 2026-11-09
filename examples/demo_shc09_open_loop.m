function demo_shc09_open_loop(dataFile)
%DEMO_SHC09_OPEN_LOOP SHC09 开环仿真示例（需要外部气动数据）
%
% 用法：
%   demo_shc09_open_loop                       % 读取环境变量 AEROGNC_SHC09_AERO_FILE
%   demo_shc09_open_loop('path/to/aero.mat')   % 显式指定数据文件
%
% 数据文件需包含 duct_raw（N×4）与 wing_raw（M×4）系数表；因再分发许可
% 尚未确认，真实气动表不随仓库发布。
%
% 场景：悬停配平起飞，0.5 s 后六舵同向 +2°，1.0 s 后电机目标 +2%。

root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
aero_setup();

if nargin < 1 || isempty(dataFile)
    dataFile = getenv('AEROGNC_SHC09_AERO_FILE');
end
if isempty(dataFile)
    fprintf(['未提供 SHC09 气动数据。\n' ...
        '请设置环境变量 AEROGNC_SHC09_AERO_FILE，或传入数据文件路径。\n' ...
        '结构测试可用 tests/run_tests.m（使用明确的合成系数，不代表物理结果）。\n']);
    return;
end

src = struct('type', 'external', 'file', dataFile, 'tables', []);
model = shc09_model(struct('aero_source', src));
trim = model.trim;

fprintf('SHC09 悬停配平：Omega=%.2f rad/s, Ve=%.2f m/s, delta=%.3f deg\n', ...
    trim.omega_rad_s, trim.exit_speed_m_s, trim.delta_deg);

cfg = aero_default_config();
cfg.tspan = [0, 2];
cfg.x0 = model.x0;
cfg.initial_command = model.initial_command;
cfg.verbose = true;
cfg.components = struct( ...
    'name', {'servo_cmd', 'motor_cmd', 'servo_latch', 'motor_latch'}, ...
    'kind', 'periodic', ...
    'period', {0.01, 0.02, 0.005, 0.02}, ...
    'priority', {40, 41, 50, 55}, ...
    'latches', {false, false, true, true}, ...
    'step', { ...
        @(t, bus) set_servos(bus, trim.delta_rad * ones(6, 1), t), ...
        @(t, bus) set_motor(bus, trim.omega_rad_s, t), ...
        @(t, bus) latch_servos(bus), ...
        @(t, bus) latch_motor(bus)});

result = aero_sim(model, cfg);

s = model.state_layout;
fprintf('\n仿真完成：%d 段，%d 个事件\n', numel(result.segments), numel(result.events));
fprintf('末态位置 [%.4f %.4f %.4f] m\n', result.state(end, s.position));
fprintf('末态姿态四元数 [%.4f %.4f %.4f %.4f]\n', result.state(end, s.quaternion));
fprintf('末态舵角均值 %.3f deg，转速 %.1f rad/s\n', ...
    rad2deg(mean(result.state(end, s.servo))), result.state(end, s.motor));

end

function bus = set_servos(bus, base, t)
if t >= 0.5 - 1e-12
    bus.command.servos = base + deg2rad(2);
else
    bus.command.servos = base;
end
end

function bus = set_motor(bus, base, t)
if t >= 1.0 - 1e-12
    bus.command.motor = base * 1.02;
else
    bus.command.motor = base;
end
end

function bus = latch_servos(bus)
bus.held_command.servos = bus.command.servos;
end

function bus = latch_motor(bus)
bus.held_command.motor = bus.command.motor;
end
