function cfg = aero_default_config()
%AERO_DEFAULT_CONFIG Return the default simulation configuration.
%
%   cfg = aero_default_config() returns the configuration struct accepted by
%   aero_sim. Fields:
%     tspan          [t0 tf] virtual simulation interval (s)
%     x0             initial continuous state ([] means use model.x0)
%     initial_command initial latched input applied to the dynamics
%     components     struct array of discrete components (see aero_validate_config)
%     solver         ode45 settings: RelTol, AbsTol, MaxStep, InitialStep
%     environment    free-form struct passed unchanged to model.dynamics
%     seed           recorded random seed (the kernel itself draws no random numbers)
%     time_tolerance grouping tolerance for simultaneous events ([] = machine scale)
%     results_root   directory for persisted results ([] = <project>/results)
%     save           struct('enabled', false, 'path', '', 'estimated_bytes', 0)
%     verbose        print progress information
%
%   This function is a pure constructor: it reads no global state.

cfg = struct();
cfg.tspan = [0, 1];
cfg.x0 = [];
cfg.initial_command = 0;
cfg.components = aero_empty_component();
cfg.solver = struct('RelTol', 1e-9, 'AbsTol', 1e-11, 'MaxStep', Inf, 'InitialStep', []);
cfg.environment = struct();
cfg.seed = 0;
cfg.time_tolerance = [];
cfg.results_root = '';
cfg.save = struct('enabled', false, 'path', '', 'estimated_bytes', 0);
cfg.verbose = false;

end
