function info = test_adaptive_integration()
%TEST_ADAPTIVE_INTEGRATION ode45 chooses interior points; they are not a grid.

model = struct();
model.name = 'oscillator';
model.x0 = 0;
model.dynamics = @(t, x, u, env) -400 * sin(50 * t); %#ok<INUSD>

solverLoose = struct('RelTol', 1e-6, 'AbsTol', 1e-8, 'MaxStep', Inf, 'InitialStep', []);
solverTight = struct('RelTol', 1e-9, 'AbsTol', 1e-11, 'MaxStep', Inf, 'InitialStep', []);

[xa, ia] = aero_integrate(model, 0, 0.5, 0, 0, struct(), solverLoose);
[xb, ib] = aero_integrate(model, 0, 0.5, 0, 0, struct(), solverTight);

analytic = (400 / 50) * (cos(50 * 0.5) - 1);
err = abs(xa - analytic);
check_that(err <= 1e-5, 'adaptive integration error %.3e exceeds 1e-5', err);
check_that(ia.internal_points >= 10, ...
    'only %d internal points for a 0.5 s oscillating segment', ia.internal_points);
check_that(ib.internal_points >= ia.internal_points, ...
    'tighter tolerance produced fewer points (%d vs %d)', ib.internal_points, ia.internal_points);

interior = ia.t(2:end - 1);
check_that(all(interior > 0 & interior < 0.5), ...
    'internal points lie outside the segment [0, 0.5]');
steps = diff(ia.t);
check_that(numel(unique(round(steps, 12))) > 1, 'internal steps are uniform (not adaptive)');

[xa2, ia2] = aero_integrate(model, 0, 0.5, 0, 0, struct(), solverLoose);
check_that(isequal(xa, xa2) && ia2.internal_points == ia.internal_points, ...
    'repeated integration is not deterministic');

info = struct('passed', true, 'summary', sprintf( ...
    '%d internal points (tight: %d), variable steps, max|error|=%.2e', ...
    ia.internal_points, ib.internal_points, err));

end
