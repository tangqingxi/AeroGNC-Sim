function info = test_state_dimension()
%TEST_STATE_DIMENSION A three-state model needs no kernel change.

lambda = [1; 2; 3];
model = diagonal_model(lambda);
cfg = aero_default_config();
cfg.tspan = [0, 0.5];
cfg.x0 = zeros(3, 1);
cfg.initial_command = 1;

res = aero_sim(model, cfg);

expected = (1 ./ lambda) .* (1 - exp(-lambda * res.time.'));
err = max(abs(res.state.' - expected), [], 'all');

check_that(size(res.state, 2) == 3, 'state width %d != 3', size(res.state, 2));
check_that(err <= 1e-9, 'three-state model: max error %.3e exceeds 1e-9', err);

% a one-state run of the same kernel must still work
cfg1 = cfg;
cfg1.x0 = 0;
model1 = first_order_model(1, 1);
res1 = aero_sim(model1, cfg1);
check_that(size(res1.state, 2) == 1, 'one-state run produced %d states', size(res1.state, 2));

info = struct('passed', true, 'summary', sprintf( ...
    '3-state max error=%.2e, 1-state run ok (state widths 3 and 1)', err));

end
