function runner()
    load('./outputs/sampled_goals.mat')
    sample_run_so_break_early = false;
    N = length(goals);
    for i = 1:10
        for alpha = [ 0.6, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.8, 0.9, 1.0]
            for blending_scheme = {'blend_safety_value_traj', 'blend_safety_control_traj'}
                tic; 
                try 
                close all;
                g = goals(i, :);
                s = starts(i, :);
                params = default_hyperparams();
                params.run_planner = true;
                params.start = s';
                params.goal = g';
                params.alpha = alpha;
                params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
                params.blending_scheme = blending_scheme{1};
                exp = load_exp(params);
                pb = Planner(exp);
                pb.blend_mpc_traj(); 
                fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
                toc; 
            end
        end
    end 
end 