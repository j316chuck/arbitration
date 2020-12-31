function runner()
    load('./outputs/sampled_goals.mat')
    sample_run_so_break_early = false;
    N = length(goals);
    for i = 1:25
        for replan_dt = 0.5:1.0:1.5
            for zls = 0:0.1:0.1
                tic;
                close all;
                g = goals(i, :);
                s = starts(i, :);
                params = default_hyperparams();
                params.run_planner = true;
                params.start = s';
                params.goal = g';
                params.zero_level_set = zls;
                params.replan_dt = replan_dt;
                params.blending_scheme = 'probabilistic_blend_safety_control_traj';
                params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_level_set_%.2f", params.replan_dt, params.num_alpha_samples, zls); 
                exp = load_exp(params);
                pb = Planner(exp);
                pb.blend_mpc_traj(); 
                fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
                toc;
                fprintf("FAILED %s\n", pb.exp_name);
            end 
            
            try 
                tic;
                close all;
                g = goals(i, :);
                s = starts(i, :);
                params = default_hyperparams();
                params.run_planner = true;
                params.start = s';
                params.goal = g';
                alpha = 0.2;
                params.alpha = alpha;
                params.replan_dt = replan_dt;
                params.blending_scheme = 'blend_safety_value_traj';
                params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
                exp = load_exp(params);
                pb = Planner(exp);
                pb.blend_mpc_traj(); 
                fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
                toc;
            catch 
                fprintf("FAILED %s\n", pb.exp_name);
            end
        end
    end 
end 
