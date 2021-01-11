function runner()
    load('./data/sampled_goals.mat');
    weights = [1, 10];
    zlsets = [0.1, 0.2];
    failed_exps = {};
    for i = 1:10
        for wi=1:2
            for zli=1:2
                pb.exp_name = 'tmp';
                try 
                    tic; 
                    close all;
                    params = default_hyperparams();
                    params.zero_level_set = zlsets(zli); 
                    params.spline_obs_weight = weights(wi); 
                    params.hyperparam_str = sprintf("replan_dt_%.3f_fine_tuned_spline_planner_zero_level_set_%.3f_spline_obs_weight_%f", params.replan_dt, params.zero_level_set, params.spline_obs_weight); 
                    s = starts(i, :); 
                    g = goals(i, :); 
                    params.start = s';
                    params.goal = goals(i, :)';
                    exp = load_exp(params); 
                    pb = Planner(exp); 
                    pb.blend_mpc_controls(); 
                    fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
                catch
                    fprintf("Failed %s\n", pb.exp_name); 
                    failed_exps{end+1} = pb.exp_name;
                end
            end 
        end 
        save('runner.mat');
    end
end
%               pb.exp_name = 'tmp';
%               try
%                 tic;
%                 close all;
%                 g = goals(i, :);
%                 s = starts(i, :);
%                 params = default_hyperparams();
%                 params.run_planner = true;
%                 params.zero_level_set = zlsets(j);
%                 params.start = s';
%                 params.goal = g';
%                 params.blending_scheme = 'probabilistic_blend_safety_control_traj';
%                 params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_level_set_%.2f", params.replan_dt, params.num_alpha_samples, params.zero_level_set); 
%                 exp = load_exp(params);
%                 pb = Planner(exp);
%                 pb.blend_mpc_traj(); 
%                 fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
%                 toc;
%                 if pb.termination_state == 0 %reached goal
%                     scores(1, j) = scores(1, j) + 1;
%                 end
%                 
%                 fprintf("%s_zls_%.3f %d/%d passed", params.blending_scheme, zlsets(j), scores(1, j), i);
%               catch 
%                  fprintf("Failed %s", pb.exp_name); 
%                  failed{length(failed)+1} = pb.exp_name; 
%               end 

    %         try
    %             tic;
    %             close all;
    %             g = goals(i, :);
    %             s = starts(i, :);
    %             params = default_hyperparams();
    %             params.run_planner = true;
    %             params.start = s';
    %             params.goal = g';
    %             params.blending_scheme = 'probabilistic_blend_safety_value_traj';
    %             params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_level_set_%.2f", params.replan_dt, params.num_alpha_samples, params.zero_level_set); 
    %             exp = load_exp(params);
    %             pb = Planner(exp);
    %             pb.blend_mpc_traj(); 
    %             fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
    %             toc;
    %             if pb.termination_state == 0 %reached goal
    %                 x2 = x2 + 1;
    %             end 
    %             fprintf("%s %d/%d passed", params.blending_scheme, x2, i); 
    %         catch 
    %             fprintf("FAILED %s\n", pb.exp_name);
    %         end 
    %             
    %         try 
    %             tic;
    %             close all;          
    %             g = goals(i, :);
    %             s = starts(i, :);
    %             params = default_hyperparams();
    %             params.run_planner = true;
    %             params.start = s';
    %             params.goal = g';
    %             params.blending_scheme = 'blend_safety_value_traj';
    %             params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
    %             exp = load_exp(params);
    %             pb = Planner(exp);
    %             pb.blend_mpc_traj(); 
    %             fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
    %             toc;
    %             if pb.termination_state == 0 %reached goal
    %                 x3 = x3 + 1;
    %             end
    %             fprintf("%s %d/%d passed", params.blending_scheme, x3, i); 
    %         catch 
    %             fprintf("FAILED %s\n", pb.exp_name);
    %         end
% 
%           try 
%                 tic;
%                 close all;          
%                 g = goals(i, :);
%                 s = starts(i, :);
%                 params = default_hyperparams();
%                 params.run_planner = true;
%                 params.start = s';
%                 params.goal = g';
%                 params.zero_level_set = zlsets(j);
%                 params.blending_scheme = 'switch';
%                 params.hyperparam_str = sprintf("replan_dt_%.3f_zero_level_set_%.3f", params.replan_dt, params.zero_level_set); 
%                 exp = load_exp(params);
%                 pb = Planner(exp);
%                 pb.blend_mpc_controls(); 
%                 fprintf("Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
%                 toc;
%                 if pb.termination_state == 0 %reached goal
%                     scores(2, j) = scores(2, j) + 1;
%                 end 
%                 fprintf("%s_zls_%.3f %d/%d passed", params.blending_scheme, zlsets(j), scores(2, j), i); 
%             catch 
%                 fprintf("FAILED %s\n", pb.exp_name);
%                  failed{length(failed)+1} = pb.exp_name; 
%             end
%                 save('./outputs/results.mat');

