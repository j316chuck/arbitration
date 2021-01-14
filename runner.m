function runner()
    N = 10; 
    run_and_cache_reach_avoid_and_brs_planners(N); 
    %run_option_0_switch_blend_baselines(N);
    %run_option_1_safety_traj_blend(N);
    %run_option_1_5_probabilistic_safety_traj_blend(N);
    %run_option_2_safety_value_blend_control_traj(N); 
    %run_option_2_safety_value_blend_value_traj(N); 
    %run_option_4_triangle_blend(N);
    %run_option_1_75_probabilistic_value_traj_blend(N); 
    %run_option_2_time_varying_value_blend_control_traj(N);
end

function run_and_cache_reach_avoid_and_brs_planners(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename, 'starts', 'goals');
    for i = 1:N
        close all;
        params = default_hyperparams();
        params.run_planner = true; 
        if i == 1 
            params.run_brs = true;
        else 
            params.run_brs = false; 
        end 
        s = starts(i, :); 
        g = goals(i, :); 
        params.start = s';
        params.goal = g';
        exp = load_exp(params);
        Planner(exp); % initialize exp and run reach avoid and brs planner
    end 
end 


function run_option_0_switch_blend_baselines(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename, 'starts', 'goals');
    failed_exps = {}; 
    weights = [1, 10];
    zlsets = [0.1, 0.2];
    for i = 1:N
        for wi=1:2
            for zli=1:2
                pb.exp_name = 'tmp';
                try 
                    tic; 
                    close all;
                    params = default_hyperparams();
                    params.zero_level_set = zlsets(zli); 
                    params.spline_obs_weight = weights(wi); 
                    params.blending_scheme = 'switch'; 
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
                save('runner.mat');
            end 
        end 
    end
end 

function run_option_1_safety_traj_blend(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    alphas = [0.4, 0.6, 0.8]; 
    for j = 1:3
        for i = 1:N
            pb.exp_name = 'tmp';
            try
                tic; 
                close all;
                params = default_hyperparams();
                params.blending_scheme =  'blend_safety_control_traj'; 
                params.spline_obs_weight = 1;
                params.alpha = alphas(j); 
                params.zero_level_set = 0.2;
                params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f_spline_obs_weight_%f", params.replan_dt, params.alpha, params.spline_obs_weight); %option 1 alpha
                params.run_planner = true;
                s = starts(i, :); 
                g = goals(i, :); 
                params.start = s';
                params.goal = goals(i, :)';
                exp = load_exp(params);
                pb = Planner(exp); 
                pb.blend_mpc_traj(); 
                fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
            catch 
                fprintf("Failed %s\n", pb.exp_name); 
                failed_exps{end+1} = pb.exp_name;
            end 
            save('runner.mat');
        end
    end 
end 

function run_option_1_5_probabilistic_safety_traj_blend(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    use_safety_traj = {false, true};
    for j = 1:2
        for i = 1:N
            pb.exp_name = 'tmp';
            try 
                tic; 
                close all;
                params = default_hyperparams();
                params.blending_scheme =  'probabilistic_blend_safety_control_traj'; 
                params.spline_obs_weight = 1;
                params.num_alpha_samples = 20; 
                params.zero_level_set = 0.2;
                params.use_safe_orig_traj = use_safety_traj{j}; 
                params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_use_safe_%d_level_set_%.2f_spline_obs_weight_%f", params.replan_dt, params.num_alpha_samples, params.use_safe_orig_traj, params.zero_level_set, params.spline_obs_weight); %option 1.5 probabilistic alpha
                params.run_planner = true;
                s = starts(i, :); 
                g = goals(i, :); 
                params.start = s';
                params.goal = goals(i, :)';
                exp = load_exp(params);
                pb = Planner(exp); 
                pb.blend_mpc_traj(); 
                fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
            catch
                fprintf("Failed %s\n", pb.exp_name); 
                failed_exps{end+1} = pb.exp_name;
            end
            save('runner.mat');
        end
    end 
end 

function run_option_1_75_probabilistic_value_traj_blend(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    for i = 1:N
        pb.exp_name = 'tmp';
        tic; 
        close all;
        params = default_hyperparams();
        params.blending_scheme =  'probabilistic_blend_safety_value_traj'; 
        params.run_planner=true;
        params.alpha = 0.2; 
        params.spline_obs_weight = 1;
        params.num_alpha_samples = 20; 
        params.zero_level_set = 0.2;
        params.use_safe_orig_traj = true; 
        params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_use_safe_%d_level_set_%.2f_spline_obs_weight_%f", params.replan_dt, params.num_alpha_samples, params.use_safe_orig_traj, params.zero_level_set, params.spline_obs_weight); %option 1.5 probabilistic alpha
        s = starts(i, :); 
        g = goals(i, :); 
        params.start = s';
        params.goal = goals(i, :)';
        exp = load_exp(params);
        pb = Planner(exp); 
        pb.blend_mpc_traj(); 
        fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
        save('runner.mat');
    end
end 

function run_option_2_safety_value_blend_control_traj(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    for i = 1:N
        pb.exp_name = 'tmp';
        tic; 
        close all;
        params = default_hyperparams();
        params.blending_scheme =  'value_blend_safety_control_traj'; 
        params.run_planner = true;
        params.alpha = 0.2; 
        params.spline_obs_weight = 1;
        params.hyperparam_str = sprintf("replan_dt_%.3f_spline_obs_weight_%f", params.replan_dt, params.spline_obs_weight); %option 2 value
        s = starts(i, :); 
        g = goals(i, :); 
        params.start = s';
        params.goal = goals(i, :)';
        exp = load_exp(params);
        pb = Planner(exp); 
        pb.blend_mpc_traj(); 
        fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
        save('runner.mat');
    end
end 


function run_option_2_safety_value_blend_value_traj(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    for i = 1:N
        pb.exp_name = 'tmp';
        tic; 
        close all;
        params = default_hyperparams();
        params.blending_scheme =  'value_blend_safety_value_traj'; 
        params.run_planner = true;
        params.alpha = 0.2; 
        params.spline_obs_weight = 1;
        params.hyperparam_str = sprintf("replan_dt_%.3f_spline_obs_weight_%f", params.replan_dt, params.spline_obs_weight); %option 2 value
        s = starts(i, :); 
        g = goals(i, :); 
        params.start = s';
        params.goal = goals(i, :)';
        exp = load_exp(params);
        pb = Planner(exp); 
        pb.blend_mpc_traj(); 
        fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
        save('runner.mat');
    end
end 

function run_option_4_triangle_blend(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    zlsets = [0.2; 0.1];
    replan_sets = [0.5; 0.35]; 
    for j = 1:2
        for i = 1:N
            pb.exp_name = 'tmp';
            try 
                tic; 
                close all;
                params = default_hyperparams();
                params.zero_level_set = zlsets(j);
                params.replan_level_set = replan_sets(j); 
                params.blending_scheme =  'replan_waypoint'; %'probabilistic_blend_safety_control_traj'; %'switch'; 
                params.hyperparam_str = sprintf("replan_dt_%.3f_zero_level_set_%.3f_replan_level_set_%.3f_spline_obs_weight_%f_max_num_candidates_%f", params.replan_dt, params.zero_level_set, params.replan_level_set, params.spline_obs_weight, params.replan_max_num_candidates); 
                params.run_planner = true;
                s = starts(i, :); 
                g = goals(i, :); 
                params.start = s';
                params.goal = goals(i, :)';
                exp = load_exp(params);
                pb = Planner(exp); 
                pb.blend_and_replan_mpc_controls(); 
                fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 
            catch
                fprintf("Failed %s\n", pb.exp_name); 
                failed_exps{end+1} = pb.exp_name;
            end
            save('runner.mat');
        end
    end 
end 

function run_option_2_time_varying_value_blend_control_traj(N)
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/sampled_goals.mat');
    load(filename);
    failed_exps = {}; 
    for i = 1:N
        pb.exp_name = 'tmp';
        
        tic; 
        close all;
        params = default_hyperparams();
        params.blending_scheme = 'time_varying_value_blend_safety_control_traj'; 
        params.hyperparam_str = sprintf("replan_dt_%.3f_spline_obs_weight_%f", ...
            params.replan_dt, params.spline_obs_weight); 
        s = starts(i, :); 
        g = goals(i, :); 
        params.start = s';
        params.goal = goals(i, :)';
        exp = load_exp(params); 
        pb = Planner(exp); 
        pb.blend_mpc_traj(); 
        fprintf("Time: %f (sec) Start: [%.2f %.2f %.2f] End: [%.2f %.2f %.2f] Result: %d\n", ...
            num2str(toc), s(1), s(2), s(3), g(1), g(2), g(3), pb.termination_state); 

        %fprintf("Failed %s\n", pb.exp_name); 
        %failed_exps{end+1} = pb.exp_name;

        save('runner.mat');
    end
end 