function runner(start_pos, end_pos)
    %% Set up experiment parameters
    dry_run = false; 
    no_rerun = true; 
    run_planners = true; 
    blend_schemes = {'none'};   
    control_schemes = {'switch'};   
    nav_task_type = "simple_env"; %"sampled"; %"smoke";  
    [starts, goals] = get_point_nav_tasks(nav_task_type); 
    hyperparam_set_type = "HJIPDE_update_methods"; %"time_vary_alpha_open"; %"default"; %"replan_zls"; %"HJIPDE_update_methods"
    hyperparam_sets = get_hyperparam_sets(hyperparam_set_type); 
    plot_level = 2; % 1 (at the end), 2 (every replan), 3 (every timestamp)
    
    %% Run Reach Avoid and BRS Planners
    if run_planners
        run_and_cache_planners(starts, goals); 
    end
    
    %% Run all experiments
    for i = start_pos:end_pos
        for h = 1:length(hyperparam_sets) 
            for k = 1:length(control_schemes)
                for j = 1:length(blend_schemes)
                    params = hyperparam_sets{h};  
                    params.start = starts{i};
                    params.goal = goals{i};
                    params.blend_scheme = blend_schemes{j}; 
                    params.control_scheme = control_schemes{k}; 
                    params.hyperparam_str = get_hyperparam_string(params); 
                    params.run_planner = false;
                    params.run_brs = false; 
                    exp = load_exp(params); 
                    pb = Planner(exp);
                    pb.plot_level = plot_level;
                    already_ran = exist(strcat(pb.output_folder, '/final_state.mat'), 'file'); 
                    if (already_ran && no_rerun) || dry_run
                        continue; 
                    end
                    pb.blend_plans(); 
                end 
            end 
        end 
    end 
end
