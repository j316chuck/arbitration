function runner(start_pos, end_pos)
    %% Set up experiment parameters
    no_rerun = true; 
    dry_run = false; 
    run_planners = false; 
    blend_schemes = {'time_vary_alpha_open_loop'};    
    control_schemes = {'switch'};   
    nav_task_type = "sampled"; %"smoke";  
    [starts, goals] = get_point_nav_tasks(nav_task_type); 
    hyperparam_set_type = "time_vary_alpha_open"; %"default"; %"replan_zls";  
    hyperparam_sets = get_hyperparam_sets(hyperparam_set_type); 
    N = length(starts); 
    plot_level = 2; % 1 (at the end), 2 (every replan), 3 (every timestamp)
    
    %% Run Reach Avoid and BRS Planners
    if run_planners
        run_and_cache_planners(N, starts, goals); 
    end
    
    %% Run all experiments
    for i = start_pos:end_pos
        for h = 1:length(hyperparam_sets) 
            for k = 1:length(control_schemes)
                for j = 1:length(blend_schemes)
                    try
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
                    catch 
                        fprintf("Failed exp: %s\n", pb.exp_name);
                    end 
                end 
            end 
        end 
    end 
end
