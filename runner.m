function runner(start_pos)
    %% Set up experiment parameters
    no_rerun = true; 
    run_planners = false; 
    blend_schemes = {'time_vary_alpha_open_loop_safety_control', 'sample_safety_control'};    
    control_schemes = {'switch'};   
    nav_task_type = "sampled"; %"smoke";  
    [starts, goals] = get_point_nav_tasks(nav_task_type); 
    hyperparam_set_type = "default"; %"replan_zls";  
    hyperparam_sets = get_hyperparam_sets(hyperparam_set_type); 
    
    %% Run Reach Avoid and BRS Planners
    if run_planners
        N = length(starts); 
        run_and_cache_planners(N, starts, goals); 
    end
    
    %% Run all experiments
    exp_names = {}; 
    failed_cases = {}; 
    results = []; 
    elapsed_time = [];
    exp_name = ""; 
    termination_state = -1;
    for i = start_pos:length(starts)
        for h = 1:length(hyperparam_sets) 
            for k = 1:length(control_schemes)
                for j = 1:length(blend_schemes)
                    tic;  
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
                        pb.plot_level = 1;
                        already_ran = exist(strcat(pb.output_folder, '/final_state.mat'), 'file'); 
                        if already_ran && no_rerun
                            continue; 
                        end
                        pb.blend_plans(); 
                        exp_name = pb.exp_name; 
                        termination_state = pb.termination_state; 
                    catch 
                        exp_name = pb.exp_name; 
                        termination_state = -1; 
                        failed_cases{end+1} = pb.exp_name; 
                    end 
                    if isempty(termination_state)
                        termination_state = -1;     
                    end 
                    exp_names{end+1} = exp_name; 
                    results(end+1) = termination_state;
                    elapsed_time(end+1) = toc; 
                    save('runner.mat', 'exp_names', 'results', 'elapsed_time', 'failed_cases'); 
                    close all;
                end 
            end 
        end 
    end 
end