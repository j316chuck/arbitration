function count_exps_done()
    %% Set up experiment parameters
    start_pos = 1;
    end_pos = 100;
    blend_schemes = {'replan_safe_traj', 'none', 'sample_safety_control', 'time_vary_alpha_open_loop_safety_control'};    
    control_schemes = {'switch'};   
    nav_task_type = "sampled"; %"smoke";  
    [starts, goals] = get_point_nav_tasks(nav_task_type); 
    hyperparam_set_type = "default"; %"default"; %"replan_zls";  
    hyperparam_sets = get_hyperparam_sets(hyperparam_set_type); 
    
    %% Run all experiments
    finished_count = 0;
    total = 0;
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
                    finished_run = exist(strcat(pb.output_folder, '/final_state.mat'), 'file'); 
                    if finished_run
                        finished_count = finished_count + 1;
                    end
                    total = total + 1;
                    fprintf("Finished %d/%d experiments\n", finished_count, total);     
                end 
            end 
        end 
    end 
end 