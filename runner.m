function runner()
    %% Set up experiment parameters
    N = 100; 
    no_rerun = true; 
    run_smoke_test = false; 
    run_planners = true; 
    blend_schemes = get_key_blend_schemes();    
    control_schemes = get_all_control_schemes();   
    
    %% Load up test cases
    repo = what("arbitration"); 
    if run_smoke_test
        [starts, goals] = get_all_smoke_test_cases(); 
    else 
        path = strcat(repo.path, '/data/sampled_goals.mat'); 
        load(path, 'starts', 'goals'); 
        starts = num2cell(starts', 1); 
        goals = num2cell(goals', 1); 
    end
    
    %% Run Reach Avoid and BRS Planners
    if run_planners
        run_and_cache_planners(N, starts, goals); 
    end
    
    %% Run all experiments
    exp_names = {}; 
    failed_cases = {}; 
    results = []; 
    elapsed_time = [];
    exp_name = ""; 
    termination_state = -1; 
    for i = 1:length(starts)
        if i > N; break; end
        for k = 1:length(control_schemes)
            for j = 1:length(blend_schemes)
                tic;  
                try 
                    params = default_hyperparams(); 
                    params.start = starts{i};
                    params.goal = goals{i};
                    params.blend_scheme = blend_schemes{j}; 
                    params.control_scheme = control_schemes{k}; 
                    params.hyperparam_str = get_hyperparam_string(params); 
                    params.run_planner = false;
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