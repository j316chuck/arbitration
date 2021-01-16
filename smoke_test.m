function smoke_test()
    % ============ Blend Scheme ============== %
    all_blend_schemes = get_all_blend_schemes();    
                    
    % ============ Control  Scheme ============== %
    all_control_schemes = get_all_control_schemes();     
  
    % ============ Smoke Test Cases ============== %
    [starts, goals] = get_all_smoke_test_cases(); 

    % ============ Run smoke test ============== %
    exp_names = {}; 
    failed_cases = {}; 
    results = []; 
    elapsed_time = []; 
    exp_name = ''; 
    termination_state = -1; 
    for i = 1:5
        for k = 1:length(all_control_schemes)
            for j = 1:length(all_blend_schemes)
                tic; 
                try 
                    params = default_hyperparams(); 
                    params.start = starts{i};
                    params.goal = goals{i};
                    params.blend_scheme = all_blend_schemes{j}; 
                    params.control_scheme = all_control_schemes{k}; 
                    params.hyperparam_str = get_hyperparam_string(params); 
                    params.run_planner = false; % save state, only have to run once
                    exp = load_exp(params); 
                    pb = Planner(exp);
                    pb.blend_plans(); 
                    exp_name = pb.exp_name; 
                    termination_state = pb.termination_state; 
                catch
                    exp_name = pb.exp_name; 
                    termination_state = 3; 
                    failed_cases{end+1} = pb.exp_name; 
                end
                exp_names{end+1} = exp_name; 
                if isempty(termination_state)
                    termination_state = 3; 
                end
                results(end+1) = termination_state;
                elapsed_time(end+1) = toc; 
                save('smoke_test.mat', 'exp_names', 'results', 'elapsed_time', 'failed_cases'); 
            end 
        end 
    end 
end
