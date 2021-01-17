function smoke_test()
    all_blend_schemes = get_all_blend_schemes();    
    all_control_schemes = get_all_control_schemes();     
    [starts, goals] = get_all_smoke_test_cases(); 

    exp_names = {}; 
    failed_cases = {}; 
    results = []; 
    elapsed_time = [];
    exp_name = ""; 
    termination_state = -1; 
    for i = 1:5
        for k = 1:length(all_control_schemes)
            for j = 1:length(all_blend_schemes)
                % uncomment to run only specific examples
                % if i ~= 5 || k ~= 2 
                %    continue; 
                % end 
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
                if isempty(termination_state)
                    termination_state = -1;     
                end 
                exp_names{end+1} = exp_name; 
                results(end+1) = termination_state;
                elapsed_time(end+1) = toc; 
                save('new_smoke_test.mat', 'exp_names', 'results', 'elapsed_time', 'failed_cases'); 
            end 
        end 
    end 
end
