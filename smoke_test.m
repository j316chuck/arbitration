function smoke_test()
    % ============ Blend Scheme ============== %
    all_blend_schemes = {'time_vary_alpha_open_loop_safety_control', ...
                        'time_vary_alpha_closed_loop_safety_control', ...
                        'safety_value', ...
                        'safety_control', ...
                        'sample_safety_value', ...
                        'sample_safety_control', ...
                        'replan_waypoint', ....
                        'none'};    
                    
    % ============ Control  Scheme ============== %
    all_control_schemes = {'follow', 'switch'};     
  
    % ============ Smoke Test Cases ============== %
    starts = {
             [-0.375; -1.915; pi; 0.01; 0], ... % open - open
             [-0.375; -1.915; pi; 0.01; 0], ... % open - tight
             [0.395; 3.86; pi/2; 0.01; 0], ... % tight - tight
             [-0.375; -1.915; pi; 0.01; 0], ... % open - obstacle
             [0.395; 3.86; -pi/2; 0.01; 0], ... % tight - tight
    };     
    goals = {
             [0.78; 1.55; pi/2; 0.01; 0], ... % open - open
             [-0.375; 2.705; pi; 0.01; 0],... % open - tight
             [0.395; 2.705; pi/2; 0.01; 0], ... % tight - tight
             [-4.61; -0.76; pi; 0.01; 0], ... % open - obstacle
             [0.395; 2.705; pi/2; 0.01; 0], ... % hard angle
    }; 

    % ============ Run smoke test ============== %
    failed_cases = {}; 
    pb.exp_name = ''; 
    for i = 1:5
        for k = 1:length(all_control_schemes)
            for j = 3:length(all_blend_schemes)
                % Uncomment to run on one single control and blend scheme
                %if j ~= 2 || k ~= 3
                %    continue; 
                %end 
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
                catch
                    failed_cases{end+1} = pb.exp_name; 
                end
            end 
        end 
    end 
    save('failed_states.mat'); 
end
