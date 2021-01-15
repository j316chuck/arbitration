function smoke_test()
    close all;
    params = default_hyperparams();
    % ============ Blend Scheme ============== %
    %params.blend_scheme = 'time_varying_value_blend_safety_control_traj'; 
    %params.blend_scheme = 'safety_value'; 
    %params.blend_scheme = 'safety_control'; 
    %params.blend_scheme = 'sample_safety_value'; 
    %params.blend_scheme = 'sample_safety_control'; 
    %params.blend_scheme = 'replan_waypoint'; 
    params.blend_scheme = 'none';
    %params.blend_scheme = 'replan_safe_traj';
    
    % ============ Control  Scheme ============== %
    params.control_scheme = 'follow'; 
    %params.control_scheme = 'switch'; 
    %params.control_scheme = 'constant'; 
    %params.control_scheme = 'distance'; 
       
    % ============ More hyperparameters here ============== %
    
  
    % ============ Smoke Tests ============== %
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
    for i = 1:5
        params.start = starts{i};
        params.goal = goals{i}; 
        params.run_planner = true; % save state, only have to run once
        exp = load_exp(params); 
        pb = Planner(exp);
        pb.blend_mpc_controls(); 
    end 
end
