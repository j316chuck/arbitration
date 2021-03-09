function test_planner()
    close all;
    params = default_hyperparams();
    params.updateMethod = 'HJIPDE';
    params.environment_type = 'unknown';
    params.hyperparam_str = get_hyperparam_string(params); 
    params.run_planner = false; 
    params.run_brs = true;
    exp = load_exp(params);
    pb = Planner(exp);
    pb.plot_level = 2; 
    pb.blend_plans(); 
    fprintf("Termination: %f\n", pb.termination_state); 
end 