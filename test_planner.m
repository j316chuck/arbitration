function test_planner()
    close all;
    params = default_hyperparams();
    params.run_planner = true; 
    params.run_brs = false;
    exp = load_exp(params);
    pb = Planner(exp);
    pb.plot_level = 2; 
    pb.blend_plans(); 
    fprintf("Termination: %f\n", pb.termination_state); 
end 