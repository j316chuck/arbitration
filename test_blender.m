function test_blender()
    close all;
    params = default_hyperparams();
    exp = load_exp(params);
    pb = Planner(exp);
    pb.blend_plans(); 
    fprintf("Termination: %f\n", pb.termination_state); 
end 