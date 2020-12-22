function test_blender()
    close all;
    params = default_hyperparams();
    exp = load_exp(params);
    pb = Planner(exp); 
    pb.blend_mpc_planning(); 
end 