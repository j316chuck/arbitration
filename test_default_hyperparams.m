function test_default_hyperparams()
    close all;
    params = default_hyperparams();
    exp = load_exp(params);
    pb = PlannerBlender(exp); 
    pb.blend_planners(); 
end 