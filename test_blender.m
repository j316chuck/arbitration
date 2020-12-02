function test_blender()
    close all;
    params = default_hyperparams();
    exp = load_exp(params);
    pb = PlannerBlender(exp); 
    pb.blend_planners(); 
end 