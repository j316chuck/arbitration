function test_blender()
    close all;
    weights = ([1:10, 20:10:100]);
    for i = 1:length(weights)
        params = default_hyperparams();
        params.run_planner = true;
        params.spline_obs_weight = 1;
        exp = load_exp(params);
        pb = Planner(exp);
        pb.blend_mpc_controls();
    end 
end 