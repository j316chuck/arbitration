function test_blender()
    close all;
    weights = flip([1:10, 20:10:100]);
    for i = 1:length(weights)
        params = default_hyperparams();
        params.spline_obs_weight = weights(i);
        exp = load_exp(params);
        pb = Planner(exp);
        pb.blend_mpc_controls();
    end 
end 