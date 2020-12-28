function test_blender()
    close all;
    params = default_hyperparams();
    exp = load_exp(params);
    pb = Planner(exp);
    %pb.blend_mpc_controls();
    pb.blend_mpc_traj(); 
    pb.termination_state
end 