function test_blender()
    try 
        close all;
        params = default_hyperparams();
        params.run_planner = false;
        exp = load_exp(params);
        pb = Planner(exp);
        %pb.blend_mpc_controls();
        %pb.blend_mpc_traj(); 
        pb.blend_and_replan_mpc_controls(); 
        pb.termination_state
    catch 
        save('./debug.mat');
    end 
end 