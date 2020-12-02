function [params] = default_hyperparams()
    % env
    params.map_basename = 'bookstore';
    params.map_yaml = 'maps/bookstore.yaml';
    params.map_name = 'maps/bookstore.png';
    params.gmin_3d = [-10; -10; -pi];
    params.gmax_3d = [5.4; 5.4; pi];
    params.gnum_3d = [41; 41; 16];
    % navigation
    params.start =  [1; 1; 0; 0.01];
    params.goal = [3, 2.75, pi/2, 0.01];
    params.goal_radius = 0.5;
    % dynsys
    params.wMax = 1;
    params.vRange = [0.1, 1.0];
    params.dMax = [0.1, 0.1];
    params.tau = 0:0.5:10;
    % spline planner
    params.num_waypts = 50;
    params.horizon = 5;
    % blending params
    params.replan_dt = 100;
    params.blending_scheme = 'distance';
    params.zero_level_set = 0.1;
    params.alpha = 0.75;
    params.temperature = 1;
    %params.hyperparam_str = sprintf("replan_dt_%.3f_zero_levelset_%.3f", params.replan_dt, params.zero_level_set); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
    params.hyperparam_str = sprintf("replan_dt_%.3f_sig_temp_%.3f", params.replan_dt, params.temperature); 

    % file path params
    params.clear_dir = false; 
    params.run_planner = true; 
    params.save_planner = true; 
    params.load_planner = true;
    params.save_blender = true; 
    params.save_plot = true; 
    params.plot_every_iter = true; 
end 