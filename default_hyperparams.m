function [params] = default_hyperparams()
    % env
    params.map_basename = 'bookstore';
    params.map_yaml = 'maps/bookstore.yaml';
    params.map_name = 'maps/bookstore.png';
    params.gmin_3d = [-10; -10; -pi];
    params.gmax_3d = [5.4; 5.4; pi];
    params.gnum_3d = [41; 41; 16];
    % navigation
    params.start =  [-4.01; -4; pi/2; 0.01]; %[-7.36; -3.87; -0.02; 0.01]; %[1.49; 4.40; 1.95; 0.01]; %[-0.41; 3.86; 0.91; 0.01]; %[1.52; 1.23; -1.31; 0.01]; %[4.56; 3.95; -pi/10; 0.01]; %[-4; -1; pi/2; 0.01]; %[-9; -9.5; 0; 0.01]; %[0; -9; 0; 0.01];
    params.goal = [-8; -5; 0; 0.01]; %[3.71; -6.28; -2; 0.01]; %[-0.57; -9.62; -1.83; 0.01]; %[0.89; -6.80; -0.77; 0.01]; %[-9.04; -7.02; -0.12; 0.01]; %[4.76; 0.84; 1.26; 0.01]; %[3, 2.75, pi/2, 0.01]; %[2, 4, pi/2, 0.01];
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
    params.blending_scheme =  'switch'; %'probabilistic_blend_safety_control_traj'; %'switch'; 
    params.replan_dt = 1.5;
    params.zero_level_set = 0.2;
    params.alpha = 0.2;
    params.temperature = 0.1;
    params.blend_function_name = 'reg_sig'; %'sub'
    params.blend_function = @(v) 1 / (1 + exp(v/params.temperature));
    params.num_alpha_samples = 10; 
    %params.blend_function = @(v) max(min(1, 1-(v/params.temperature)), 0);
    params.spline_obs_weight = 1; 
    params.hyperparam_str = sprintf("replan_dt_%.3f_zero_level_set_%.3f_spline_obs_weight_%f", params.replan_dt, params.zero_level_set, params.spline_obs_weight); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_%s_temp_%.3f", params.replan_dt, params.blend_function_name, params.temperature); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_level_set_%.2f_spline_obs_weight_%f", params.replan_dt, params.num_alpha_samples, params.zero_level_set, params.spline_obs_weight); 

    % file path params
    params.clear_dir = false; 
    params.run_planner = true; 
    params.run_brs = false;
    params.save_planner = true; 
    params.load_planner = true;
    params.save_blender = true; 
    params.save_plot = true; 
    params.plot_level = 2; %every iter, every replan, at the end, not at all
end 
