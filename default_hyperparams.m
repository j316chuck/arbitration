function [params] = default_hyperparams()
    % ============ Grid ============== % 
    params.map_basename = 'bookstore';
    repo = what('arbitration');
    params.map_yaml = strcat(repo.path, '/maps/bookstore.yaml');
    params.map_name = strcat(repo.path, '/maps/bookstore.png');
    params.gmin_3d = [-10; -10; -pi];
    params.gmax_3d = [5.4; 5.4; pi];
    params.gnum_3d = [41; 41; 16];
    params.goal_radius = 0.5;
    
    % ============ DynSys ============== %
    params.wMax = 1;
    params.vRange = [0.1, 1.0];
    params.dMax = [0.1, 0.1];
    params.tau = 0:0.5:10;
    
    % ============ Spline Planner ============== %
    params.num_waypts = 50;
    params.horizon = 5;
    
    % ============ Start and End ============== %
    params.start = [1.5; 1.5; pi/2; 0.01]; %[2; 1; pi/2; 0.01]; %[4; 0; pi/2; 0.01]; %[-9;-9;0;0.01]; 
    
    params.goal = [4; 3; pi/2; 0.01]; 
    % ============ Blend Scheme ============== %
    %params.blend_scheme = 'time_vary_alpha_open_loop_safety_control'; 
    params.blend_scheme = 'time_vary_alpha_closed_loop_safety_control'; 
    %params.blend_scheme = 'safety_value'; 
    %params.blend_scheme = 'safety_control'; 
    %params.blend_scheme = 'sample_safety_value'; 
    %params.blend_scheme = 'sample_safety_control'; 
    %params.blend_scheme = 'replan_waypoint';  
    %params.blend_scheme = 'none';
    %params.blend_scheme = 'replan_safe_traj';

    % ============ Control  Scheme ============== %
    params.control_scheme = 'follow'; 
    %params.control_scheme = 'switch'; 
    %params.control_scheme = 'constant'; 
    %params.control_scheme = 'distance';  
    
    % ============ Blending Params ============== %
    params.replan_dt = 1.5;
    params.zero_level_set = 0.2;
    params.replan_level_set = 0.3; 
    params.replan_spline_max_num_candidates = 10; 
    params.alpha = 0.8;
    params.temperature = 0.2;
    params.blend_function_name = 'reg_sig'; %'sub' %'identity'
    params.blend_function = @(v) 1 / (1 + exp(v/params.temperature));
    params.num_alpha_samples = 10; 
    params.spline_obs_weight = 1;  
    params.blend_function = @(v) max(min(1, v), 0);
    params.hyperparam_str = get_hyperparam_string(params); 

    %% Outputs
    % ============ Output and Caching ============== %
    params.clear_dir = false; 
    params.run_planner = true; 
    params.run_brs = true;
    params.save_plot = true;
    % level 1: every timestamp, level 2: every mpc, level 3: at the end
    params.plot_level = 2;
end 
