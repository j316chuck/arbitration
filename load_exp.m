function [exp] = load_exp(params)
    %% Create map data
    exp = create_map_data(params); 
    
    %% Navigation Task
    exp.start = params.start;
    exp.goal = params.goal;
    exp.goal_radius = params.goal_radius;
    exp.stop_goal_dx = 0.5;
    exp.max_num_planning_pts = 1000;
    exp.stop_on_collision = true;
    exp.goal_map_2d = shapeCylinder(exp.grid_2d, 3, exp.goal(1:2), exp.goal_radius);
    exp.goal_map_3d = shapeCylinder(exp.grid_3d, 3, exp.goal(1:3), exp.goal_radius); 
  
    %% Planners 
    % Dynsys info
    exp.wMax = params.wMax;
    exp.vRange = params.vRange; %[0, 0.65]
    exp.dMax = params.dMax;
    exp.tau = params.tau; 
    exp.num_waypts = params.num_waypts;
    exp.horizon = params.horizon;
    exp.dt = exp.horizon / (exp.num_waypts - 1);
    
    %% Reach avoid 
    xstart = exp.start(1:3);
    wMax = exp.wMax;
    vRange = exp.vRange;
    dMax = exp.dMax;
    reachAvoidDynSys = Plane(xstart, wMax, vRange, dMax); 
    reachAvoidDynSys.x = exp.start(1:3); 
    exp.reachAvoidDynSys = reachAvoidDynSys; 
    reachAvoidSchemeData.dynSys = exp.reachAvoidDynSys;
    reachAvoidSchemeData.grid = exp.grid_3d;
    reachAvoidSchemeData.uMode = 'min';
    reachAvoidSchemeData.dMode = 'max';
    long_tau = 0:0.5:30;
    exp.reachAvoidSchemeData = reachAvoidSchemeData;
    exp.reach_avoid_planner = ReachAvoidPlanner(exp.grid_3d, exp.reachAvoidSchemeData, long_tau); 

    %% Avoid BRS
    xstart = exp.start(1:3);
    wMax = exp.wMax; 
    vRange = exp.vRange; 
    dMax = exp.dMax; 
    avoidBrsDynSys = Plane(xstart, wMax, vRange, dMax); 
    exp.avoidBrsDynSys = avoidBrsDynSys; 
    avoidBrsSchemeData.dynSys = exp.avoidBrsDynSys;
    avoidBrsSchemeData.grid = exp.grid_3d;
    avoidBrsSchemeData.uMode = 'max';
    avoidBrsSchemeData.dMode = 'min';
    exp.avoidBrsSchemeData = avoidBrsSchemeData; 
    exp.brs_planner = BRSAvoidPlanner(exp.grid_3d, exp.avoidBrsSchemeData, long_tau, exp.dt); 
         
    %% Spline Planner
    xstart = exp.start(1:3);
    wMax = exp.wMax; 
    vRange = exp.vRange; 
    dMax = exp.dMax; 
    splineDynSys = Plane(xstart, wMax, vRange, dMax);
    exp.splineDynSys = splineDynSys;
    exp.spline_planner = SplinePlanner(exp.num_waypts, exp.horizon, exp.grid_2d, exp.splineDynSys, exp.binary_occ_map);  
    exp.spline_planner.set_sd_goal(exp.goal, exp.goal_map_2d);
    exp.spline_planner.set_sd_obs(exp.masked_obs_map, params.spline_obs_weight); 
    
    %% Blending Scheme
    exp.blending.blend_scheme = params.blend_scheme;
    exp.blending.control_scheme = params.control_scheme; 
    exp.blending.replan_dt = params.replan_dt; 
    exp.blending.zero_level_set = params.zero_level_set;
    exp.blending.replan_level_set = params.replan_level_set;
    exp.blending.alpha = params.alpha;
    exp.blending.temperature = params.temperature;
    exp.blending.num_alpha_samples = params.num_alpha_samples;
    exp.blending.replan_spline_max_num_candidates = params.replan_spline_max_num_candidates;
    exp.blending.blend_function_name = params.blend_function_name;
    exp.blending.blend_function = params.blend_function;
    
    %% Experiment Params
    exp.hyperparam_str = params.hyperparam_str;
    exp.clear_dir = params.clear_dir; 
    exp.run_planner = params.run_planner;     
    exp.run_brs = params.run_brs; 
    exp.save_plot = params.save_plot; 
    exp.plot_level = params.plot_level; 
end 

