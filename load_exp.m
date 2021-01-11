function [exp] = load_exp(params)
    %% Read in Map Data
    exp.map_basename = params.map_basename; 
    exp.map_yaml = params.map_yaml;
    exp.map_name = params.map_name; 
    map_data = yaml.ReadYaml(exp.map_yaml);
    obs_data_2d = imread(exp.map_name); 
    obs_data_2d = double(obs_data_2d) / 255;
    exp.map_data = map_data;
    exp.obs_data = obs_data_2d;

    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [map_data.origin{1}, map_data.origin{2}];
    gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img);
    exp.grid_img = grid_img;
   
    % Define 3D grid
    gmin_3d = params.gmin_3d;  % Lower corner of computation domain
    gmax_3d = params.gmax_3d;  % Upper corner of computation domain
    gnum_3d = params.gnum_3d;  % Number of grid points per dimension
    pdDim = 3;                 % 3rd dimension is periodic
    grid_3d = createGrid(gmin_3d, gmax_3d, gnum_3d, pdDim);
    exp.grid_3d = grid_3d; 
    
    % create grid representing the new image size
    gmin_2d = params.gmin_3d(1:2);
    gmax_2d = params.gmax_3d(1:2);
    gnum_2d = params.gnum_3d(1:2);
    grid_2d = createGrid(gmin_2d, gmax_2d, gnum_2d);
    exp.grid_2d = grid_2d;
   
    % create an array of points at which to interpolate old image
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, gnum_2d');
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +1 for free 0 for occupied
    exp.occ_map = occ_map; 
    % +1 for free -1 for occupied 
    binary_occ_map = 2 * occ_map - 1;
    exp.binary_occ_map = binary_occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, binary_occ_map);
    % positive inside obstacle, zero within free space
    masked_obs_map = -signed_obs_map .* (1-occ_map);
    % fmm map, negative inside obstacle, positive inside free space
    exp.obs_map = signed_obs_map;
    exp.masked_obs_map = masked_obs_map;
    % debug obstacle map
    figure(10);
    hold on;
    contour(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.occ_map, [0.5, 0.5]); 
    contour(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.binary_occ_map); 
    contourf(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.masked_obs_map, [0.1 0.1]);
    colorbar; 
    
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
    reachAvoidSchemeData.grid = grid_3d;
    reachAvoidSchemeData.uMode = 'min';
    reachAvoidSchemeData.dMode = 'max';
    long_tau = 0:0.5:30;
    exp.reachAvoidSchemeData = reachAvoidSchemeData;
    exp.obstacle = repmat(exp.obs_map, 1, 1, exp.grid_3d.N(3));
    exp.reach_avoid_planner = ReachAvoidPlanner(exp.grid_3d, exp.reachAvoidSchemeData, long_tau); 

    %% Avoid BRS
    xstart = exp.start(1:3);
    wMax = exp.wMax; 
    vRange = exp.vRange; 
    dMax = exp.dMax; 
    avoidBrsDynSys = Plane(xstart, wMax, vRange, dMax); 
    exp.avoidBrsDynSys = avoidBrsDynSys; 
    avoidBrsSchemeData.dynSys = exp.avoidBrsDynSys;
    avoidBrsSchemeData.grid = grid_3d;
    avoidBrsSchemeData.uMode = 'max';
    avoidBrsSchemeData.dMode = 'min';
    exp.avoidBrsSchemeData = avoidBrsSchemeData; 
    exp.brs_planner = BRSAvoidPlanner(exp.grid_3d, exp.avoidBrsSchemeData, long_tau); 
         
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
    exp.blending.scheme = params.blending_scheme;
    exp.blending.replan_dt = params.replan_dt; 
    exp.blending.zero_level_set = params.zero_level_set;
    exp.blending.alpha = params.alpha;
    exp.blending.temperature = params.temperature;
    exp.blending.num_alpha_samples = params.num_alpha_samples;
    exp.blending.blend_function_name = params.blend_function_name;
    exp.blending.blend_function = params.blend_function; 
    
    %% Experiment Params
    exp.hyperparam_str = params.hyperparam_str;
    exp.clear_dir = params.clear_dir; 
    exp.run_planner = params.run_planner;     
    exp.run_brs = params.run_brs; 
    exp.save_planner = params.save_planner; 
    exp.load_planner = params.load_planner;
    exp.save_blender = params.save_blender; 
    exp.save_plot = params.save_plot; 
    exp.plot_level = params.plot_level; 
end 

