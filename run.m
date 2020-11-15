function run()
    %% Read in Map Data
    map_name = './maps/bookstore.png';
    map_yaml = './maps/bookstore.yaml';
    map_data = yaml.ReadYaml(map_yaml);
    obs_data_2d = imread(map_name); % alternatively: obs_data_2d = rgb2gray(obs_data_3d);
    obs_data_2d = double(obs_data_2d) / 255;

    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [map_data.origin{1}, map_data.origin{2}];
    gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img); 
    % create grid representing the new image size
    gmin_2d = [-10; -10];
    gmax_2d = [5.4; 5.4];
    gnums_2d = [20; 20];
    grid_2d = createGrid(gmin_2d, gmax_2d, gnums_2d);
    % create an array of points at which to interpolate old image
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, [20, 20]);
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +1 for free -1 for for occupied 
    occ_map = 1 - 2 * occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, occ_map);
    heatmap(signed_obs_map);
    contour(occ_map);
    % Define grid
    grid_min = [gmin_2d; -pi]; % Lower corner of computation domain
    grid_max = [gmax_2d; pi];    % Upper corner of computation domain
    N = [gnums_2d; 40];         % Number of grid points per dimension
    pdDims = 3;               % 3rd dimension is periodic
    grid_3d = createGrid(grid_min, grid_max, N, pdDims);
    % Goal
    goal_radius = 1;
    goal = [5, 5, pi/2, 0.01];
    sd_goal = -1 .* shapeCylinder(grid_2d, 3, goal(1:2), goal_radius); % 2D function (x,y)
    contour(sd_goal); 
    num_waypts = 50;
    horizon = 8;
    max_linear_vel = 2.5;
    max_angular_vel = 1;
    % Spline Planner
    planner = SplinePlanner(num_waypts, ...
                            horizon, ...
                            goal, ...
                            max_linear_vel, ...
                            max_angular_vel, ...
                            signed_obs_map, ...
                            sd_goal, ...
                            grid_2d, ...
                            grid_3d, ...
                            grid_min, ...
                            grid_max, ...
                            N); 
    start =[4, 4, 0, 0.01]; 
    opt_spline = planner.plan(start, goal);
            
    %% Plot!
    xs = opt_spline{1};
    ys = opt_spline{2};
    u1s = opt_spline{3};
    u2s = opt_spline{4};
    ths = opt_spline{5};
    

    %% Calculate the Safety Control
    data0 = repmat(signed_obs_map, 1, 1, grid_3d.N(3));
    t0 = 0;
    tMax = 2;
    dt = 0.05;
    tau = t0:dt:tMax;
    % Define dynamic system
    speed = 1.5;
    wRange = [-1 1];
    dMax = [0.2, 0.2, 0];
    uMode = 'min';
    dMode = 'max';
    dCar = DubinsCar([0, 0, 0], wRange, speed, dMax);
    schemeData.grid = grid_3d;
    schemeData.dynSys = dCar;
    schemeData.accuracy = 'high'; %set accuracy
    schemeData.uMode = uMode;
    schemeData.dMode = dMode;
    % Define extra args
    HJIextraArgs.visualize.valueSet = 1;
    HJIextraArgs.visualize.initialValueSet = 1;
    HJIextraArgs.visualize.figNum = 1; %set figure number
    HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
    [data, tau, ~] = HJIPDE_solve(data0, tau, schemeData, 'minVOverTime', HJIextraArgs);
    [u, in_brs] = get_optimal_trajectory([0, 0, 0], data, tau, schemeData);

    %% Calculate Optimal Plan
    brs = data(:, :, :, end);
    data_2d = proj(grid_2d, brs, [0 0 1], [0]);
    visSetIm(grid_2d, data_2d);
    
    %% Blending
    
        
       
end 

function [] = FMM_planner(xinit, xgoal, cost_map)
    
end 

function [u, in_brs] = get_optimal_trajectory(xinit, data, tau, schemeData)
    %check if this initial state is in the BRS/BRT
    %value = eval_u(g, data, x)
    value = eval_u(schemeData.grid, data(:,:,:,end), xinit);
    if value < 0
        in_brs = true; 
    else 
        in_brs = false; 
    end 
    schemeData.dynSys.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = schemeData.uMode; %set if control wants to min or max
    TrajextraArgs.dMode = schemeData.dMode;
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number

    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 

    %flip data time points so we start from the beginning of time
    dataTraj = flip(data, 4);
    [traj, ~] = computeOptTraj(schemeData.grid, dataTraj, tau, schemeData.dynSys, TrajextraArgs);
    u = traj(:, 1);
end
 

