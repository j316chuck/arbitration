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
    gnums_2d = [40; 40];
    grid_2d = createGrid(gmin_2d, gmax_2d, gnums_2d);
    % create an array of points at which to interpolate old image
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, [40, 40]);
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +1 for free -1 for for occupied
    occ_map = 1 - 2 * occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, occ_map);
    heatmap(signed_obs_map);
    contour(occ_map);
    
    %% Calculate the BRS Control
    % Define grid
    grid_min = [gmin_2d; -pi]; % Lower corner of computation domain
    grid_max = [gmax_2d; pi];    % Upper corner of computation domain
    N = [gnums_2d; 40];         % Number of grid points per dimension
    pdDims = 3;               % 3rd dimension is periodic
    grid_3d = createGrid(grid_min, grid_max, N, pdDims);
    data0 = repmat(signed_obs_map, 1, 1, grid_3d.N(3));
    t0 = 0;
    tMax = 2;
    dt = 0.05;
    tau = t0:dt:tMax;
    % Define dynamic system
    speed = 1;
    wRange = [-1 1];
    dMax = [0.1, 0.1, 0];
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

    %% Calculate FMM Planner
    
    %% Calculate Spline PlanneR
    
    
    %% Blending
    
        
       
end 

function [] = FMM_planner(xinit, xgoal, cost_map)
    
end 

function [u, in_brs] = get_optimal_trajectory(xinit, data, tau, schemeData)
    %check if this initial state is in the BRS/BRT
    %value = eval_u(g, data, x)
    value = eval_u(g, data(:,:,:,end), xinit);
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
    [traj, ~] = computeOptTraj(g, dataTraj, tau, schemeData.dynSys, TrajextraArgs);
    u = traj(:, 1);
end
 

