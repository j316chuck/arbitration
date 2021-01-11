function params = spline_params() 
    %% Grid representation.
    params.gmin = [-7.75, -7.75]; % should take into accound theta too?
    params.gmax = [7.75, 7.75];
    params.gnums = [15,15];
    params.g2d = createGrid(params.gmin, params.gmax, params.gnums);

    % 3D grid including orientation
    pdDim = 3;
    ntheta = 15;
    offset_pi = 0.01;
    params.g3d = createGrid([params.gmin,-pi+offset_pi], [params.gmax,pi], [params.gnums,ntheta], pdDim);

    %% Trajectory Info
    params.num_waypts = 50; % Note: should match the number of steps predicted. 
    params.horizon = 8;
    params.dt = params.horizon/(params.num_waypts-1);
    params.goal = [1.83, 6.5, pi/2, 0.01]; 

    % 13 meters / second ~= 30 mph
    % 8 meters / second ~= 18 mph (is the average speed at intersection) 
    dv = 1;
    params.max_linear_vel = 6.; 
    params.max_angular_vel = 1.; 
    xstart = [100; 75; 220*pi/180];
    wMax = 1;
    vrange = [1.1,6];
    dMax = [0.3; 0.3];
    params.dynSys = Plane(xstart, wMax, vrange, dMax);

    %% Signed dist functions.
    % Axis-aligned rectangular obstacle convention is:
    %       [lower_x, lower_y, width, height]
    params.obstacles = {[-7.75, -7.75, 4.1, 4.1]...
                [3.65, -7.75, 4.1, 4.1], ...
                [-7.75, 3.65, 4.1, 4.1], ...
                [3.65, 3.65, 4.1, 4.1]};
    params.raw_sd_obs = nan(params.g2d.shape);

    %% Create obstacle signed distance (zero outside obstacle, negative inside).
    params.obs_padding = ones(1,2)*0.25;
    for i=1:length(params.obstacles)
        info = params.obstacles{i};
        lower = info(1:2) - params.obs_padding;
        upper = info(1:2) + info(3:4) + params.obs_padding;

        sd = shapeRectangleByCorners(params.g2d, lower, upper);

        % convert the signed distance into a "bump distance"
        % where phi(x,u) = 0 if outside obstacle
        % and   phi(x,u) = signedDist inside obstacle.
        obs_mask = (params.g2d.xs{1} >= lower(1)) & ...
            (params.g2d.xs{2} >= lower(2)) & ...
            (params.g2d.xs{1} <= upper(1)) & ...
            (params.g2d.xs{2} <= upper(2)); 

        sd = sd .* obs_mask;

        params.raw_sd_obs = min(params.raw_sd_obs, sd);
    end

    %% ADD IN PENALTY IF YOU GO INTO OTHER LANE!
    lower = [3.65,-7.75] - params.obs_padding;
    upper = [7.75,7.75] + params.obs_padding;
    params.other_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
    params.sd_obs = min(params.raw_sd_obs, params.other_lane_sd);

    %% ADD ANOTHER PENALTY IF YOU GO INTO OPPOSING LANE
    lower = [-3.65, 3.65] - params.obs_padding;
    upper = [0, 7.75] + params.obs_padding;
    params.final_lane_sd = shapeRectangleByCorners(params.g2d, lower, upper);
    params.sd_obs = min(params.sd_obs, params.final_lane_sd);
    params.sd_obs = -params.sd_obs;
    %% Create goal signed distance (positive in goal, negative outside).
    params.goal_radius = 1;
    params.sd_goal = shapeCylinder(params.g2d, 3, params.goal(1:2), params.goal_radius); % 2D function (x,y)

    %% Create spline planner!
    fprintf('Setting up SINGLE SPLINE DRIVING PLANNER...\n');
    %% Create spline planner!
    params.planner = SplinePlanner(params.num_waypts, ...
                                params.horizon, ...
                                params.g2d, ...
                                params.dynSys, ...
                                params.raw_sd_obs);   
                         
                            
end 