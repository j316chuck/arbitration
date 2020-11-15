function test_blending()
    % plot the grid and the obstacle map;     
    env = load_env();
    % Goal
    goal_radius = 1;
    goal = [4, 4, pi/2, 0.01];
    spline_goal = -1 .* shapeCylinder(grid_2d, 3, goal(1:2), goal_radius); % 2D function (x,y)
    num_waypts = 50;
    horizon = 4;
    max_linear_vel = env.reachAvoidDynSys.vrange(2);
    max_angular_vel = env.reachAvoidDynSys.wMax;
    % Spline Planner
    spline_planner = SplinePlanner(num_waypts, ...
                                    horizon, ...
                                    goal, ...
                                    max_linear_vel, ...
                                    max_angular_vel, ...
                                    masked_obs_map, ...
                                    spline_goal, ...
                                    env.grid_2d, ...
                                    env.grid_3d, ...
                                    env.grid_2d.min', ...
                                    env.grid_2d.max', ...
                                    env.grid_2d.N'); 
    start = [1, 1, 0, 0.01]; 
    opt_spline = spline_planner.plan(start, goal);
    xs = opt_spline{1};
    ys = opt_spline{2};
    u1s = opt_spline{3};
    u2s = opt_spline{4};
    ths = opt_spline{5};  
    robot_colors = [linspace(0.1, 0.9, length(xs))', zeros([length(xs),1]), zeros([length(xs),1])];
    figure(5);
    hold on; 
    s = scatter(xs, ys, 'r', 'filled');
    s.CData = robot_colors; %[colors', zeros(length(xs), 1), zeros(length(xs), 1)];
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', robot_colors(end,:));
    q.ShowArrowHead = 'off';
    q.AutoScale = 'off';
    q.AutoScaleFactor = 0.5;
    
    %% Calculate the Safety Control
    data0 = repmat(signed_obs_map, 1, 1, grid_3d.N(3));
    t0 = 0;
    tMax = 2;
    dt = 0.05;
    tau = t0:dt:tMax;
    % Define dynamic system
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

function plot_traj(xs, ys, ths, colors)
    %     for i=1:length(xs)
    %         %plot_car(xs(i), ys(i), ths(i), car_len, car_width, colors(i));
    %         pos = [xs(i)-car_rad, ys(i)-car_rad, ...
    %                car_rad*2, car_rad*2];
    %         rectangle('position', pos, ...
    %                     'curvature', [1,1], ...
    %                     'EdgeColor', colors(i, :));
    %     end
    s = scatter(xs, ys, 'r', 'filled');
    s.CData = colors; %[colors', zeros(length(xs), 1), zeros(length(xs), 1)];
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', colors(end,:));
    q.ShowArrowHead = 'off';
    q.AutoScale = 'off';
    q.AutoScaleFactor = 0.5;
end


