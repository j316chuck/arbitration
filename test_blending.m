function test_blending()
    % plot the grid and the obstacle map;     
    env = load_env();
    % Goal
    goal_radius = 0.5;
    goal = [4, 4, pi/2, 0.01];
    sd_goal_map = shapeCylinder(env.grid_2d, 3, goal(1:2), goal_radius); % 2D function (x,y)
    num_waypts = 50;
    horizon = 4;

    % Spline Planner
    spline_planner = SplinePlanner(num_waypts, horizon, env.grid_2d, env.splineDynSys); 
    spline_planner.set_sd_goal(goal, sd_goal_map);
    spline_planner.set_sd_obs(-env.masked_obs_map);
    
    start = [1, 1, 0, 0.01]; 
    opt_spline = spline_planner.plan(start);
    
    % splines
    xs = opt_spline{1};
    ys = opt_spline{2};
    ths = opt_spline{3};
    u1s = opt_spline{4};
    u2s = opt_spline{5};
    robot_colors = [linspace(0.1, 0.9, length(xs))', zeros([length(xs),1]), zeros([length(xs),1])];
    figure(5);
    hold on; 
    s = scatter(xs, ys, 'r', 'filled');
    s.CData = robot_colors; %[colors', zeros(length(xs), 1), zeros(length(xs), 1)];
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', robot_colors(end,:));
    q.ShowArrowHead = 'off';
    q.AutoScale = 'off';
    q.AutoScaleFactor = 0.5;
    
    %% Plane parameters
    env.reachAvoidDynsys.x = start(1:3);
    hji_tstart = 0;
    hji_tend = 500;
    hji_dt = 0.5;
    tau = hji_tstart:hji_dt:hji_tend;
    reach_avoid_planner = ReachAvoidPlanner(env.grid_3d, env.reachAvoidSchemeData, tau);

    %% Plan!
    % note: need a non-zero starting velocity to avoid singularities in spline
    sd_goal_3d = shapeCylinder(env.grid_3d, 3, goal(1:3), goal_radius); 
    obstacle = -repmat(env.obs_map, 1, 1, env.grid_3d.N(3));
    reach_avoid_planner.solve_reach_avoid(start(1:3), goal(1:3), sd_goal_3d, obstacle);    
    reach_avoid_planner.plot_traj();
        
       
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


