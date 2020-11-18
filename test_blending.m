function test_blending()
    % plot the grid and the obstacle map;     
    env = load_env();
    % Goal
    goal_radius = 0.2;
    goal = [3, 2.75, pi/2, 0.01];
    sd_goal_map = shapeCylinder(env.grid_2d, 3, goal(1:2), goal_radius); % 2D function (x,y)
    num_waypts = 50;
    horizon = 4;

    % Spline Planner
    spline_planner = SplinePlanner(num_waypts, horizon, env.grid_2d, env.splineDynSys); 
    spline_planner.set_sd_goal(goal, sd_goal_map);
    spline_planner.set_sd_obs(-env.masked_obs_map);
    start = [1, 1, 0, 0.01]; 
    opt_spline = spline_planner.plan(start);

    %% Plane parameters
    env.reachAvoidDynsys.x = start(1:3);
    tau = 0:0.5:500;
    reach_avoid_planner = ReachAvoidPlanner(env.grid_3d, env.reachAvoidSchemeData, tau);

    %% Plan!
    % note: need a non-zero starting velocity to avoid singularities in spline
    sd_goal_3d = shapeCylinder(env.grid_3d, 3, goal(1:3), goal_radius); 
    obstacle = -repmat(env.obs_map, 1, 1, env.grid_3d.N(3));
    optTrajDt = spline_planner.dt;
    reach_avoid_planner.solve_reach_avoid(start(1:3), goal(1:3), sd_goal_3d, obstacle, optTrajDt);   

    figure(5);
    clf; 
    hold on 
    [M, c] = contourf(env.grid_2d.xs{1}, env.grid_2d.xs{2}, -sd_goal_3d(:, :, 1), [0 0], 'DisplayName', 'goal shape', 'color', 'red');
    %caxis([min(M(:)),max(M(:))]);
    %colormap([[1, 0, 0]]);
    [M, c] = contourf(env.grid_2d.xs{1}, env.grid_2d.xs{2}, env.obs_map, [0 0], 'DisplayName', 'obstacle');
    %caxis([min(M(:)),max(M(:))]);
    %colormap([[1, 1, 1]]);
    
    reach_avoid_xs = reach_avoid_planner.opt_traj(1, :);
    reach_avoid_ys = reach_avoid_planner.opt_traj(2, :);
    reach_avoid_ths = reach_avoid_planner.opt_traj(3, :);
    plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');
    
    opt_xs = opt_spline{1};
    opt_ys = opt_spline{2};
    opt_ths = opt_spline{3};
    plot_traj(opt_xs, opt_ys, opt_ths, 'cyan', 'spline');
    scatter(goal(1), goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
    scatter(start(1), start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
  
    xlim([env.grid_2d.min(1),env.grid_2d.max(1)]);
    ylim([env.grid_2d.min(2),env.grid_2d.max(2)]);
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    legend('Location', 'SouthWest');
    title('Blending Controls');

end 

function plot_traj(xs, ys, ths, color, name)
    s = scatter(xs, ys, color, 'filled', 'DisplayName', name);
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.HandleVisibility = 'off';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.3;
end


