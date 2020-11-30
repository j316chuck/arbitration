function test_blending()
    %run_planners();
    %blend_planners();
    for a = [0.75] %[0.75, 0.9, 0.95, 1]
        alpha_blend_planners(a);
    end
    plot_planners();
end

function blend_planners() 
    load('./data/planners.mat'); 
    horizon = 5;
    num_waypts = 50;
    dt = horizon / (num_waypts - 1);
    splineDynSys = env.splineDynSys;
    splineDynSys.x = [1, 1, 0]';
    state = [splineDynSys.x', 0.01, 0];
    traj = zeros(6, 0); 
    zero_levelset_threshold = 0;
    mpc = false;
    use_avoid_control = 1;
    for i = 1:num_waypts
        %updated_num_waypts = num_waypts - (i - 1); 
        %updated_horizon = horizon - (dt * (i - 1)); 
        if brs_planner.get_value(state(1:3)) <= zero_levelset_threshold
            u = brs_planner.get_avoid_u(state(1:3))';
            use_avoid_control = 1;
        else 
            if mpc || use_avoid_control == 1
               spline_planner.set_spline_planning_points(num_waypts, horizon);
               spline_planner.plan(state(1:4));
            end 
            u = spline_planner.get_next_control();
            use_avoid_control = 0; 
        end
        [~, vf_slice] = proj(env.grid_3d, brs_planner.valueFun, [0 0 1], state(3));
        traj(:, end+1) = [splineDynSys.x', u, use_avoid_control]'; % old state and new control
        splineDynSys.updateState(u, dt, splineDynSys.x); 
        opt_cur_spline = spline_planner.opt_spline;
        blend_name = 'safe blending';
        save('./data/blenders.mat', 'traj', 'blend_name', 'opt_cur_spline', 'vf_slice', 'state'); 
        state = [splineDynSys.x', u]; % new state and new control
        plot_planners(); 
    end 
end


function alpha_blend_planners(alpha) 
    load('./data/planners.mat'); 
    horizon = 5;
    num_waypts = 50;
    dt = horizon / (num_waypts - 1);
    splineDynSys = env.splineDynSys;
    splineDynSys.x = [1, 1, 0]';
    state = [splineDynSys.x', 0.01, 0];
    traj = zeros(6, 0); 
    zero_levelset_threshold = 0;
    mpc = false;
    use_avoid_control = 1;
    alpha_value = alpha;
    replan_index = 5;
    spline_planner.plan(state(1:4));
    for i = 1:num_waypts
        updated_num_waypts = num_waypts - (i - 1); 
        updated_horizon = horizon - (dt * (i - 1)); 
        u1 = spline_planner.get_next_control();
        u2 = brs_planner.get_avoid_u(state(1:3))';
        u = alpha * u1 + (1 - alpha) * u2;
        [~, vf_slice] = proj(env.grid_3d, brs_planner.valueFun, [0 0 1], state(3));
        traj(:, end+1) = [splineDynSys.x', u, use_avoid_control]'; % old state and new control
        splineDynSys.updateState(u, dt, splineDynSys.x); 
        opt_cur_spline = spline_planner.opt_spline;
        blend_name = 'safe blending';
        save('./data/blenders.mat', 'traj', 'blend_name', 'opt_cur_spline', 'vf_slice', 'state', 'alpha_value', 'replan_index'); 
        state = [splineDynSys.x', u]; % new state and new control
        plot_planners(); 
        replan = mod(i, replan_index) == 0;
        if replan
           spline_planner.set_spline_planning_points(updated_num_waypts, updated_horizon);
           spline_planner.plan(state(1:4));
        end 
    end 
end


function plot_planners()
    load('./data/planners.mat');
    load('./data/blenders.mat');
    figure(5);
    clf;
    hold on     
    % plot environment, goal, and start
    contour(env.grid_2d.xs{1}, env.grid_2d.xs{2}, -sd_goal_3d(:, :, 1), [0 0], 'DisplayName', 'goal shape', 'color', 'red');
    contourf(env.grid_2d.xs{1}, env.grid_2d.xs{2}, env.obs_map, [0 0], 'DisplayName', 'obstacle');
    scatter(goal(1), goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
    scatter(start(1), start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
    % plot value functions
    %contour(env.grid_2d.xs{1}, env.grid_2d.xs{2}, brs_planner.valueFun(:, :, 9), [0 0], 'DisplayName', 'BRS (theta = 0)', 'color', 'magenta');
    %contour(env.grid_2d.xs{1}, env.grid_2d.xs{2}, brs_planner.valueFun(:, :, 12), [0 0], 'DisplayName', 'BRS (theta = pi/2)', 'color', '#FFC0CB');
    name = sprintf("BRS (theta = %s)", state(3));
    %contour(env.grid_2d.xs{1}, env.grid_2d.xs{2}, vf_slice, [0 0], 'DisplayName', name, 'color', '#CC1FCB');
    contour(env.grid_2d.xs{1}, env.grid_2d.xs{2}, vf_slice, 'DisplayName', name, 'color', '#CC1FCB');
    
    % plot mpc spline traj
    mpc_spline_xs = opt_cur_spline{1}; 
    mpc_spline_ys = opt_cur_spline{2}; 
    mpc_spline_ths = opt_cur_spline{3}; 
    plot_traj(mpc_spline_xs, mpc_spline_ys, mpc_spline_ths, 'red', 'mpc spline');    
    % plot original spline traj
    orig_spline_xs = opt_spline{1};
    orig_spline_ys = opt_spline{2};
    orig_spline_ths = opt_spline{3};
    plot_traj(orig_spline_xs, orig_spline_ys, orig_spline_ths, 'cyan', 'orig spline');
    % plot blending traj
    blend_xs = traj(1, :); 
    blend_ys = traj(2, :); 
    blend_ths = traj(3, :); 
    use_avoid_probs = traj(6, :);
    plot_traj_probs(blend_xs, blend_ys, blend_ths, use_avoid_probs, blend_name);
    % plot reach avoid traj
    reach_avoid_xs = reach_avoid_planner.opt_traj(1, :);
    reach_avoid_ys = reach_avoid_planner.opt_traj(2, :);
    reach_avoid_ths = reach_avoid_planner.opt_traj(3, :);
    plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');
    
    obj = objectives();
    obj.calc_kinematics(blend_xs, blend_ys, spline_planner.dt); 
    obj.calc_dist_to_goal(blend_xs, blend_ys, goal); 
    obj.calc_dist_to_opt_traj(blend_xs, blend_ys, reach_avoid_xs, reach_avoid_ys)
    obj.calc_safety_score(blend_xs, blend_ys, blend_ths, brs_planner); 
    
    % figure parameters
    xlim([0, 5]); %xlim([env.grid_2d.min(1),env.grid_2d.max(1)]);
    ylim([0, 5]); %ylim([env.grid_2d.min(2),env.grid_2d.max(2)]);
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    legend('Location', 'NorthWest');
    title("Blending Controls");
    
    %title(sprintf("Alpha %.2f Replan Index %.2f Blending Controls", alpha_value, replan_index));
    %savefig(sprintf("./outputs/replan_index_%d_naive_blending_%.2f.fig", replan_index, alpha_value)); 
end 


function set_quiver_colors(q, probs)
    %// Get the current colormap
    currentColormap = colormap(gca);

    %// Now determine the color to make each arrow using a colormap
    [~, ~, ind] = histcounts(probs, size(currentColormap, 1));

    %// Now map this to a colormap to get RGB
    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
    cmap(:,:,4) = 255;
    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);

    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
    set(q.Head, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'

    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
    set(q.Tail, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:2,:,:), [], 4).');
end 

function plot_traj(xs, ys, ths, color, name)
    s = scatter(xs, ys, 15, 'black', 'filled', 'DisplayName', name);
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.3;
end

function plot_traj_probs(xs, ys, ths, probs, name)
%     rgb_indexes = int32(probs * 255) + 1; 
%     currentColormap = colormap(gca);
%     M = currentColormap(rgb_indexes, :); 
    s = scatter(xs, ys, 15, 'black', 'filled');
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths));
    set_quiver_colors(q, probs); 
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.DisplayName = name;
    q.AutoScaleFactor = 0.3;
end

function run_planners() 
    % plot the grid and the obstacle map;     
    env = load_env();
    % Goal
    goal_radius = 0.5;
    goal = [3, 2.75, pi/2, 0.01];
    sd_goal_map = shapeCylinder(env.grid_2d, 3, goal(1:2), goal_radius); % 2D function (x,y)
    horizon = 5;
    num_waypts = 50;

    % Spline Planner
    spline_planner = SplinePlanner(num_waypts, horizon, env.grid_2d, env.splineDynSys); 
    spline_planner.set_sd_goal(goal, sd_goal_map);
    spline_planner.set_sd_obs(-env.masked_obs_map);
    start = [1, 1, 0, 0.01]; 
    opt_spline = spline_planner.plan(start);

    %% Plane parameters
    env.reachAvoidDynsys.x = start(1:3);
    tau = 0:0.5:10;
    reach_avoid_planner = ReachAvoidPlanner(env.grid_3d, env.reachAvoidSchemeData, tau);
    
    %% Plan!
    % note: need a non-zero starting velocity to avoid singularities in spline
    sd_goal_3d = shapeCylinder(env.grid_3d, 3, goal(1:3), goal_radius); 
    obstacle = -repmat(env.obs_map, 1, 1, env.grid_3d.N(3));
    optTrajDt = spline_planner.dt;
    reach_avoid_planner.solve_reach_avoid(start(1:3), goal(1:3), sd_goal_3d, obstacle, optTrajDt);   
    
    
    brs_planner = BRSAvoidPlanner(env.grid_3d, env.avoidBrsSchemeData, tau);
    brs_planner.solve_brs_avoid(obstacle); 
    save('./data/planners.mat');
end 




