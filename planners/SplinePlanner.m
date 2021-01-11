classdef SplinePlanner < handle
    %SPLINEPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        num_waypts
        horizon
        dt
        cur_spline_ctrl_idx
        grid_2d
        dynSys
        max_linear_vel
        max_angular_vel 
        gmin
        gmax
        gnum
        gdisc
        disc_2d
        disc_3d
        sd_obs
        sd_goal
        start
        goal
        opt_spline
        traj_xs
        traj_ys 
        replan_scores %x, y, safety_cost, replan_cost, reward
        binary_occ_map
        x2d
        y2d
        x3d 
        y3d
        t3d 
        all_costs
    end
    
    methods
        %% Constructs Spline Planner.
        function obj = SplinePlanner(num_waypts, horizon, grid_2d, dynSys, binary_occ_map)
            obj.num_waypts = num_waypts;
            obj.horizon = horizon;
            obj.dt = horizon / (num_waypts - 1);
            obj.cur_spline_ctrl_idx = 1;
            obj.grid_2d = grid_2d;
            obj.dynSys = dynSys;
            obj.max_linear_vel = dynSys.vrange(2);
            obj.max_angular_vel = dynSys.wMax;
            obj.gmin = grid_2d.min';
            obj.gmax = grid_2d.max';
            obj.gnum = grid_2d.N';
            obj.gdisc = (obj.gmax - obj.gmin) ./ (obj.gnum - 1);
            [x2d, y2d] = meshgrid(grid_2d.vs{1}, grid_2d.vs{2});
            obj.x2d = x2d;
            obj.y2d = y2d;
            obj.disc_2d = [obj.x2d(:), obj.y2d(:)];
            obj.t3d = 0:pi/2:(2*pi-0.01);
            [x3d, y3d, t3d] = meshgrid(grid_2d.vs{1}, grid_2d.vs{2}, obj.t3d); 
            obj.disc_3d = [x3d(:), y3d(:), t3d(:)];
            obj.binary_occ_map = binary_occ_map; 
        end
        
        %% Updates the signed distance to goal.
        function obj = set_sd_goal(obj, goal, sd_goal)
            obj.goal = goal; 
            obj.sd_goal = sd_goal;
        end
        %% Updates the signed distance to obsatacle.
        function obj = set_sd_obs(obj, sd_obs, weight)
            obj.sd_obs = sd_obs * weight;
        end
        
        function obj = set_spline_planning_points(obj, num_waypts, horizon)
            obj.num_waypts = num_waypts;
            obj.horizon = horizon;
            obj.dt = horizon / (num_waypts - 1); 
            obj.cur_spline_ctrl_idx = 1;
        end 
        
        %% Replans a path from the base path
        function opt_spline = replan_with_brs_planner(obj, start, traj_xs, traj_ys, brs_planner, alpha)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_reward = 100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            all_rewards = [];
            plt_handles = {};
            
            for ti=1:length(obj.disc_2d) 
                candidate_goal = obj.disc_2d(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.grid_2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, 0.01]; %[candidate_goal, 0.01];
                
                % Compute spline from start to candidate (x,y) goal.
                curr_spline = ...
                    spline(start, candidate_goal, obj.horizon, obj.num_waypts);
                
                % Sanity check (and correct) all points on spline to be within env bounds.
                curr_spline = obj.sanity_check_spline(curr_spline);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon = ...
                    obj.compute_dyn_feasible_horizon(curr_spline, ...
                    obj.max_linear_vel, ...
                    obj.max_angular_vel, ...
                    obj.horizon);
                
                % If current spline is dyamically feasible, check if it is low cost.
                if (feasible_horizon <= obj.horizon)
                    safety_cost = obj.eval_traj_safety_cost(curr_spline, brs_planner); 
                    replan_cost = obj.eval_traj_replan_cost(curr_spline, traj_xs, traj_ys); 
                    reward = alpha * replan_cost + (1-alpha) * safety_cost;
                    replan_score = [candidate_goal(1); candidate_goal(2); safety_cost; replan_cost; reward];
                    obj.replan_scores = [obj.replan_scores, replan_score];

                    if (reward < opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;                        
                    end
                else 
                    replan_score = [candidate_goal(1); candidate_goal(2); -5; -5; -5];
                    obj.replan_scores = [obj.replan_scores, replan_score];
                end             
            end
            if isempty(opt_spline)
                warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
            end
        end 
        
        %% Replans a path from the base path
        function opt_spline = replan_with_safety_controls(obj, start, traj_xs, traj_ys, safe_xs, safe_ys, alpha)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_reward = 100000000000000.0;
            opt_spline = {};        
            
            for ti=1:length(obj.disc_2d) 
                candidate_goal = obj.disc_2d(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.grid_2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, obj.goal(3), 0.01]; %[candidate_goal, 0.01];
                
                % Compute spline from start to candidate (x,y) goal.
                curr_spline = ...
                    spline(start, candidate_goal, obj.horizon, obj.num_waypts);
                
                % Sanity check (and correct) all points on spline to be within env bounds.
                curr_spline = obj.sanity_check_spline(curr_spline);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon = ...
                    obj.compute_dyn_feasible_horizon(curr_spline, ...
                    obj.max_linear_vel, ...
                    obj.max_angular_vel, ...
                    obj.horizon);
                
                % If current spline is dyamically feasible, check if it is low cost.
                if (feasible_horizon <= obj.horizon)
                    spline_xs = curr_spline{1}; spline_ys = curr_spline{2};
                    safety_cost = sum(obj.l2_dist(spline_xs(:), safe_xs(:), spline_ys(:), safe_ys(:))); 
                    replan_cost = sum(obj.l2_dist(spline_xs(:), traj_xs(:), spline_ys(:), traj_ys(:))); 
                    reward = alpha * replan_cost + (1-alpha) * safety_cost;
                    replan_score = [candidate_goal(1); candidate_goal(2); safety_cost; replan_cost; reward];
                    obj.replan_scores = [obj.replan_scores, replan_score];
                    if (reward < opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;                        
                    end
                else 
                    replan_score = [candidate_goal(1); candidate_goal(2); -5; -5; -5];
                    obj.replan_scores = [obj.replan_scores, replan_score];
                end             
            end
            if isempty(opt_spline)
                warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
            end
        end 
        
        
        function plot_replan_scores(obj, savefig_path)
            opt_xs = obj.opt_spline{1}; opt_ys = obj.opt_spline{2};
            sx = opt_xs(1); sy = opt_ys(1); gx = opt_xs(end); gy = opt_ys(end);
            xs = obj.replan_scores(1, :); 
            ys = obj.replan_scores(2, :); 
            safety_cost = reshape(obj.replan_scores(3, :), size(obj.x2d));
            replan_cost = reshape(obj.replan_scores(4, :), size(obj.x2d));
            reward = reshape(obj.replan_scores(5, :), size(obj.x2d));
            figure(6); 
            clf; 
            gxs = obj.grid_2d.xs{1}; gys = obj.grid_2d.xs{2};
            set(gcf,'Position', [10 10 800 900])
            subplot(2, 2, 1); 
            hold on 
            contourf(gxs, gys, obj.binary_occ_map, [0 0]);
            scatter(sx, sy, 20, 'go'); 
            scatter(gx, gy, 20, 'rx'); 
            colormap(gca,'gray')
            hcb1 = colorbar;
            title("Obstacle");
            xlabel("x (m)");
            ylabel("y (m)"); 
            subplot(2, 2, 2); 
            hold on 
            contourf(obj.x2d, obj.y2d, safety_cost);
            scatter(sx, sy, 20, 'go'); 
            scatter(gx, gy, 20, 'rx');
            contour(gxs, gys, obj.binary_occ_map, [0 0]);
            colormap(gca, 'jet');
            hcb2 = colorbar;
            title("Safety Cost");
            xlabel("x (m)");
            ylabel("y (m)"); 
            subplot(2, 2, 3); 
            hold on 
            contourf(obj.x2d, obj.y2d, replan_cost);
            scatter(sx, sy, 20, 'go'); 
            scatter(gx, gy, 20, 'rx'); 
            contour(gxs, gys, obj.binary_occ_map, [0 0]);
            colormap(gca, 'jet');
            hcb2 = colorbar;
            title("Replan Cost");
            xlabel("x (m)");
            ylabel("y (m)"); 
            subplot(2, 2, 4); 
            hold on 
            contourf(obj.x2d, obj.y2d, reward);
            scatter(sx, sy, 20, 'go'); 
            scatter(gx, gy, 20, 'rx');
            contour(gxs, gys, obj.binary_occ_map, [0 0]);
            colormap(gca,'jet');
            hcb2 = colorbar;
            title("Alpha Blend Cost");
            xlabel("x (m)");
            ylabel("y (m)");
            if ~isempty(savefig_path)
                savefig(savefig_path); 
            end 
        end 
            
        function safety_cost = eval_traj_safety_cost(obj, curr_spline, brs_planner)
            safety_cost = 0;
            xs = curr_spline{1}; ys = curr_spline{2}; ths = curr_spline{3}; 
            for i = 1:length(xs)
                state = [xs(i), ys(i), ths(i)];
                v = brs_planner.get_value(state(1:3));
                cost = (v <= 0.2); %zero_level_set threshold
                safety_cost = safety_cost + cost;
            end 
        end 

        function d = l2_dist(obj, x1, x2, y1, y2)
            d =  ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
        end

        function replan_cost = eval_traj_replan_cost(obj, curr_spline, old_xs, old_ys)
            xs = curr_spline{1}; ys = curr_spline{2};
            replan_cost = 0;
            if length(old_xs) ~= length(old_ys) || length(old_xs) ~= length(xs) || length(xs) ~= length(ys)
                warning("Input trajectories not of equal length"); 
                replan_cost = inf; 
                return; 
            end 
            for i = 1:length(xs)
                x1 = old_xs(i); x2 = xs(i); y1 = old_ys(i); y2 = ys(i); 
                d = obj.l2_dist(x1, x2, y1, y2);
                cost = (d >= 0.5); %(d >= 1.5); 
                replan_cost = replan_cost + cost;
            end 
        end 

        
        %% Plans a path from start to goal.
        function opt_spline = plan(obj, start)
            obj.start = start;
            
            opt_cost = 100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            N = length(obj.disc_3d);
            obj.all_costs = zeros(N, 5);
            for ti=1:N
                candidate_goal = obj.disc_3d(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.grid_2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal, 0.01]; %[candidate_goal, 0.01];
                
                % Compute spline from start to candidate (x,y) goal.
                curr_spline = ...
                    spline(start, candidate_goal, obj.horizon, obj.num_waypts);
                
                % Sanity check (and correct) all points on spline to be within env bounds.
                curr_spline = obj.sanity_check_spline(curr_spline);
                
                % Compute the dynamically feasible horizon for the current plan.
                feasible_horizon = ...
                    obj.compute_dyn_feasible_horizon(curr_spline, ...
                    obj.max_linear_vel, ...
                    obj.max_angular_vel, ...
                    obj.horizon);
               
                % If current spline is dyamically feasible, check if it is low cost.
                if (feasible_horizon <= obj.horizon)
                    goal_cost = obj.eval_goal_cost(curr_spline);
                    obs_cost = obj.eval_obstacle_cost(curr_spline); 
                    total_cost = obs_cost + goal_cost;
                    if (total_cost < opt_cost)
                        opt_cost = total_cost;
                        opt_spline = curr_spline;
                    end
                    obj.all_costs(ti, :) = [candidate_goal(1), candidate_goal(2), goal_cost, obs_cost, total_cost];
                else
                    BIG_COST = 1000; % hyperparams to make contour plots look pretty
                    SMALL_COST = 200; % hyperparams to make contour plots look pretty
                    obj.all_costs(ti, :) = [candidate_goal(1), candidate_goal(2), SMALL_COST, BIG_COST, BIG_COST]; 
                end
            end
            if isempty(opt_spline)
                warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
            else
                obj.opt_spline = opt_spline;
            end 
        end
       
        function plot_spline_costs(obj, savefig_path)
            %% Plot all spline costs
            cost_names = {'goal_cost', 'obs_cost', 'total_cost'};
            for ci = 1:length(cost_names)
                h = figure(3); 
                clf;
                set(gcf, 'Position', [100, 100, 1200, 900])
                N = length(obj.t3d); 
                Nc = 3;
                Nr = ceil(N / Nc); 
                ns = prod(obj.grid_2d.N); 
                name = cost_names{ci};
                for ti = 1:N
                    % getting reward at specific theta
                    theta = obj.t3d(ti);
                    si = (ti-1) * ns + 1;
                    ei = ti * ns;
                    ai = ci + 2; % all_costs_index for goal, obs, total cost is 3, 4, 5
                    rews = obj.all_costs(si:ei, ai); 
                    rews = reshape(rews, obj.grid_2d.N')'; % reshape and take tranpose
                    % plotting the obstacle, start point, and spline rewards
                    subplot(Nr, Nc, ti); 
                    hold on; 
                    title(sprintf("%s theta %f", name, theta), 'interpreter', 'none'); 
                    xlabel('x(m)');
                    ylabel('y(m)'); 
                    contourf(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, rews);
                    colorbar;
                    contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0]);
                    scatter(obj.start(1), obj.start(2), 100, 'rx'); 
                    th = obj.start(3);
                    quiver(obj.start(1), obj.start(2), cos(th), sin(th));
                end 
                if ~isempty(savefig_path)
                    path = sprintf("%s_%s.fig", savefig_path, name); 
                    savefig(h, path);
                end 
            end 
        end
        
        function u = get_mpc_control(obj)
            if isempty(obj.opt_spline) || length(obj.opt_spline{4}) < 1
                error("Opt spline not calculated yet"); 
            end 
            u = [obj.opt_spline{4}(2), obj.opt_spline{5}(2)];
        end 
        
        function u = get_next_control(obj)
            if isempty(obj.opt_spline) || length(obj.opt_spline{4}) < 1
                error("Opt spline not calculated yet"); 
            end 
            u = [obj.opt_spline{4}(obj.cur_spline_ctrl_idx), obj.opt_spline{5}(obj.cur_spline_ctrl_idx)];
            obj.cur_spline_ctrl_idx =  obj.cur_spline_ctrl_idx + 1;
        end 
        

        
        %% Check (and correct) if spline is inside environment bounds.
        function checked_spline = sanity_check_spline(obj, curr_spline)
            checked_spline = curr_spline;
            
            for i=1:obj.num_waypts
                x = checked_spline{1}(i);
                y = checked_spline{2}(i);
                
                if (x > obj.gmax(1))
                    checked_spline{1}(i) = obj.gmax(1);
                end
                if (y > obj.gmax(2))
                    checked_spline{2}(i) = obj.gmax(2);
                end
                if (x < obj.gmin(1))
                    checked_spline{1}(i) = obj.gmin(1);
                end
                if (y < obj.gmin(2))
                    checked_spline{2}(i) = obj.gmin(2);
                end
            end
        end
        
        
        
        %% Computes dynamically feasible horizon (given dynamics of car).
        function feasible_horizon = ...
                compute_dyn_feasible_horizon(obj, spline, ...
                max_linear_vel, ...
                max_angular_vel, ...
                final_t)
            % Compute max linear and angular speed.
            plan_max_lin_vel = max(spline{4});
            plan_max_angular_vel = max(abs(spline{5}));
            
            % Compute required horizon to acheive max speed of planned spline.
            feasible_horizon_speed = ...
                final_t * plan_max_lin_vel / max_linear_vel;
            
            % Compute required horizon to acheive max angular vel of planned spline.
            feasible_horizon_angular_vel = ...
                final_t * plan_max_angular_vel / max_angular_vel;
            
            feasible_horizon = ...
                max(feasible_horizon_speed, feasible_horizon_angular_vel);
        end
        
         %% Evaluates the obstacle cost along the trajectory.
        function obs_cost = eval_obstacle_cost(obj, curr_spline)
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            obs_cost = sum(eval_u(obj.grid_2d, obj.sd_obs, traj));
        end
        
        %% Evaluates the goal cost along the trajectory.
        function goal_cost = eval_goal_cost(obj, curr_spline)
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            goal_cost = sum(eval_u(obj.grid_2d, obj.sd_goal, traj));
        end 
        
        %% Evaluates the total reward along the trajectory.
        function reward = eval_reward(obj, curr_spline)
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            obs_r = eval_u(obj.grid_2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.grid_2d, obj.sd_goal, traj);
         
            reward = sum(obs_r + goal_r);
        end        
    end
end

