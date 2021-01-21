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
        
        %% Sets the signed distance to goal.
        function obj = set_sd_goal(obj, goal, sd_goal)
            obj.goal = goal; 
            obj.sd_goal = sd_goal;
        end
        %% Sets the signed distance to obstacle.
        function obj = set_sd_obs(obj, sd_obs, weight)
            obj.sd_obs = sd_obs * weight;
        end
        
        %% Sets the num of spline planning points
        function obj = set_spline_planning_points(obj, num_waypts, horizon)
            obj.num_waypts = num_waypts;
            obj.horizon = horizon;
            obj.dt = horizon / (num_waypts - 1); 
            obj.cur_spline_ctrl_idx = 1;
        end 
        
        %% Plans a path from start to goal.
        function opt_spline = plan(obj, start)
            obj.start = start;            
            opt_cost = 100000000000000.0;
            opt_spline = {};
            curr_spline = {};
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
        
        %% Replans a path that uses thresholded safety and distance scores to find a safe trajectory 
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
            
            for ti=1:length(obj.disc_3d) 
                candidate_goal = obj.disc_3d(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.grid_2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal,  0.01]; %[candidate_goal, 0.01];
                
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
                    safety_cost = obj.eval_traj_threshold_safety_cost(curr_spline, brs_planner); 
                    replan_cost = obj.eval_traj_threshold_replan_cost(curr_spline, traj_xs, traj_ys); 
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
        
        %% Replans a path that uses only safe trajectories
        function opt_spline = replan_only_safe_traj(obj, start, brs_planner, zero_level_set, mpc_horizon)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_cost = 100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            all_rewards = [];
            plt_handles = {};
            
            for ti=1:length(obj.disc_3d) 
                candidate_goal = obj.disc_3d(ti, :);
                
                % ignore candidate goals inside obstacles.
                if eval_u(obj.grid_2d, obj.sd_obs, candidate_goal(1:2)) < 0
                    continue;
                end
                
                % orientation should match with goal final vel ~= 0.
                candidate_goal = [candidate_goal,  0.01]; %[candidate_goal, 0.01];
                
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
                
                % If current spline is dyamically feasible, safe, and low
                % cost, then we replace with the optimal trajectory
                goal_cost = obj.eval_goal_cost(curr_spline);
                obs_cost = obj.eval_obstacle_cost(curr_spline); 
                spline_cost = obs_cost + goal_cost;
                if feasible_horizon <= obj.horizon && spline_cost < opt_cost 
                    spline_vec = [curr_spline{1}; curr_spline{2}; curr_spline{3}];
                    spline_vec = spline_vec(:, 1:mpc_horizon);
                    alphas = brs_planner.get_value(spline_vec);
                    is_safe = ~(any(alphas < zero_level_set)); 
                    if is_safe
                        opt_cost = spline_cost;
                        opt_spline = curr_spline;                        
                    end
                end   
            end
            if isempty(opt_spline)
                warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
            end
        end 
        
        %% Replans a path that minimizes distance between the original trajectory and the safety trajectory
        function opt_spline = replan_with_safety_controls(obj, start, traj_xs, traj_ys, safe_xs, safe_ys, alpha)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_reward = 100000000000000.0;
            opt_spline = {};        
            
            for ti=1:length(obj.disc_3d) 
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
        
        %% Replans a path from start to goal that goes through an intermediate waypoint; 
        function opt_spline = replan_through_waypoint(obj, start, intermediate_waypoint)
            obj.start = start;
            
            opt_cost = 100000000000000.0;
            opt_spline = {};
            curr_spline = {};
            
            % DEBUGGING
            %figure
            N = length(obj.disc_3d);
            obj.all_costs = zeros(N, 5);
            num_candidate_splines = 0; 
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
               
                % If current spline is dyamically feasible and passes through the waypoint, check if it is low cost.
                spline_dist_radius = 0.5;
                is_valid_horizon = feasible_horizon <= obj.horizon;
                spline_passes_point = obj.spline_passes_through_point(curr_spline, intermediate_waypoint, spline_dist_radius);
                if  is_valid_horizon && spline_passes_point
                    goal_cost = obj.eval_goal_cost(curr_spline);
                    obs_cost = obj.eval_obstacle_cost(curr_spline); 
                    total_cost = obs_cost + goal_cost;
                    if (total_cost < opt_cost)
                        opt_cost = total_cost;
                        opt_spline = curr_spline;
                    end
                    num_candidate_splines = num_candidate_splines + 1; 
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
            fprintf("Total num candidate splines: %d\n", num_candidate_splines); 
        end
        
        %% Replans a path from start to goal that goes through an intermediate waypoint;
        function opt_spline = adaptive_replan_through_waypoint(obj, ...
                                                               start, ...
                                                               intermediate_waypoint,...
                                                               max_num_candidate_splines)
            % Adaptively adjusts the dist to spline threshold until 
            % the spline planner evaluates between 
            % 1 and max_num_candidate_splines number of candidate splines
            obj.start = start;
            spline_dist_hi = 2; spline_dist_lo = 0; spline_dist_radius = 1;
            num_candidate_splines = max_num_candidate_splines + 1; 
            num_iter = 0; 
            while true && num_iter < 10 % avoid infinite loop
                num_iter = num_iter + 1;
                if 0 < num_candidate_splines && num_candidate_splines <= max_num_candidate_splines
                    break 
                elseif num_candidate_splines == 0 % increase spline dist thresh
                    spline_dist_lo = spline_dist_radius; 
                elseif num_candidate_splines > max_num_candidate_splines % decrease spline dist thresh
                    spline_dist_hi = spline_dist_radius;                     
                end
                spline_dist_radius = (spline_dist_hi + spline_dist_lo) / 2;
                
                % Spline planner portion
                N = length(obj.disc_3d);
                obj.all_costs = zeros(N, 5);
                num_candidate_splines = 0; 
                opt_cost = 100000000000000.0;
                opt_spline = {};
                curr_spline = {};
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

                    % If current spline is dyamically feasible and passes 
                    % through the waypoint within a certain threshold, check if it is low cost.
                    is_valid_horizon = (feasible_horizon <= obj.horizon);
                    spline_passes_point = obj.spline_passes_through_point(curr_spline, .... 
                                    intermediate_waypoint, spline_dist_radius);
                    if is_valid_horizon && spline_passes_point 
                        goal_cost = obj.eval_goal_cost(curr_spline);
                        obs_cost = obj.eval_obstacle_cost(curr_spline); 
                        total_cost = obs_cost + goal_cost;
                        if (total_cost < opt_cost)
                            opt_cost = total_cost;
                            opt_spline = curr_spline;
                        end
                        num_candidate_splines = num_candidate_splines + 1; 
                        obj.all_costs(ti, :) = [candidate_goal(1), candidate_goal(2), goal_cost, obs_cost, total_cost];
                    else
                        BIG_COST = 1000; % hyperparams to make contour plots look pretty
                        SMALL_COST = 200; % hyperparams to make contour plots look pretty
                        obj.all_costs(ti, :) = [candidate_goal(1), candidate_goal(2), SMALL_COST, BIG_COST, BIG_COST]; 
                    end
                end
                fprintf("Spline dist threshold %f, num valid splines: %d\n", spline_dist_radius, num_candidate_splines); 
                if isempty(opt_spline)
                    warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
                else
                    obj.opt_spline = opt_spline;
                end 
            end 
        end
        
        %% Replans by choosing alpha proportional to the value at each state:
        %       
        %   x^* = arg min_x \sum^T_{t=0} cost(x_t, x^plan_t, x^safe_t)
        %           s.t. x_{t+1} = f(x_t, u_t)   \forall t \in [0,T]
        %   
        %   where the running cost function is:
        %       cost(., ., .) = alpha(x_t)*||x_t - x^plan_t|| + 
        %                             (1-alpha(x_t)) * || x_t - x^safe_t||
        % 
        %   and the blending is via the value function:
        %       alpha(x_t) = V^safe(x_t)
        %  
        function [opt_spline, opt_alphas] = open_loop_replan_with_value_blending(obj, start, ...
                                                        traj_xs, traj_ys, ...
                                                        safe_xs, safe_ys, ...
                                                        brs_planner, blend_function)
            %figure(8);
            %clf(8)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_reward = 100000000000000.0;
            opt_spline = {}; 
            opt_alphas = [];
            
            for ti=1:length(obj.disc_3d) 
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
                    spline_xs = curr_spline{1}; 
                    spline_ys = curr_spline{2};
                    spline_ths = curr_spline{3};
                    spline_vec = [spline_xs', spline_ys', spline_ths'];
                    
                    % Get a vector of the safety values at each planned state
                    raw_alphas_along_spline = brs_planner.get_value(spline_vec);
                    alphas_along_spline = blend_function(raw_alphas_along_spline);
                    
                    % Compute weighted plan-relevant part of objective: 
                    %   alpha(x_t) * || x_t - x^plan_t || for all times
                    replan_dist = alphas_along_spline .* obj.l2_dist(spline_xs(:), traj_xs(:), spline_ys(:), traj_ys(:));
                    replan_cost = sum(replan_dist);
                    
                    % Compute weighted safety-relevant part of objective: 
                    %   (1 - alpha(x_t)) * || x_t - x^safe_t|| for all times
                    safety_dist = (1 - alphas_along_spline) .* obj.l2_dist(spline_xs(:), safe_xs(:), spline_ys(:), safe_ys(:));
                    safety_cost = sum(safety_dist);
                    
                    % Compute the total objective summed over time.
                    reward = replan_cost + safety_cost;
                    
                    replan_score = [candidate_goal(1); candidate_goal(2); safety_cost; replan_cost; reward];
                    obj.replan_scores = [obj.replan_scores, replan_score];
                    
                    if (reward < opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;    
                        opt_alphas = alphas_along_spline;
                        
                        % Plots the intermeddiate optimal plans for
                        % debugging!
                        % obj.plot_plans_and_alphas(traj_xs, traj_ys, ...
                        %                safe_xs, safe_ys, ...
                        %                curr_spline, ...
                        %                alphas_along_spline);
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
        
        %% Replans by choosing alpha proportional to the value at each state
        %  The distance in the second equation is a closed loop distance
        %  between the safest action and the current action
        %   x^* = arg min_x \sum^T_{t=0} cost(x_t, x^plan_t, x^safe_t)
        %           s.t. x_{t+1} = f(x_t, u_t)   \forall t \in [0,T]
        %   
        %   where the running cost function is:
        %       cost(., ., .) = alpha(x_t)*||x_t - x^plan_t|| + 
        %                             (1-alpha(x_t)) * ||x_t - f(x_{t-1}, u_{safe})||
        % 
        %   and the blending is via the value function:
        %       alpha(x_t) = V^safe(x_t)
        %       u_{t} = safety control
        %  
        function [opt_spline, opt_alphas] = closed_loop_replan_with_value_blending(obj, start, ...
                                                        traj_xs, traj_ys, ...
                                                        brs_planner, blend_function)
            %figure(8);
            %clf(8)
            obj.start = start;
            obj.replan_scores = zeros(5, 0);
            
            opt_reward = 100000000000000.0;
            opt_spline = {}; 
            opt_alphas = [];
            
            for ti=1:length(obj.disc_3d) 
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
                    spline_xs = curr_spline{1}; 
                    spline_ys = curr_spline{2};
                    spline_ths = curr_spline{3};
                    spline_vec = [spline_xs', spline_ys', spline_ths'];
                    
                    % Get a vector of the safety values at each planned state
                    raw_alphas_along_spline = brs_planner.get_value(spline_vec);                    
                    alphas_along_spline = blend_function(raw_alphas_along_spline);

                    % Compute weighted plan-relevant part of objective: 
                    %   alpha(x_t) * || x_t - x^plan_t || for all times
                    replan_dist = alphas_along_spline .* obj.l2_dist(spline_xs(:), traj_xs(:), spline_ys(:), traj_ys(:));
                    replan_cost = sum(replan_dist);
                    
                    % Compute weighted safety-relevant part of objective: 
                    %   (1 - alpha(x_t)) * || x_t - f(x_{t-1}, u_{safe}) || for all times
                    Ns = length(alphas_along_spline) - 1; 
                    safe_states = zeros(3, Ns);  
                    for i = 1:Ns
                       state = [spline_xs(i), spline_ys(i), spline_ths(i)]; 
                       safe_state = brs_planner.use_avoid_control(state); 
                       safe_states(:, i) =  safe_state; 
                    end 
                    safe_xs = safe_states(1, :); 
                    safe_ys = safe_states(2, :); 
                    spline_xs = spline_xs(2:end); 
                    spline_ys = spline_ys(2:end); 
                    safety_dist = (1 - alphas_along_spline(2:end)) .* obj.l2_dist(spline_xs(:), safe_xs(:), spline_ys(:), safe_ys(:));
                    safety_cost = sum(safety_dist);
                    % Compute the total objective summed over time.
                    reward = replan_cost + safety_cost;
    
                    replan_score = [candidate_goal(1); candidate_goal(2); safety_cost; replan_cost; reward];
                    obj.replan_scores = [obj.replan_scores, replan_score];
                    if (reward < opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;    
                        opt_alphas = alphas_along_spline;
                        fprintf("Total cost %f Safety cost: %f replan cost: %f\n", reward, safety_cost, replan_cost); 
                        % Plots the intermeddiate optimal plans for
                        % debugging!
                        %obj.plot_plans_and_alphas(traj_xs, traj_ys, ...
                        %safe_xs, safe_ys, curr_spline, alphas_along_spline);
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
        
        %% This function plots the original planned trajectory (in black)
        % the safety trajectory (in red) and the blended trajectory where
        % the alpha \in [0,1] and each blended waypoint is colored
        % proportional to its alpha blending value. 
        function plot_plans_and_alphas(obj, traj_xs, traj_ys, ...
                                        safe_xs, safe_ys, ...
                                        curr_spline, ...
                                        alphas_along_spline)
            % Assumes alphas are normalized to [0,1] to 
            % color code the planned trajectory based on alphas!
            min_alpha = 0; %min(alphas_along_spline);
            max_alpha = 1; %max(alphas_along_spline);
            
            figure(8);
            hold on;
            % Plot the background obstacles.
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.sd_obs, [0 0.05]);
            sp = scatter(traj_xs, traj_ys, 15, 'black', 'filled', 'DisplayName', 'plan');
            sp.HandleVisibility = 'off';
            ss = scatter(safe_xs, safe_ys, 15, [168/255., 8/255., 0], 'filled', 'DisplayName', 'plan');
            ss.HandleVisibility = 'off';
            for i=1:length(curr_spline{1})
                curr_alpha = alphas_along_spline(i);
                [val_color, ~] = custom_colormap(curr_alpha, min_alpha, max_alpha);
                sb = scatter(curr_spline{1}(i), curr_spline{2}(i), ...
                            15, val_color, ...
                            'filled', 'DisplayName', 'plan');
                sb.HandleVisibility = 'off';
            end
            [~, cmap] = custom_colormap(curr_alpha, min_alpha, max_alpha);
            colormap(cmap);
            h = colorbar;
            caxis([min_alpha, max_alpha]);
            ylabel(h, 'alpha (low: more safe, high: more plan)');
        end
                
        %% Plot all spline replan scores
        function plot_replan_scores(obj, savefig_path)
            opt_xs = obj.opt_spline{1}; opt_ys = obj.opt_spline{2};
            sx = opt_xs(1); sy = opt_ys(1); gx = opt_xs(end); gy = opt_ys(end);
            N = numel(obj.x2d); 
            safety_cost = reshape(obj.replan_scores(3, 1:N), size(obj.x2d));
            replan_cost = reshape(obj.replan_scores(4, 1:N), size(obj.x2d));
            reward = reshape(obj.replan_scores(5, 1:N), size(obj.x2d));
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
            title("Safety Cost (Theta 0)");
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
            title("Replan Cost (Theta 0)");
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
            title("Alpha Blend Cost (Theta 0)");
            xlabel("x (m)");
            ylabel("y (m)");
            if ~isempty(savefig_path)
                savefig(savefig_path); 
            end 
        end 
        
        %% Debugging plots to see which splines pass through point
        function plot_spline_passes_through_point(obj, curr_spline, pt)
            figure(11); 
            hold on; 
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0]);
            plot(curr_spline{1}, curr_spline{2}, 'b--o'); 
            scatter(pt(1), pt(2), 20, 'rx'); 
        end 
        
        %% Plot all spline costs that it evaluates
        function plot_spline_costs(obj, savefig_path)
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
        
        %% Check if spline trajectory passes through a point up to a certain radius 
        function pass = spline_passes_through_point(obj, curr_spline, pt, radius)
            %obj.plot_spline_passes_through_point(curr_spline, pt); % debug
            pass = false; 
            pt_x = pt(1); pt_y = pt(2); pt_theta = pt(3);
            ANGLE_THRESH = pi/10; 
            for i=1:obj.num_waypts
                x = curr_spline{1}(i);
                y = curr_spline{2}(i);
                t = curr_spline{3}(i);
                d = obj.l2_dist(x, pt_x, y, pt_y);
                angle_is_close = abs(t - pt_theta) < ANGLE_THRESH;
                point_is_close = d < radius; 
                if angle_is_close && point_is_close 
                    pass = true;
                    return;
                end 
            end
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
        
        %% Evaluates the l2 dist
        function d = l2_dist(obj, x1, x2, y1, y2)
            d = ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
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
        
        %% Evaluates the total cost along the trajectory.
        function cost = eval_cost(obj, curr_spline)
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            obs_r = eval_u(obj.grid_2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.grid_2d, obj.sd_goal, traj);
         
            cost = sum(obs_r + goal_r);
        end     
        
        %% Evaluates the safety cost along the trajectory
        function safety_cost = eval_traj_safety_cost(obj, curr_spline, brs_planner)
            safety_cost = 0;
            xs = curr_spline{1}; ys = curr_spline{2}; ths = curr_spline{3}; 
            for i = 1:length(xs)
                state = [xs(i), ys(i), ths(i)];
                cost = brs_planner.get_value(state(1:3));
                safety_cost = safety_cost + cost;
            end 
        end
        
        %% Evaluates the safety cost thresholded by a constant value along the trajectory    
        function safety_cost = eval_traj_threshold_safety_cost(obj, curr_spline, brs_planner)
            safety_cost = 0;
            xs = curr_spline{1}; ys = curr_spline{2}; ths = curr_spline{3}; 
            for i = 1:length(xs)
                state = [xs(i), ys(i), ths(i)];
                v = brs_planner.get_value(state(1:3));
                cost = (v <= 0.2); %zero_level_set threshold
                safety_cost = safety_cost + cost;
            end 
        end 

        %% Evaluates the cost of replanning a new trajectory with respect to the current spline thresholded by a value
        function replan_cost = eval_traj_threshold_replan_cost(obj, curr_spline, old_xs, old_ys)
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
    end
end

