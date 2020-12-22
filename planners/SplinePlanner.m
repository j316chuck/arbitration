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
        sd_obs
        sd_goal
        start
        goal
        opt_spline
    end
    
    methods
        %% Constructs Spline Planner.
        function obj = SplinePlanner(num_waypts, horizon, grid_2d, dynSys)
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
            [X2D, Y2D] = meshgrid(obj.gmin(1):obj.gdisc(1):obj.gmax(1), ...
                obj.gmin(2):obj.gdisc(2):obj.gmax(2));
            obj.disc_2d = [X2D(:), Y2D(:)];
        end
        
        %% Updates the signed distance to goal.
        function obj = set_sd_goal(obj, goal, sd_goal)
            obj.goal = goal; 
            obj.sd_goal = sd_goal;
        end
        %% Updates the signed distance to obsatacle.
        function obj = set_sd_obs(obj, sd_obs)
            obj.sd_obs = sd_obs;
        end
        
        function obj = set_spline_planning_points(obj, num_waypts, horizon)
            obj.num_waypts = num_waypts;
            obj.horizon = horizon;
            obj.dt = horizon / (num_waypts - 1); 
            obj.cur_spline_ctrl_idx = 1;
        end 
        
        %% Plans a path from start to goal.
        function opt_spline = plan(obj, start)
            obj.start = start;
            
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
                    reward = obj.eval_reward(curr_spline);
                    
                    if (reward < opt_reward)
                        opt_reward = reward;
                        opt_spline = curr_spline;
%                         figure(6);
%                         hold on
%                         contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.sd_obs, [0,0]);
%                         colors = [linspace(0,1,length(curr_spline{1}))', ...
%                                     zeros([length(curr_spline{1}), 1]), ...
%                                     zeros([length(curr_spline{1}), 1])];
%                         p = plot(curr_spline{1}, curr_spline{2});
%                         xlim([obj.gmin(1),obj.gmax(1)]);
%                         ylim([obj.gmin(2),obj.gmax(2)]);
%                         all_rewards(end+1) = reward;
%                         plt_handles{end+1} = p;
                        grid on
                        
                    end
                end
            end
            
            if isempty(opt_spline)
                warning("Unable to find dynamically feasible and low cost spline plan! Using old spline");
            else
                obj.opt_spline = opt_spline;
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
            
            % TODO: can optimize this by MATLAB-ifying it.
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
        
        %% Evaluates the total reward along the trajectory.
        function reward = eval_reward(obj, curr_spline)
            xs = curr_spline{1};
            ys = curr_spline{2};
            traj = [xs', ys'];
            % TODO: add in penalty for orientation too?
            % TODO: add in penalty for human-driven vehicle prediction.
            obs_r = eval_u(obj.grid_2d, obj.sd_obs, traj);
            goal_r = eval_u(obj.grid_2d, obj.sd_goal, traj);
         
            reward = sum(obs_r + goal_r);
        end        
    end
end

