classdef objectives < handle
    %Evaluate objective costs
    
    properties
        lin_vel
        avg_lin_vel
        lin_accel
        avg_lin_accel
        lin_jerk
        avg_lin_jerk  
        ang_vel 
        avg_ang_vel
        ang_accel
        avg_ang_accel
        ang_jerk
        avg_ang_jerk
        dist_to_goal
        avg_dist_to_goal
        safety_score
        avg_safety_score
        dist_to_opt_traj
        avg_dist_to_opt_traj
    end
    
    methods
        function obj = objectives(blend_traj, opt_traj, brs_planner, dt, goal)
            blend_xs = blend_traj(1, :); 
            blend_ys = blend_traj(2, :); 
            blend_ths = blend_traj(3, :);
            angular_vel = blend_traj(5, :); 
            reach_avoid_xs = opt_traj(1, :);
            reach_avoid_ys = opt_traj(2, :);
            obj.calc_linear_kinematics(blend_xs, blend_ys, dt)
            obj.calc_angular_kinematics(angular_vel, dt)
            obj.calc_dist_to_goal(blend_xs, blend_ys, goal); 
            obj.calc_dist_to_opt_traj(blend_xs, blend_ys, reach_avoid_xs, reach_avoid_ys)
            obj.calc_safety_score(blend_xs, blend_ys, blend_ths, brs_planner); 
        end
        
        function [d] = l2_dist(~, x, y) 
            d = (x .^ 2 + y .^ 2) .^ 0.5;
        end 
        
        function obj = calc_linear_kinematics(obj, x, y, dt)
            if length(x) < 4 || length(y) < 4 || length(x) ~= length(y)
                obj.avg_lin_vel = -1;
                obj.avg_lin_accel = -1;
                obj.avg_lin_accel = -1;
                return 
            end
            vx = x(2:length(x)) - x(1:length(x) - 1);
            vy = y(2:length(y)) - y(1:length(x) - 1); 
            ax = vx(2:length(vx)) - vx(1:length(vx) - 1); 
            ay = vy(2:length(vy)) - vy(1:length(vy) - 1); 
            jx = ax(2:length(ax)) - ax(1:length(ax) - 1);
            jy = ay(2:length(ay)) - ay(1:length(ay) - 1); 
            obj.lin_vel = 1/dt .* obj.l2_dist(vx, vy); 
            obj.avg_lin_vel = mean(obj.lin_vel);
            obj.lin_accel = 1/dt .* obj.l2_dist(ax, ay); 
            obj.avg_lin_accel = mean(obj.lin_accel); 
            obj.lin_jerk =  1/dt .* obj.l2_dist(jx, jy);
            obj.avg_lin_jerk = mean(obj.lin_accel);  
        end 

        function obj = calc_angular_kinematics(obj, ang_vel, dt)
            n = length(ang_vel);
            obj.ang_vel = ang_vel; 
            obj.avg_lin_vel = mean(abs(obj.ang_vel));
            obj.ang_accel = 1/dt .* (ang_vel(2:n) - ang_vel(1:n-1)); 
            obj.avg_ang_accel = mean(abs(obj.ang_accel)); 
            obj.ang_jerk =  1/dt .* (obj.ang_accel(2:n-1) - obj.ang_accel(1:n-2));
            obj.avg_ang_jerk = mean(abs(obj.ang_jerk));  
        end 
        
        function obj = calc_dist_to_goal(obj, x, y, goal)
            if length(x) ~= length(y) 
                obj.dist_to_goal = [];
                obj.avg_dist_to_goal = -1;
                return 
            end 
            obj.dist_to_goal = obj.l2_dist(x(:) - goal(1), y(:) - goal(2));
            obj.avg_dist_to_goal = mean(obj.dist_to_goal);
        end 
        
        function obj = calc_dist_to_opt_traj(obj, x, y, opt_x, opt_y)
            if length(x) ~= length(y) || length(opt_x) ~= length(opt_y)
                obj.dist_to_opt_traj = [];
                obj.avg_dist_to_opt_traj = -1;
                return 
            end
            opt_traj = [opt_x', opt_y'];
            obj.dist_to_opt_traj = zeros(length(x), 1)'; 
            for i = 1:length(x)
                state = [x(i), y(i)];
                dist_to_each_opt_traj_pt = sum((opt_traj - state) .^ 2, 2);
                min_dist = min(dist_to_each_opt_traj_pt);
                obj.dist_to_opt_traj(i) = min_dist;
            end 
            obj.avg_dist_to_opt_traj = mean(obj.dist_to_opt_traj);
        end 
        
        function obj = calc_safety_score(obj, x, y, ths, safety_planner)
            if length(x) ~= length(y) || length(y) ~= length(ths)
                obj.safety_score = [];
                obj.avg_safety_score = -1;
                return 
            end
            obj.safety_score = zeros(length(x), 1)';
            for i = 1:length(x)
                state = [x(i), y(i), ths(i)];
                obj.safety_score(i) = safety_planner.get_value(state); 
            end 
            obj.avg_safety_score = mean(obj.safety_score); 
        end
    end
end

