
classdef Planner < handle
    %PLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        exp 
        start 
        goal
        stop_goal_dx
        max_num_planning_pts
        stop_on_collision
        cur_timestamp
        state
        blending
        dt
        replan_time_counter
        num_waypts
        horizon
        reach_avoid_planner
        brs_planner
        spline_planner
        exp_name
        output_folder
        dynSys
        orig_traj
        blend_traj
        objective_str 
        scores
        plot_level
    end
    
    methods
        function obj = Planner(exp)
            % Planner Construct an instance of this class
            %   Detailed explanation goes here
            obj.exp = exp; 
            obj.start = exp.start;
            obj.goal = exp.goal;
            obj.stop_goal_dx = exp.stop_goal_dx;
            obj.max_num_planning_pts = exp.max_num_planning_pts;
            obj.stop_on_collision = exp.stop_on_collision;
            obj.cur_timestamp = 1;
            obj.state = [obj.start(1:4); 0];
            obj.blending = exp.blending;
            obj.dt = exp.dt; 
            obj.replan_time_counter = obj.blending.replan_dt; %start of with a replan
            obj.num_waypts = exp.num_waypts;
            obj.horizon = exp.horizon;
            obj.reach_avoid_planner = exp.reach_avoid_planner;
            obj.brs_planner = exp.brs_planner;
            obj.spline_planner = exp.spline_planner;
            obj.orig_traj = [];
            start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
            goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
            obj.exp_name = sprintf("%s_map_%s_%s_blending_scheme_%s_%s", exp.map_basename, start_str, goal_str, exp.blending.scheme, exp.hyperparam_str); 
            obj.output_folder = sprintf("outputs/%s", obj.exp_name); 
            obj.plot_level = obj.exp.plot_level;
            
            if exp.clear_dir && exist(obj.output_folder, 'dir')
                rmdir(obj.output_folder, 's');
            end 
            if ~exist(obj.output_folder, 'dir')
                mkdir(obj.output_folder); 
            end 
            if exp.run_planner 
                obj.reach_avoid_planner.solve_reach_avoid(exp.start(1:3), exp.goal(1:3), exp.goal_map_3d, exp.obstacle, exp.dt); 
                obj.brs_planner.solve_brs_avoid(exp.obstacle);
            elseif exp.load_planner
                save_planner_file = sprintf("%s/run_planner.mat", obj.output_folder);
                load(save_planner_file, 'obj'); 
            else 
                warn("Planners not initialized"); 
                return
            end 
            if exp.save_planner
                save_planner_file = sprintf("%s/run_planner.mat", obj.output_folder);
                save(save_planner_file, 'obj');
            end 
        end
        
        function is_unsafe = is_unsafe_state(obj, value)
            is_unsafe = (value <= obj.blending.zero_level_set);
        end 
        
        function is_collision = collided_with_obstacle(obj)
            u = eval_u(obj.exp.grid_2d, obj.exp.binary_occ_map, obj.state(1:2));
            is_collision = u >= 0;
        end 
        
        function is_goal = reached_goal(obj)
            dist = obj.l2_dist(obj.state(1), obj.goal(1), obj.state(2), obj.goal(2));
            is_goal = dist <= obj.stop_goal_dx; 
        end 
        
        function d = l2_dist(x1, x2, y1, y2)
            d =  ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
        end 
        
        function verbose_plot(threshold)
            if obj.plot_level >= threshold
                obj.plot_planners();
                obj.plot_metrics();
            end 
        end 
        
        function blend_mpc_planning(obj) 
            while obj.cur_timestamp < obj.max_num_planning_pts
               if obj.replan_time_counter >= obj.blending.replan_dt
                  obj.replan_time_counter = 0;
                  plan = obj.spline_planner.plan(obj.state);
                  next_orig_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
                  obj.orig_traj = [obj.orig_traj, next_orig_traj];
                  obj.verbose_plot(2);
               end 
               switched_to_safety = false; 
               for i = 1:obj.num_waypts
                   x = obj.state(1:3);
                   v = obj.brs_planner.get_value(x);
                   u1 = [obj.orig_traj(4, obj.cur_timestamp), obj.orig_traj(5, obj.cur_timestamp)];
                   u2 = obj.brs_planner.get_avoid_u(x)';
                   if isequal(obj.blending.scheme, 'switch')
                       if switched_to_safety || obj.is_unsafe_state(v)
                           switched_to_safety = true; 
                           alpha = 0;
                       else 
                           alpha = 1;
                       end 
                   elseif isequal(obj.blending.scheme, 'constant') 
                       alpha = obj.blending.alpha;
                   elseif isequal(obj.blending.scheme, 'distance')
                       alpha = obj.blending.blend_function(v); 
                   else
                       warn("blending scheme not supported");  
                       return
                   end
                   assert(0 <= alpha && alpha <= 1);
                   u = alpha * u1 + (1 - alpha) * u2;
                   obj.blend_traj(:, obj.cur_timestamp) = [x, u, alpha]'; % old state and new control
                   obj.dynSys.updateState(u, obj.dt, x); 
                   obj.state = [obj.dynSys.x', u]; % new state and new control
                   obj.cur_timestamp = obj.cur_timestamp + 1;
                   obj.replan_time_counter = obj.replan_time_counter + obj.dt;
                   obj.verbose_plot(3);
                   if obj.collided_with_obstacle(obj.state) || obj.reached_goal(obj.state)
                       obj.verbose_plot(1);
                       return 
                   end 
               end 
            end 
            obj.verbose_plot(1);
        end
        
        function plot_metrics(obj)
            obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
            figure(10);
            clf;
            hold on 
            set(gcf,'Position', [10 10 800 900])
            subplot(4, 2, 1); 
            plot(1:length(obj.scores.vel), obj.scores.vel, 'bo--');
            title("Linear Velocity");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(4, 2, 2); 
            plot(1:length(obj.blend_traj(5, :)), obj.blend_traj(5, :), 'bo--');
            title("Angular Velocity");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(4, 2, 3); 
            plot(1:length(obj.scores.accel), obj.scores.accel, 'bo--');
            title("Linear Accel");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(4, 2, 4); 
            plot(1:length(obj.scores.jerk), obj.scores.jerk, 'bo--');
            title("Linear Jerk");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(4, 2, 5); 
            plot(1:length(obj.scores.safety_score), obj.scores.safety_score, 'bo--');
            title("Safety Score");
            xlabel("iteration");
            ylabel("brs value function"); 
            subplot(4, 2, 6); 
            plot(1:length(obj.scores.dist_to_opt_traj), obj.scores.dist_to_opt_traj, 'bo--');
            title("Dist to Opt Traj");
            xlabel("iteration");
            ylabel("meters"); 
            subplot(4, 2, 7); 
            plot(1:length(obj.scores.dist_to_goal), obj.scores.dist_to_goal, 'bo--');
            title("Dist to Goal");
            xlabel("iteration");
            ylabel("meters"); 
            subplot(4, 2, 8); 
            plot(1:length(obj.blend_traj(6, :)), obj.blend_traj(6, :), 'bo--');
            title("Blend Probability");
            xlabel("iteration");
            ylabel("alpha");
            if obj.exp.save_plot
                savefigpath = sprintf("%s/metrics.fig", obj.output_folder);
                savefig(savefigpath); 
            end 
        end 
        
        function plot_planners(obj)
            figure(5);
            clf;
            hold on 
            set(gcf,'Position',[10 10 1000 800])
            % plot environment, goal, and start
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal shape', 'color', 'red');
            contourf(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'obstacle');
            scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
            scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
            % plot value functions
            name = sprintf("BRS (theta = %s)", obj.state(3));
            [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], obj.state(3));
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, 'DisplayName', name, 'color', '#CC1FCB');
            % plot mpc spline traj
            mpc_spline_xs = obj.orig_traj(1, :); 
            mpc_spline_ys = obj.orig_traj(2, :); 
            mpc_spline_ths = obj.orig_traj(3, :); 
            obj.plot_traj(mpc_spline_xs, mpc_spline_ys, mpc_spline_ths, 'red', 'mpc spline');    
            % plot blending traj
            blend_xs = obj.blend_traj(1, :); 
            blend_ys = obj.blend_traj(2, :); 
            blend_ths = obj.blend_traj(3, :); 
            use_avoid_probs = obj.blend_traj(6, :);
            blend_name = obj.blending.scheme;
            obj.plot_traj_probs(blend_xs, blend_ys, blend_ths, use_avoid_probs, blend_name);
            % plot reach avoid traj
            reach_avoid_xs = obj.reach_avoid_planner.opt_traj(1, :);
            reach_avoid_ys = obj.reach_avoid_planner.opt_traj(2, :);
            reach_avoid_ths = obj.reach_avoid_planner.opt_traj(3, :);
            obj.plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');
            % calculate scores
            obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
            % figure parameters
            gap = 2.5;
            xlim([min(blend_xs(:)) - gap, max(blend_xs(:)) + gap]); %xlim([env.grid_2d.min(1),env.grid_2d.max(1)]);
            ylim([min(blend_ys(:)) - gap, max(blend_ys(:)) + gap]); %ylim([env.grid_2d.min(2),env.grid_2d.max(2)]);
            view(0, 90)
            set(gcf, 'color', 'white')
            set(gcf, 'position', [0, 0, 800, 800])
            xlabel('x (meters)');
            ylabel('y (meters)');
            legend('Location', 'NorthWest');
            title(obj.exp_name, 'Interpreter', 'None');
            ajs = sprintf("avg_jerk: %.5f", obj.scores.avg_jerk);
            adgs = sprintf("avg_dist_to_goal: %.5f", obj.scores.avg_dist_to_goal); 
            ash = sprintf("avg_safety_score: %.5f", obj.scores.avg_safety_score); 
            adot = sprintf("avg_dist_to_opt_traj: %.5f", obj.scores.avg_dist_to_opt_traj); 
            obj.objective_str = sprintf('%s\n%s\n%s\n%s', ajs, adgs, ash, adot); 
            annotation('textbox',[.7 .90 1 .0], 'String', obj.objective_str,'FitBoxToText','on', 'Interpreter', 'None');
            if obj.exp.save_plot
                savefigpath = sprintf("%s/planners.fig", obj.output_folder);
                savefig(savefigpath); 
            end 
        end 
        
        function plot_traj(obj, xs, ys, ths, color, name)
            s = scatter(xs, ys, 15, 'black', 'filled', 'DisplayName', name);
            s.HandleVisibility = 'off';
            q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
            q.DisplayName = name;
            q.HandleVisibility = 'on';
            q.ShowArrowHead = 'on';
            q.AutoScale = 'on';
            q.AutoScaleFactor = 0.3;
        end

        function plot_traj_probs(obj, xs, ys, ths, probs, name)
            s = scatter(xs, ys, 15, 'black', 'filled');
            s.HandleVisibility = 'off';
            q = quiver(xs, ys, cos(ths), sin(ths));
            obj.set_quiver_colors(q, probs); 
            q.DisplayName = name;
            q.HandleVisibility = 'on';
            q.ShowArrowHead = 'on';
            q.AutoScale = 'on';
            q.DisplayName = name;
            q.AutoScaleFactor = 0.3;
        end
        
        function set_quiver_colors(obj, q, probs)
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
    end
end

