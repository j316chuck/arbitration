classdef PlannerBlender < handle
    %BLENDPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        exp 
        start 
        goal
        blending
        dt
        num_waypts
        horizon
        reach_avoid_planner
        brs_planner
        spline_planner
        exp_name
        output_folder
        state
        dynSys
        initial_traj
        blend_traj
        objective_str 
        scores
    end
    
    methods
        function obj = PlannerBlender(exp)
            %BLENDPLANNER Construct an instance of this class
            %   Detailed explanation goes here
            obj.exp = exp; 
            obj.start = exp.start;
            obj.goal = exp.goal;
            obj.dt = exp.dt; 
            obj.num_waypts = exp.num_waypts;
            obj.horizon = exp.horizon;
            obj.blending = exp.blending;
            obj.reach_avoid_planner = exp.reach_avoid_planner;
            obj.brs_planner = exp.brs_planner;
            obj.spline_planner = exp.spline_planner;
            start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
            goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
            obj.exp_name = sprintf("%s_map_%s_%s_blending_scheme_%s_%s", exp.map_basename, start_str, goal_str, exp.blending.scheme, exp.hyperparam_str); 
            obj.output_folder = sprintf("outputs/%s", obj.exp_name); 
            if exp.clear_dir && exist(obj.output_folder, 'dir')
                rmdir(obj.output_folder, 's');
            end 
            if ~exist(obj.output_folder, 'dir')
                mkdir(obj.output_folder); 
            end 
            
            if exp.run_planner 
                obj.initial_traj = obj.spline_planner.plan(exp.start);
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
        
        function blend_planners(obj)
            if isequal(obj.blending.scheme, 'switch')
                obj.switch_blend()
            elseif isequal(obj.blending.scheme, 'constant') 
                obj.constant_blend()
            elseif isequal(obj.blending.scheme, 'distance')
                obj.distance_blend()
            else
                warn("blending scheme not supported");  
                return
            end
            obj.plot_all();
        end 
        
        function switch_blend(obj)
            obj.dynSys = obj.exp.splineDynSys;
            obj.dynSys.x = obj.start(1:3);
            obj.state = [obj.start(1:3)', 0.01, 0]; %v = 0.01, w = 0;
            obj.blend_traj = zeros(6, 0); 
            use_avoid_control = 1;
            replan_time = 0;
            for i = 1:obj.num_waypts
                %updated_num_waypts = num_waypts - (i - 1); 
                %updated_horizon = horizon - (dt * (i - 1)); 
                if obj.brs_planner.get_value(obj.state(1:3)) <= obj.blending.zero_level_set
                    u = obj.brs_planner.get_avoid_u(obj.state(1:3))';
                    use_avoid_control = 1;
                else 
                    if replan_time >= obj.blending.replan_dt || use_avoid_control
                       replan_time = replan_time - obj.blending.replan_dt;
                       obj.spline_planner.set_spline_planning_points(obj.num_waypts, obj.horizon);
                       obj.spline_planner.plan(obj.state(1:4));
                    end 
                    u = obj.spline_planner.get_next_control();
                    use_avoid_control = 0; 
                end
                obj.blend_traj(:, i) = [obj.dynSys.x', u, use_avoid_control]'; % old state and new control
                obj.dynSys.updateState(u, obj.dt, obj.dynSys.x); 
                obj.state = [obj.dynSys.x', u]; % new state and new control
                replan_time = replan_time + obj.dt;
                if obj.exp.plot_every_iter 
                    obj.plot_all()
                end 
            end 
        end 
        
        function constant_blend(obj)
            obj.dynSys = obj.exp.splineDynSys;
            obj.dynSys.x = obj.start(1:3);
            obj.state = [obj.start(1:3)', 0.01, 0]; %v = 0.01, w = 0;
            obj.blend_traj = zeros(6, 0); 
            replan_time = 0;
            for i = 1:obj.num_waypts
                if replan_time >= obj.blending.replan_dt
                   replan_time = replan_time - obj.blending.replan_dt;
                   obj.spline_planner.set_spline_planning_points(obj.num_waypts, obj.horizon);
                   obj.spline_planner.plan(obj.state(1:4));
                end 
                u1 = obj.spline_planner.get_next_control();
                u2 = obj.brs_planner.get_avoid_u(obj.state(1:3))';
                u = obj.blending.alpha * u1 + (1 - obj.blending.alpha) * u2;
                obj.blend_traj(:, i) = [obj.dynSys.x', u, obj.blending.alpha]'; % old state and new control
                obj.dynSys.updateState(u, obj.dt, obj.dynSys.x); 
                obj.state = [obj.dynSys.x', u]; % new state and new control
                replan_time = replan_time + obj.dt;
                if obj.exp.plot_every_iter 
                    obj.plot_all()
                end 
            end 
        end 
        
        function distance_blend(obj)
            obj.dynSys = obj.exp.splineDynSys;
            obj.dynSys.x = obj.start(1:3);
            obj.state = [obj.start(1:3)', 0.01, 0]; %v = 0.01, w = 0;
            obj.blend_traj = zeros(6, 0); 
            replan_time = 0;
            for i = 1:obj.num_waypts
                if replan_time >= obj.blending.replan_dt
                   replan_time = replan_time - obj.blending.replan_dt;
                   obj.spline_planner.set_spline_planning_points(obj.num_waypts, obj.horizon);
                   obj.spline_planner.plan(obj.state(1:4));
                end 
                u1 = obj.spline_planner.get_next_control();
                u2 = obj.brs_planner.get_avoid_u(obj.state(1:3))';
                v = obj.brs_planner.get_value(obj.state(1:3));
                alpha = obj.sigmoid(v); 
                u = alpha * u1 + (1 - alpha) * u2;
                obj.blend_traj(:, i) = [obj.dynSys.x', u, alpha]'; % old state and new control
                obj.dynSys.updateState(u, obj.dt, obj.dynSys.x); 
                obj.state = [obj.dynSys.x', u]; % new state and new control
                replan_time = replan_time + obj.dt;
                if obj.exp.plot_every_iter 
                    obj.plot_all()
                end 
            end 
        end 
        
        function [prob] = sigmoid(obj, x)
            prob = 1 / (1 + exp(-x/obj.blending.temperature));
        end 
        
        function plot_all(obj)
            obj.plot_planners();
            obj.plot_auxillary();
        end 
        
        function plot_auxillary(obj)
            figure(10);
            clf;
            hold on 
            set(gcf,'Position', [10 10 800 900])
            subplot(3, 2, 1); 
            plot(1:length(obj.scores.vel), obj.scores.vel, 'bo--');
            title("Linear Velocity")
            xlabel("way point");
            ylabel("mps"); 
            subplot(3, 2, 2); 
            plot(1:length(obj.scores.accel), obj.scores.accel, 'bo--');
            title("Linear Accel")
            xlabel("way point");
            ylabel("mps"); 
            subplot(3, 2, 3); 
            plot(1:length(obj.scores.jerk), obj.scores.jerk, 'bo--');
            title("Linear Jerk")
            xlabel("way point");
            ylabel("mps"); 
            subplot(3, 2, 4); 
            plot(1:length(obj.scores.safety_score), obj.scores.safety_score, 'bo--');
            title("Safety Score")
            xlabel("way point");
            ylabel("brs value function"); 
            subplot(3, 2, 5); 
            plot(1:length(obj.scores.dist_to_opt_traj), obj.scores.dist_to_opt_traj, 'bo--');
            title("Dist to Opt Traj")
            xlabel("way point");
            ylabel("meters"); 
            subplot(3, 2, 6); 
            plot(1:length(obj.scores.dist_to_goal), obj.scores.dist_to_goal, 'bo--');
            title("Dist to Goal")
            xlabel("way point");
            ylabel("meters"); 
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
            opt_cur_spline = obj.spline_planner.opt_spline;
            mpc_spline_xs = opt_cur_spline{1}; 
            mpc_spline_ys = opt_cur_spline{2}; 
            mpc_spline_ths = opt_cur_spline{3}; 
            obj.plot_traj(mpc_spline_xs, mpc_spline_ys, mpc_spline_ths, 'red', 'mpc spline');    
            % plot original spline traj
            orig_spline_xs = obj.initial_traj{1};
            orig_spline_ys = obj.initial_traj{2};
            orig_spline_ths = obj.initial_traj{3};
            obj.plot_traj(orig_spline_xs, orig_spline_ys, orig_spline_ths, 'cyan', 'orig spline');
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
            %obj.plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');

            obj.scores = objectives();
            obj.scores.calc_kinematics(blend_xs, blend_ys, obj.dt); 
            obj.scores.calc_dist_to_goal(blend_xs, blend_ys, obj.goal); 
            obj.scores.calc_dist_to_opt_traj(blend_xs, blend_ys, reach_avoid_xs, reach_avoid_ys)
            obj.scores.calc_safety_score(blend_xs, blend_ys, blend_ths, obj.brs_planner); 
            % figure parameters
            xlim([0, 5]); %xlim([env.grid_2d.min(1),env.grid_2d.max(1)]);
            ylim([0, 5]); %ylim([env.grid_2d.min(2),env.grid_2d.max(2)]);
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
                savefigpath = sprintf("%s/plot.fig", obj.output_folder);
                savefig(savefigpath); 
            end 
            obj.plot_auxillary();

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

