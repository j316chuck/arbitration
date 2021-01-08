
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
        dynSys
        exp_name
        output_folder
        plot_level
        orig_traj
        safety_traj
        blend_traj
        objective_str 
        scores
        termination_state
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
            obj.state = [obj.start(1:4)', 0];
            obj.blending = exp.blending;
            obj.dt = exp.dt; 
            obj.replan_time_counter = obj.blending.replan_dt; %start of with a replan
            obj.num_waypts = exp.num_waypts;
            obj.horizon = exp.horizon;
            obj.reach_avoid_planner = exp.reach_avoid_planner;
            obj.brs_planner = exp.brs_planner;
            obj.spline_planner = exp.spline_planner;
            obj.dynSys = obj.spline_planner.dynSys;
            start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
            goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
            obj.exp_name = sprintf("%s_map_%s_%s_blending_scheme_%s_%s", exp.map_basename, start_str, goal_str, exp.blending.scheme, exp.hyperparam_str); 
            obj.output_folder = sprintf("outputs/%s", obj.exp_name); 
            obj.plot_level = obj.exp.plot_level;
            obj.orig_traj = zeros(0, 5);
            obj.blend_traj = [];
            
            if exp.clear_dir && exist(obj.output_folder, 'dir')
                rmdir(obj.output_folder, 's');
            end 
            if ~exist(obj.output_folder, 'dir')
                mkdir(obj.output_folder); 
            end
            
            if exp.run_brs
                obj.brs_planner.solve_brs_avoid(exp.obstacle);
                brs_planner = obj.brs_planner;
                save('./data/brs_planner.mat', 'brs_planner'); 
            else
                load('./data/brs_planner.mat'); 
                obj.brs_planner = brs_planner;
            end 
            
            if exp.run_planner
                obj.reach_avoid_planner.solve_reach_avoid(exp.start(1:3), exp.goal(1:3), exp.goal_map_3d, exp.obstacle, exp.dt); 
            else
                save_planner_file = sprintf("%s/run_planner.mat", obj.output_folder);
                load(save_planner_file, 'obj'); 
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
            is_collision = (u >= 0);
        end 
        
        function is_goal = reached_goal(obj)
            dist = obj.l2_dist(obj.state(1), obj.goal(1), obj.state(2), obj.goal(2));
            is_goal = (dist <= obj.stop_goal_dx); 
        end 
        
        function is_max_timestamps = reached_max_timestamps(obj)
            is_max_timestamps = (obj.cur_timestamp >= obj.max_num_planning_pts);
        end 
        
        function d = l2_dist(obj, x1, x2, y1, y2)
            d = ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
        end 
        
        function verbose_plot(obj, threshold)
            if obj.plot_level >= threshold
                obj.plot_planners();
                obj.plot_metrics(); 
            end 
        end 
        
%         
%         function blend_mpc_controls_replan(obj) 
%             obj.dynSys = obj.spline_planner.dynSys; % temporary hack to fix the dynSys not working
%             while 1 
%                % finish mpc trajectory condition
%                if obj.reached_max_timestamps() || obj.reached_goal() || obj.collided_with_obstacle()
%                   obj.verbose_plot(1);
%                   obj.save_state();
%                   return;
%                end 
%                % replan condition
%                if obj.replan_time_counter >= obj.blending.replan_dt 
%                   obj.replan_time_counter = 0;
%                   plan = obj.spline_planner.plan(obj.state); 
%                   
%                   if ~isempty(plan)
%                       next_orig_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
%                       obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), next_orig_traj];
%                       old_x = obj.dynSys.x; 
%                       num_timestamps = obj.replan_dt/obj.dt; 
%                       for i=1:num_timestamps 
%                           x = reshape(next_orig_traj(i, 1:3), [1, 3]);
%                           u = obj.brs_planner.get_avoid_u(x); 
%                           obj.dynSys.x = x;
%                           obj.dynSys.updateState(u, obj.dt, x); 
%                           state = [obj.dynSys.x', u];
%                           new_plan = obj.spline_planner.plan(obj.state); 
%                           
%                       end 
%                       obj.dynSys.x = old_x;
%                   end %TODO add if condition for not enough steps in orig_traj
%                   obj.verbose_plot(2);
%                end 
%                % get control per timestamp
%                x = reshape(obj.state(1:3), [1, 3]);
%                v = obj.brs_planner.get_value(x);
%                u1 = [obj.orig_traj(4, obj.cur_timestamp), obj.orig_traj(5, obj.cur_timestamp)];
%                u2 = obj.brs_planner.get_avoid_u(x)';
%                if obj.next_traj_safety_scores_decreasing()
%                    alpha = 0; 
%                    obj.replan_time_counter = obj.blending.replan_dt;
%                else 
%                    alpha = 1; 
%                end 
%                assert(0 <= alpha && alpha <= 1);
%                u = alpha * u1 + (1 - alpha) * u2;
%                % update state
%                obj.blend_traj(:, obj.cur_timestamp) = [x, u, alpha]'; % old state and new control
%                obj.dynSys.updateState(u, obj.dt, x'); 
%                obj.state = [obj.dynSys.x', u]; % new state and new control
%                obj.cur_timestamp = obj.cur_timestamp + 1;
%                obj.replan_time_counter = obj.replan_time_counter + obj.dt;
%             end 
%         end
        
        function blend_mpc_controls(obj) 
            obj.dynSys = obj.spline_planner.dynSys; % temporary hack to fix the dynSys not working
            while 1 
               % finish mpc trajectory condition
               if obj.reached_max_timestamps() || obj.reached_goal() || obj.collided_with_obstacle()
                  obj.verbose_plot(1);
                  obj.save_state();
                  return;
               end 
               % replan condition
               if obj.replan_time_counter >= obj.blending.replan_dt
                  obj.replan_time_counter = 0;
                  switched_to_safety = false; 
                  plan = obj.spline_planner.plan(obj.state); 
                  if ~isempty(plan)
                      next_orig_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
                      obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), next_orig_traj];
                  end %TODO add if condition for not enough steps in orig_traj
                  obj.verbose_plot(2);
               end 
               % get control per timestamp
               x = reshape(obj.state(1:3), [1, 3]);
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
                   warning("blending scheme not supported");  
                   return
               end
               assert(0 <= alpha && alpha <= 1);
               u = alpha * u1 + (1 - alpha) * u2;
               % update state
               obj.blend_traj(:, obj.cur_timestamp) = [x, u, alpha]'; % old state and new control
               obj.dynSys.updateState(u, obj.dt, x'); 
               obj.state = [obj.dynSys.x', u]; % new state and new control
               obj.cur_timestamp = obj.cur_timestamp + 1;
               obj.replan_time_counter = obj.replan_time_counter + obj.dt;
            end 
        end
        
        function blend_mpc_traj(obj) 
            obj.dynSys = obj.spline_planner.dynSys; % temporary hack to fix the dynSys not working
            while 1 
               % finish mpc trajectory condition
               if obj.reached_max_timestamps() || obj.reached_goal() || obj.collided_with_obstacle()
                  obj.verbose_plot(1);
                  obj.save_state();
                  return;
               end 
               % replan condition
               if obj.replan_time_counter >= obj.blending.replan_dt
                  obj.replan_time_counter = 0;
                  plan = obj.spline_planner.plan(obj.state); 
                  if ~isempty(plan)
                      next_orig_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
                      obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), next_orig_traj];
                      next_safety_traj = obj.get_next_safety_traj();
                      obj.safety_traj = [obj.safety_traj(:, 1:obj.cur_timestamp-1), next_safety_traj];
                      if strcmp(obj.exp.blending.scheme, 'blend_safety_value_traj') 
                          alpha = obj.blending.alpha;
                          new_plan = obj.spline_planner.replan_with_brs_planner(obj.state, plan{1}, plan{2}, obj.brs_planner, alpha);
                          %new_plan = obj.truncate_plan_with_replan_time(new_plan);
                          blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                          next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      elseif strcmp(obj.exp.blending.scheme, 'blend_safety_control_traj')
                          alpha = obj.blending.alpha;
                          new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, plan{1}, plan{2}, next_safety_traj(1, :), next_safety_traj(2, :), alpha);    
                          %new_plan = obj.truncate_plan_with_replan_time(new_plan);
                          blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                          next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      elseif strcmp(obj.exp.blending.scheme, 'probabilistic_blend_safety_value_traj')
                          alphas = flip([0; sort(rand(obj.exp.blending.num_alpha_samples, 1)); 1.0]);
                          blended_traj = false;
                          for i = 1:length(alphas)
                              alpha = alphas(i); 
                              new_plan = obj.spline_planner.replan_with_brs_planner(obj.state, plan{1}, plan{2}, obj.brs_planner, alpha);
                              trucated_new_plan = obj.truncate_plan_with_replan_time(new_plan);
                              if obj.is_safe_traj(trucated_new_plan)
                                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                                  next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                                  blended_traj = true;
                                  break
                              end
                          end
                          if ~blended_traj
                              obj.use_modified_safe_orig_traj_in_next_mpc_plan(next_orig_traj);
                              %obj.use_safety_traj_in_next_mpc_plan(next_safety_traj)
                          end 
                      elseif strcmp(obj.exp.blending.scheme, 'probabilistic_blend_safety_control_traj')
                          alphas = flip([0; sort(rand(obj.exp.blending.num_alpha_samples, 1)); 1.0]);
                          blended_traj = false;
                          for i = 1:length(alphas)
                              alpha = alphas(i); 
                              new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, plan{1}, plan{2}, next_safety_traj(1, :), next_safety_traj(2, :), alpha);    
                              trucated_new_plan = obj.truncate_plan_with_replan_time(new_plan);
                              if obj.is_safe_traj(trucated_new_plan)
                                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                                  next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                                  blended_traj = true;
                                  break
                              end 
                          end             
                          if ~blended_traj
                              obj.use_modified_safe_orig_traj_in_next_mpc_plan(next_orig_traj);
                              %obj.use_safety_traj_in_next_mpc_plan(next_safety_traj)
                          end 
                      end 
                      obj.replan_plot(2);
                      obj.verbose_plot(2);
                  end
               end 
               % update state
               x = reshape(obj.state(1:3), [1, 3]);
               u = [obj.blend_traj(4, obj.cur_timestamp), obj.blend_traj(5, obj.cur_timestamp)]; 
               a = obj.blend_traj(6, obj.cur_timestamp);
               obj.blend_traj(:, obj.cur_timestamp) = [x, u, a]'; % old state and new control
               obj.dynSys.updateState(u, obj.dt, x'); 
               obj.state = [obj.dynSys.x', u]; % new state and new control
               obj.cur_timestamp = obj.cur_timestamp + 1;
               obj.replan_time_counter = obj.replan_time_counter + obj.dt;
            end 
        end
       
        function new_plan = truncate_plan_with_replan_time(obj, plan)
            num_steps_per_mpc_replan = ceil(obj.blending.replan_dt / obj.dt); 
            if iscell(plan)
                new_plan = {plan{1}(1:num_steps_per_mpc_replan), plan{2}(1:num_steps_per_mpc_replan), plan{3}(1:num_steps_per_mpc_replan), plan{4}(1:num_steps_per_mpc_replan), plan{5}(1:num_steps_per_mpc_replan)};
            else
                new_plan = plan(:, 1:num_steps_per_mpc_replan);
            end 
        end 
        
        function use_safety_traj_in_next_mpc_plan(obj, next_safety_traj)
            alpha = -0.1; 
            blend_alpha = (ones(size(next_safety_traj, 2), 1) * alpha)';
            next_blend_traj = [next_safety_traj; blend_alpha];
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
        end 
        
                
        function use_modified_safe_orig_traj_in_next_mpc_plan(obj, orig_safety_traj)
            original_x = obj.dynSys.x; % copy state 
            safe_orig_traj = zeros(6, 0);
            safety_state = obj.state; 
            used_safety = false;            
            for i = 1:length(orig_safety_traj)
              x = reshape(safety_state(1:3), [1, 3]);
              u = [orig_safety_traj(4, i), orig_safety_traj(5, i)];
              a = 1;
              if used_safety || (obj.brs_planner.get_value(x) < obj.blending.zero_level_set)
                  used_safety = true; 
                  u = reshape(obj.brs_planner.get_avoid_u(x), [1, 2]);
                  a = -0.2;
              end 
              obj.dynSys.updateState(u, obj.dt, x'); 
              safe_orig_traj(:, i) = [x, u, a]; %old state new control
              safety_state = [obj.dynSys.x', u]; % new state and new control
            end 
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), safe_orig_traj]; 
            obj.dynSys.x = original_x; % restore state
        end 
        
        function is_safe = is_safe_traj(obj, traj)
            xs = traj{1}; ys = traj{2}; ths = traj{3};
            for i=1:length(xs)
                pos = [xs(i), ys(i), ths(i)];
                if obj.brs_planner.get_value(pos) < obj.blending.zero_level_set
                    is_safe = false; 
                    return 
                end 
            end 
            is_safe = true;                           
        end 
        
        function safety_traj = get_next_safety_traj(obj)
            original_x = obj.dynSys.x; % copy state 
            safety_traj = zeros(5, 0);
            safety_state = obj.state; 
            for i = 1:obj.num_waypts
              x = reshape(safety_state(1:3), [1, 3]);
              u = obj.brs_planner.get_avoid_u(x)';
              obj.dynSys.updateState(u, obj.dt, x'); 
              safety_traj(:, i) = [x, u]; %old state new control
              safety_state = [obj.dynSys.x', u]; % new state and new control
            end 
            obj.dynSys.x = original_x; % restore state
        end                    
        
        function save_state(obj)
            if obj.reached_goal() 
                obj.termination_state = 0;
            elseif obj.collided_with_obstacle()
                obj.termination_state = 1;
            elseif obj.reached_max_timestamps()
                obj.termination_state = 2;
            else
                obj.termination_state = 3;
            end 
            save_planner_file = sprintf("%s/final_state.mat", obj.output_folder);
            save(save_planner_file, 'obj');
        end 
        
        function replan_plot(obj, verbosity)
            if obj.plot_level < verbosity 
                return 
            end 
            savefigpath = '';
            if obj.exp.save_plot
                savefigpath = sprintf("%s/replan_traj_timestamp_%d.fig", obj.output_folder, obj.cur_timestamp);  
            end 
            obj.spline_planner.plot_replan_scores(savefigpath);
        end 
        
        function plot_metrics(obj)
            if isempty(obj.blend_traj)
                return
            end 
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
            % plot safety_traj
            if ~isempty(obj.safety_traj)
                safety_spline_xs = obj.safety_traj(1, :); 
                safety_spline_ys = obj.safety_traj(2, :); 
                safety_spline_ths = obj.safety_traj(3, :); 
                obj.plot_traj(safety_spline_xs, safety_spline_ys, safety_spline_ths, 'magenta', 'safety traj');    
            end 
            
            % plot blending traj
            if ~isempty(obj.blend_traj)
                blend_xs = obj.blend_traj(1, :); 
                blend_ys = obj.blend_traj(2, :); 
                blend_ths = obj.blend_traj(3, :); 
                use_avoid_probs = obj.blend_traj(6, :);
                blend_name = obj.blending.scheme;
                obj.plot_traj_probs(blend_xs, blend_ys, blend_ths, use_avoid_probs, blend_name);
            end 
            % plot reach avoid traj
            reach_avoid_xs = obj.reach_avoid_planner.opt_traj(1, :);
            reach_avoid_ys = obj.reach_avoid_planner.opt_traj(2, :);
            reach_avoid_ths = obj.reach_avoid_planner.opt_traj(3, :);
            obj.plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');
            % calculate scores
            if ~isempty(obj.blend_traj)
                obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
            end 
            % figure parameters
            view(0, 90)
            set(gcf, 'color', 'white')
            set(gcf, 'position', [0, 0, 800, 800])
            xlabel('x (meters)');
            ylabel('y (meters)');
            l = legend('Location', 'NorthWest');
            set(l, 'Interpreter', 'none')
            title(obj.exp_name, 'Interpreter', 'None');
            if ~isempty(obj.scores) 
                ajs = sprintf("avg_jerk: %.5f", obj.scores.avg_jerk);
                adgs = sprintf("avg_dist_to_goal: %.5f", obj.scores.avg_dist_to_goal); 
                ash = sprintf("avg_safety_score: %.5f", obj.scores.avg_safety_score); 
                adot = sprintf("avg_dist_to_opt_traj: %.5f", obj.scores.avg_dist_to_opt_traj); 
                obj.objective_str = sprintf('%s\n%s\n%s\n%s', ajs, adgs, ash, adot); 
                annotation('textbox',[.7 .90 1 .0], 'String', obj.objective_str,'FitBoxToText','on', 'Interpreter', 'None');
            end 
            if obj.exp.save_plot
                savefigpath = sprintf("%s/planners_%d.fig", obj.output_folder, obj.cur_timestamp);
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

