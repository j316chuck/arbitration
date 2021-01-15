
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
        num_mpc_steps
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
        switch_traj
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
            obj.num_mpc_steps = ceil(obj.blending.replan_dt / obj.dt); % num steps we take in a mpc 
            obj.replan_time_counter = obj.blending.replan_dt; %start of with a replan
            obj.num_waypts = exp.num_waypts;
            obj.horizon = exp.horizon;
            obj.reach_avoid_planner = exp.reach_avoid_planner;
            obj.brs_planner = exp.brs_planner;
            obj.spline_planner = exp.spline_planner;
            obj.dynSys = obj.spline_planner.dynSys;
            start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
            goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
            point_nav_str = sprintf("%s_map_%s_%s", exp.map_basename, start_str, goal_str); 
            obj.exp_name = sprintf("%s_blending_scheme_%s_%s", point_nav_str, exp.blending.scheme, exp.hyperparam_str); 
            
            repo = what('arbitration');
            obj.output_folder = strcat(repo.path, "/outputs/", obj.exp_name);
            obj.plot_level = obj.exp.plot_level;
            obj.orig_traj = zeros(0, 5);
            obj.blend_traj = [];
            
            if exp.clear_dir && exist(obj.output_folder, 'dir')
                rmdir(obj.output_folder, 's');
            end 
            if ~exist(obj.output_folder, 'dir')
                mkdir(obj.output_folder); 
            end
            
            repo = what('arbitration');
            if exp.run_brs
                obj.brs_planner.solve_brs_avoid(exp.obstacle);
                brs_planner = obj.brs_planner;
                filename = strcat(repo.path, '/data/brs_planner.mat');
                save(filename, 'brs_planner'); 
            else
                filename = strcat(repo.path, '/data/brs_planner.mat');
                load(filename, 'brs_planner'); 
                obj.brs_planner = brs_planner;
            end 
            
            if exp.run_planner
                obj.reach_avoid_planner.solve_reach_avoid(exp.start(1:3), exp.goal(1:3), exp.goal_map_3d, exp.obstacle, exp.dt);
                reach_avoid_planner = obj.reach_avoid_planner;
                filename = sprintf("%s/data/%s.mat", repo.path, point_nav_str); 
                save(filename, 'reach_avoid_planner'); 
            else
                filename =  sprintf("%s/data/%s.mat", repo.path, point_nav_str); 
                load(filename, 'reach_avoid_planner');
                obj.reach_avoid_planner = reach_avoid_planner;
            end 
        end
        
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
                  %obj.plot_spline_cost(2); %add for spline planner visualization debugging
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
                          blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                          next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      elseif strcmp(obj.exp.blending.scheme, 'blend_safety_control_traj')
                          alpha = obj.blending.alpha;
                          new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, plan{1}, plan{2}, next_safety_traj(1, :), next_safety_traj(2, :), alpha);    
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
                              use_zero_alpha_blending = (i == length(alphas) && ~obj.exp.blending.use_safe_orig_traj);
                              if obj.is_safe_traj(trucated_new_plan) || use_zero_alpha_blending
                                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                                  next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                                  blended_traj = true;
                                  break
                              end
                          end
                          if ~blended_traj
                              obj.use_modified_safe_orig_traj_in_next_mpc_plan(next_orig_traj);
                          end 
                      elseif strcmp(obj.exp.blending.scheme, 'probabilistic_blend_safety_control_traj')
                          alphas = flip([0; sort(rand(obj.exp.blending.num_alpha_samples, 1)); 1.0]);
                          blended_traj = false;
                          for i = 1:length(alphas)
                              alpha = alphas(i); 
                              new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, plan{1}, plan{2}, next_safety_traj(1, :), next_safety_traj(2, :), alpha);    
                              trucated_new_plan = obj.truncate_plan_with_replan_time(new_plan);
                              use_zero_alpha_blending = (i == length(alphas) && ~obj.exp.blending.use_safe_orig_traj);
                              if obj.is_safe_traj(trucated_new_plan) || use_zero_alpha_blending
                                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                                  next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                                  blended_traj = true;
                                  break
                              end 
                          end             
                          if ~blended_traj
                              obj.use_modified_safe_orig_traj_in_next_mpc_plan(next_orig_traj);
                          end 
                      elseif strcmp(obj.exp.blending.scheme, 'mean_value_blend_safety_value_traj') 
                          avg_safety_score = mean(obj.get_mpc_traj_safety_scores(plan)); 
                          alpha = max(min(avg_safety_score, 1), 0); 
                          fprintf("Alpha chosen: %f, safety score %f\n", alpha, avg_safety_score); 
                          new_plan = obj.spline_planner.replan_with_brs_planner(obj.state, plan{1}, plan{2}, obj.brs_planner, alpha);
                          blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                          next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      elseif strcmp(obj.exp.blending.scheme, 'mean_value_blend_safety_control_traj')
                          avg_safety_score = mean(obj.get_mpc_traj_safety_scores(plan)); 
                          alpha = max(min(avg_safety_score, 1), 0); 
                          fprintf("Alpha chosen: %f, safety score %f\n", alpha, avg_safety_score); 
                          new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, plan{1}, plan{2}, next_safety_traj(1, :), next_safety_traj(2, :), alpha);    
                          blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                          next_blend_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      elseif strcmp(obj.exp.blending.scheme, 'time_varying_value_blend_safety_control_traj')
                          [new_plan, new_alphas] = obj.spline_planner.replan_with_value_blending(obj.state, ...
                                                        plan{1}, ...
                                                        plan{2}, ...
                                                        next_safety_traj(1, :), ...
                                                        next_safety_traj(2, :), ...
                                                        obj.brs_planner);
                          % ========== DEBUGGING! =========== %                          
                          obj.plot_value_blended_traj(plan{1}, ...
                                                        plan{2}, ...
                                                        next_safety_traj(1, :), ...
                                                        next_safety_traj(2, :), ...
                                                        new_plan, new_alphas, ...
                                                        (obj.cur_timestamp-1)*obj.dt);
                          % ========== DEBUGGING! =========== %   
                                                    
                          next_blend_traj = [new_plan{1}; new_plan{2}; ...
                                                new_plan{3}; new_plan{4}; ...
                                                new_plan{5}; new_alphas'];  
                          obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_blend_traj]; 
                      end 
                      obj.plot_spline_replan(2);
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
                
        function safety_scores = get_mpc_traj_safety_scores(obj, plan)
            safety_scores = zeros(obj.num_mpc_steps, 1); 
            for i = 1:obj.num_mpc_steps
                x = [plan{1}(i), plan{2}(i), plan{3}(i)]; 
                safety_scores(i) = obj.brs_planner.get_value(x); 
            end 
        end 
        
        function [opt_traj] = replan_after_every_k_timestamps_with_safety_controls(obj, plan)
              if isempty(plan)
                  opt_traj = plan;
                  return; 
              end
              old_x = obj.dynSys.x; % save state
              opt_traj = plan;
              opt_planner_cost = obj.spline_planner.eval_cost(plan); 
              opt_safety_cost = obj.spline_planner.eval_traj_safety_cost(plan, obj.brs_planner);
              next_orig_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
              Nt = length(plan{1}); 
              obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), next_orig_traj];
              interval = 5;
              num_safety_controls = 3; 
              for i=1:interval:obj.num_mpc_steps
                  % move along orig traj
                  st = obj.cur_timestamp; %16
                  et = obj.cur_timestamp + i - 2; %16 + 6 - 2 = 20
                  new_traj = obj.orig_traj(:, st:et); %16-20
                  obj.dynSys.x = obj.orig_traj(:, et+1); %21
                  % apply safety controls
                  safety_counter = 0; 
                  while safety_counter < num_safety_controls
                      x = reshape(obj.dynSys.x, [1, 3]); 
                      u = obj.brs_planner.get_avoid_u(x); 
                      new_traj(:, end) = [x, u]; 
                      obj.dynSys.updateState(u, obj.dt, x');
                      safety_counter = safety_counter + 1; 
                  end 
                  % replan after applying safety controls
                  new_state = obj.dynSys.x; 
                  new_plan = obj.spline_planner.plan(new_state);
                  replan_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}];
                  new_traj = [new_traj, replan_traj];
                  new_traj = new_traj(1:Nt, :); 
                  planner_cost = obj.spline_planner.eval_cost(new_traj); 
                  safety_cost = obj.spline_planner.eval_traj_safety_cost(new_traj, obj.brs_planner); 
                  if safety_cost < opt_safety_cost && planner_cost < opt_planner_cost
                      opt_planner_cost = planner_cost; 
                      opt_safety_cost = safety_cost; 
                      opt_traj = new_traj; 
                  end 
              end
              opt_traj(6, :) = 1; % blend_alpha
              obj.blend_traj = opt_traj; % set blend trajectory
              obj.dynSys.x = old_x; % restore state
        end 
        
        function [chosen_traj] = replan_waypoint(obj, plan)
              if isempty(plan) 
                  return; 
              end
              chosen_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
              obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
              mpc_plan = obj.truncate_plan_with_replan_time(plan);
              if obj.is_safe_traj(mpc_plan)
                  chosen_traj(6, :) = 1; %blend alpha
                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
                  return; 
              end 
              old_x = obj.dynSys.x; % save state
              replan_level_set = obj.blending.replan_level_set;
              % use original mpc planner until it's unsafe
              mpc_traj = [mpc_plan{1}; mpc_plan{2}; mpc_plan{3}; mpc_plan{4}; mpc_plan{5}];
              ui = obj.get_first_unsafe_index_in_traj(mpc_plan);
              switch_blend_traj = mpc_traj(:, 1:ui-1); 
              obj.dynSys.x = mpc_traj(1:3, ui); 
              safety_vertex = obj.dynSys.x(1:3); 
              % apply zero levelset controls until safety score reaches a threshold
              while 1 
                  x = reshape(obj.dynSys.x, [1, 3]); 
                  safety_score = obj.brs_planner.get_value(x');
                  reached_safe_point = (safety_score >= replan_level_set);
                  if reached_safe_point
                      break 
                  end 
                  u = obj.brs_planner.get_avoid_u(x)'; 
                  switch_blend_traj(:, end+1) = [x, u]; 
                  obj.dynSys.updateState(u, obj.dt, x');
              end 
              intermediate_waypt_vertex = [obj.dynSys.x', u];
              % replan through intermediate_waypoint after applying safety controls
              max_num_candidates = obj.exp.blending.replan_max_num_candidates;
              new_plan = obj.spline_planner.adaptive_replan_through_waypoint(obj.state, intermediate_waypt_vertex, max_num_candidates);
              % obj.plot_triangular_traj(switch_blend_traj, safety_vertex, intermediate_waypt_vertex); % debugging plots
              % obj.plot_spline_cost(2); % debugging plots
              if isempty(new_plan)
                chosen_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
              else 
                chosen_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}];                  
              end
              chosen_traj(6, :) = 1; % blend_alpha
              % store info
              obj.switch_traj = [obj.switch_traj, switch_blend_traj];
              obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
              obj.dynSys.x = old_x; % restore state
        end
            
                
        function [chosen_traj] = adaptive_replan_waypoint(obj, plan)
              if isempty(plan) 
                  return; 
              end
              chosen_traj = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
              obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
              mpc_plan = obj.truncate_plan_with_replan_time(plan);
              if obj.is_safe_traj(mpc_plan)
                  chosen_traj(6, :) = 1; %blend alpha
                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
                  return; 
              end 
              old_x = obj.dynSys.x; % save state
              old_chosen_traj = []; 
              replan_level_set = obj.blending.replan_level_set;
              while 1
                  % use original mpc planner until it's unsafe
                  mpc_traj = [mpc_plan{1}; mpc_plan{2}; mpc_plan{3}; mpc_plan{4}; mpc_plan{5}];
                  ui = obj.get_first_unsafe_index_in_traj(mpc_plan);
                  switch_blend_traj = mpc_traj(:, 1:ui-1); 
                  obj.dynSys.x = mpc_traj(1:3, ui); 
                  safety_vertex = obj.dynSys.x(1:3); 
                  % apply zero levelset controls until safety score reaches a threshold
                  while 1 
                      x = reshape(obj.dynSys.x, [1, 3]); 
                      safety_score = obj.brs_planner.get_value(x');
                      reached_safe_point = (safety_score >= replan_level_set);
                      if reached_safe_point
                          break 
                      end 
                      u = obj.brs_planner.get_avoid_u(x)'; 
                      switch_blend_traj(:, end+1) = [x, u]; 
                      obj.dynSys.updateState(u, obj.dt, x');
                  end 
                  intermediate_waypt_vertex = [obj.dynSys.x', u];
                  % replan through intermediate_waypoint after applying safety controls
                  max_num_candidates = obj.exp.blending.replan_max_num_candidates;
                  new_plan = obj.spline_planner.adaptive_replan_through_waypoint(obj.state, intermediate_waypt_vertex, max_num_candidates);
                  obj.plot_triangular_traj(switch_blend_traj, safety_vertex, intermediate_waypt_vertex); % debugging plots
                  % obj.plot_spline_cost(2); % debugging plots
                  chosen_traj = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}];
                  chosen_traj(6, :) = 1; % blend_alpha
                  if obj.is_safe_traj(new_plan) || obj.is_safer_traj(old_chosen_traj, new_plan)
                      break 
                  else
                      fprintf("Increasing replan level set\n"); 
                      replan_level_set = replan_level_set + 0.1;
                      old_chosen_traj = new_plan; 
                  end 
              end
              obj.switch_traj = [obj.switch_traj, switch_blend_traj];
              obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), chosen_traj];
              obj.dynSys.x = old_x; % restore state
        end
        
        function blend_and_replan_mpc_controls(obj) 
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
                  if strcmp(obj.blending.scheme, 'replan_after_k')
                      obj.replan_after_every_k_timestamps_with_safety_controls(plan);
                  elseif strcmp(obj.blending.scheme, 'replan_waypoint')
                      obj.replan_waypoint(plan); 
                  end
                  obj.verbose_plot(2);
               end 
               % get control per timestamp
               x = reshape(obj.state(1:3), [1, 3]);
               u = [obj.blend_traj(4, obj.cur_timestamp), obj.blend_traj(5, obj.cur_timestamp)];
               alpha = 1;
               % update state
               obj.blend_traj(:, obj.cur_timestamp) = [x, u, alpha]'; % old state and new control
               obj.dynSys.updateState(u, obj.dt, x'); 
               obj.state = [obj.dynSys.x', u]; % new state and new control
               obj.cur_timestamp = obj.cur_timestamp + 1;
               obj.replan_time_counter = obj.replan_time_counter + obj.dt;
            end 
        end
                
        function is_unsafe = is_unsafe_state(obj, value)
            is_unsafe = (value <= obj.blending.zero_level_set);
        end 
        
        function is_collision = collided_with_obstacle(obj)
            u = eval_u(obj.exp.grid_2d, obj.exp.binary_occ_map, obj.state(1:2));
            is_collision = (u < 0);
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
        
        function new_plan = truncate_plan_with_replan_time(obj, plan)
            if iscell(plan)
                new_plan = {plan{1}(1:obj.num_mpc_steps), ...
                            plan{2}(1:obj.num_mpc_steps), ...
                            plan{3}(1:obj.num_mpc_steps), ...
                            plan{4}(1:obj.num_mpc_steps), ...
                            plan{5}(1:obj.num_mpc_steps)};
            else
                new_plan = plan(:, 1:obj.num_mpc_steps);
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
                  a = -0.2; % low alpha to indicate we used safety control
              end 
              obj.dynSys.updateState(u, obj.dt, x'); 
              safe_orig_traj(:, i) = [x, u, a]; % old state new control
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
        
        function [safety_score] = eval_traj_safety_score(obj, traj)
            xs = traj{1}; ys = traj{2}; ths = traj{3};
            n = length(xs); 
            safety_score = 0; 
            for i = 1:n
                s = [xs(i), ys(i), ths(i)]; 
                v = obj.brs_planner.get_value(s); 
                safety_score = safety_score + v; 
            end 
        end 
        
        function is_safer = is_safer_traj(obj, traj_a, traj_b)
            % Check if traj_a is safer than traj_b
            if isempty(traj_a)
                is_safer = false; 
                return
            elseif isempty(traj_b)
                is_safer = true;
                return;
            end 
            sa = obj.eval_traj_safety_score(traj_a);
            sb = obj.eval_traj_safety_score(traj_b);
            is_safer = sa < sb;                  
        end
        
        function unsafe_index = get_first_unsafe_index_in_traj(obj, traj)
            xs = traj{1}; ys = traj{2}; ths = traj{3};
            for i=1:length(xs)
                pos = [xs(i), ys(i), ths(i)];
                if obj.brs_planner.get_value(pos) < obj.blending.zero_level_set
                    unsafe_index = i; 
                    return 
                end 
            end 
            unsafe_index = -1;                           
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
        
        function plot_spline_replan(obj, verbosity)
            if obj.plot_level < verbosity 
                return 
            end 
            savefigpath = '';
            if obj.exp.save_plot
                savefigpath = sprintf("%s/replan_traj_timestamp_%d.fig", obj.output_folder, obj.cur_timestamp);  
            end 
            obj.spline_planner.plot_replan_scores(savefigpath);
        end 
        
        function plot_spline_cost(obj, verbosity)
            if obj.plot_level < verbosity 
                return 
            end 
            savefigpath = '';
            if obj.exp.save_plot
                savefigpath = sprintf("%s/spline_cost_timestamp_%d", obj.output_folder, obj.cur_timestamp);  
            end 
            obj.spline_planner.plot_spline_costs(savefigpath);
        end 
        
        function plot_metrics(obj)
            if isempty(obj.blend_traj)
                return
            end 
            obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
            figure(10);
            clf;
            hold on 
            set(gcf,'Position', [10 10 800 1200])
            subplot(5, 2, 1); 
            plot(1:length(obj.scores.lin_vel), obj.scores.lin_vel, 'bo--');
            title("Linear Velocity");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(5, 2, 2); 
            plot(1:length(obj.scores.ang_vel), obj.scores.ang_vel, 'bo--');
            title("Angular Velocity");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(5, 2, 3); 
            plot(1:length(obj.scores.lin_accel), obj.scores.lin_accel, 'bo--');
            title("Linear Accel");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(5, 2, 4); 
            plot(1:length(obj.scores.ang_accel), obj.scores.ang_accel, 'bo--');
            title("Angular Accel");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(5, 2, 5); 
            plot(1:length(obj.scores.lin_jerk), obj.scores.lin_jerk, 'bo--');
            title("Linear Jerk");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(5, 2, 6); 
            plot(1:length(obj.scores.ang_jerk), obj.scores.ang_jerk, 'bo--');
            title("Angular Jerk");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(5, 2, 7); 
            plot(1:length(obj.scores.safety_score), obj.scores.safety_score, 'bo--');
            title("Safety Score");
            xlabel("iteration");
            ylabel("brs value function"); 
            subplot(5, 2, 8); 
            plot(1:length(obj.blend_traj(6, :)), obj.blend_traj(6, :), 'bo--');
            title("Blend Probability");
            xlabel("iteration");
            ylabel("alpha");
            subplot(5, 2, 9); 
            plot(1:length(obj.scores.dist_to_opt_traj), obj.scores.dist_to_opt_traj, 'bo--');
            title("Dist to Opt Traj");
            xlabel("iteration");
            ylabel("meters"); 
            subplot(5, 2, 10); 
            plot(1:length(obj.scores.dist_to_goal), obj.scores.dist_to_goal, 'bo--');
            title("Dist to Goal");
            xlabel("iteration");
            ylabel("meters"); 
            if obj.exp.save_plot
                savefigpath = sprintf("%s/metrics.fig", obj.output_folder);
                savefig(savefigpath); 
            end 
        end 
        
        function plot_triangular_traj(obj, traj, v1, v2)
            figure(6);
            hold on;
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0]);
            obj.plot_traj(traj(1, :), traj(2, :), traj(3, :), 'red', 'spline');
            scatter(v1(1), v1(2), 30, 'bo'); 
            scatter(v2(1), v2(2), 30, 'bo');
        end 
        
        function plot_value_blended_traj(obj, plan_x, plan_y, safe_x, safe_y, blended_plan, alphas, sim_tstep)
            figure(7);
            clf(7);
            hold on;
            
            title(strcat('Simulation at real-time: ', num2str(sim_tstep), ' s'));
            % Plot the background obstacles.
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0]);
            
            sp = scatter(plan_x, plan_y, 15, 'black', 'markerfacecolor', [0.4,0.4,0.4], 'DisplayName', 'plan');
            sp.HandleVisibility = 'off';
            
            ss = scatter(safe_x, safe_y, 15, [168/255., 8/255., 0], 'markerfacecolor', [255/255., 176/255., 176/255.], 'DisplayName', 'plan');
            ss.HandleVisibility = 'off';
            
            for i=1:length(blended_plan{1})
                [val_color, ~] = custom_colormap(alphas(i), 0, 1);
                sb = scatter(blended_plan{1}(i), blended_plan{2}(i), 15, val_color, 'filled', 'DisplayName', 'plan');
                sb.HandleVisibility = 'off';
            end
            % set colorbar.
            [~, cmap] = custom_colormap(0.1, 0, 1);
            colormap(cmap)
            h = colorbar;
            caxis([0,1]);
            ylabel(h, 'alpha (low: more safe, high: more plan)');
            % plot the goal.
            scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
        end 
        
        function plot_planners(obj)
            figure(5);
            clf;
            hold on 
            set(gcf,'Position',[10 10 1000 800])
            % plot environment, goal, and start
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'occ_fmm_map', 'LineWidth', 1, 'color', 'blue');
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal_shape', 'color', 'red');
            scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
            scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
            % plot zero level set value function
            zls = obj.exp.blending.zero_level_set; 
            name = sprintf("BRS (theta=%.2f, levelset=%.2f)", obj.state(3), zls);
            [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], obj.state(3));
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', name, 'color', '#CC1FCB');
            % plot mpc spline traj
            if ~isempty(obj.orig_traj)
                mpc_spline_xs = obj.orig_traj(1, :); 
                mpc_spline_ys = obj.orig_traj(2, :); 
                mpc_spline_ths = obj.orig_traj(3, :); 
                obj.plot_traj(mpc_spline_xs, mpc_spline_ys, mpc_spline_ths, 'red', 'orig traj');   
            end 
            % plot switch mpc traj
            if ~isempty(obj.switch_traj)
                switch_xs = obj.switch_traj(1, :); 
                switch_ys = obj.switch_traj(2, :); 
                switch_ths = obj.switch_traj(3, :); 
                obj.plot_traj(switch_xs, switch_ys, switch_ths, 'blue', 'switch traj');   
            end 
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
            % calculate metrics
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
            % add metrics caption
            if ~isempty(obj.scores) 
                aljs = sprintf("avg_lin_jerk: %.5f", obj.scores.avg_lin_jerk);
                aajs = sprintf("avg_ang_jerk: %.5f", obj.scores.avg_ang_jerk);
                adgs = sprintf("avg_dist_to_goal: %.5f", obj.scores.avg_dist_to_goal); 
                assh = sprintf("avg_safety_score: %.5f", obj.scores.avg_safety_score); 
                adot = sprintf("avg_dist_to_opt_traj: %.5f", obj.scores.avg_dist_to_opt_traj); 
                obj.objective_str = sprintf('%s\n%s\n%s\n%s\n%s', aljs, aajs, adgs, assh, adot); 
                annotation('textbox', [.7 .90 1 .0],'String', obj.objective_str, 'FitBoxToText', 'on', 'Interpreter', 'None');
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

