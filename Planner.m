
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
        blend_scheme
        control_scheme
        dt
        num_mpc_steps
        replan_time_counter
        num_waypts
        horizon
        reach_avoid_planner
        brs_planner
        spline_planner
        dynSys
        orig_traj
        safety_traj
        blend_traj
        switch_traj
        exp_name
        output_folder
        plot_level
        objective_str 
        scores
        termination_state
        use_safety_control
        unknown_map
        is_unknown_environment
        sensor_shape
        sensor_rad 
        senseFOV 
        farPlane
        start_time
        total_exp_time
        mpc_plan_time
        hjipde_time
    end
    
    methods
        function obj = Planner(exp)
            obj.exp = exp; 
            obj.start = exp.start;
            obj.goal = exp.goal;
            obj.stop_goal_dx = exp.stop_goal_dx;
            obj.max_num_planning_pts = exp.max_num_planning_pts;
            obj.stop_on_collision = exp.stop_on_collision;
            obj.cur_timestamp = 1;
            obj.state = [obj.start(1:4)', 0];
            obj.blending = exp.blending;
            obj.blend_scheme = exp.blending.blend_scheme;
            obj.control_scheme = exp.blending.control_scheme; 
            obj.dt = exp.dt; 
            obj.num_mpc_steps = ceil(obj.blending.replan_dt / obj.dt); % num steps we take in a mpc 
            % goes from [0-obj.blending.replan_dt], start of with a replan
            obj.replan_time_counter = obj.blending.replan_dt;
            obj.num_waypts = exp.num_waypts;
            obj.horizon = exp.horizon;
            obj.reach_avoid_planner = exp.reach_avoid_planner;
            obj.brs_planner = exp.brs_planner;
            obj.spline_planner = exp.spline_planner;
            obj.dynSys = obj.spline_planner.dynSys;
            % stores the state (x), control (u), blending prob (alpha)
            % 6 x N matrix where N is control trajectory
            obj.blend_traj = []; % alpha = blending_scheme
            obj.orig_traj = []; % alpha = 1
            obj.safety_traj = [];  % alpha = 0
            obj.switch_traj = []; % alpha = 0 if V(x) < 0 
            obj.termination_state = -1; 
            obj.start_time = tic; 
            obj.total_exp_time = zeros(obj.max_num_planning_pts, 1); 
            obj.mpc_plan_time = []; 
            obj.hjipde_time = [];
            % Logging information
            repo = what('arbitration');
            obj.exp_name = exp.exp_name; 
            obj.output_folder = strcat(repo.path, "/outputs/", obj.exp_name);
            obj.plot_level = obj.exp.plot_level;
            if exp.clear_dir && exist(obj.output_folder, 'dir')
                rmdir(obj.output_folder, 's');
            end
            if ~exist(obj.output_folder, 'dir')
                mkdir(obj.output_folder); 
                mkdir(sprintf("%s/planners_fig", obj.output_folder));
                mkdir(sprintf("%s/planners_png", obj.output_folder)); 
            end
            if exp.run_brs
                obj.brs_planner.solve_brs_avoid(exp.obstacle);
                brs_planner = obj.brs_planner;
                filename = sprintf("%s/data/%s_%s/brs_planner.mat", repo.path, exp.map_basename, exp.grid_size); 
                save(filename, 'brs_planner'); 
            elseif strcmp(exp.environment_type, "known")  % load cached planner only if the environment is known
                filename = sprintf("%s/data/%s_%s/brs_planner.mat", repo.path, exp.map_basename, exp.grid_size); 
                load(filename, 'brs_planner'); 
                obj.brs_planner = brs_planner;
            end 
            if exp.run_planner
                obj.reach_avoid_planner.solve_reach_avoid(exp.start(1:3), exp.goal(1:3), exp.goal_map_3d, exp.obstacle, exp.dt);
                reach_avoid_planner = obj.reach_avoid_planner;
                filename = sprintf("%s/data/%s_%s/%s.mat", repo.path, exp.map_basename, exp.grid_size, exp.point_nav_str); 
                save(filename, 'reach_avoid_planner'); 
            else
                filename =  sprintf("%s/data/%s_%s/%s.mat", repo.path, exp.map_basename, exp.grid_size, exp.point_nav_str); 
                load(filename, 'reach_avoid_planner');
                obj.reach_avoid_planner = reach_avoid_planner;
            end 
            if strcmp(exp.environment_type, 'unknown')
                obj.is_unknown_environment = true;
                obj.unknown_map = exp.unknown_occ_map; 
                obj.unknown_map.update_map_and_cost(exp.start);
            else
                obj.is_unknown_environment = false; 
            end 
        end
        
        %% Entry point
        function blend_plans(obj)
            while true  
               if obj.reached_stop_condition()
                  return;
               end 
               if obj.replan_time_counter >= obj.blending.replan_dt
                   obj.plan_mpc_controls(); % modifies obj.blend_traj
               end 
               [x, u, alpha] = obj.get_next_control(); 
               obj.update_state(x, u, alpha); % modifies obj.blend_traj
               obj.verbose_plot(3); % plot plan and metrics if plot_level >= 3
            end 
        end 
                
        %% Stop Conditions
        function stop = reached_stop_condition(obj)
            if obj.reached_max_timestamps() || obj.reached_goal() || obj.collided_with_obstacle()
                obj.verbose_plot(1);
                obj.save_state();
                stop = true;
            else 
                stop = false; 
            end     
        end 
        
        %% Get next low level mpc control at current timestamp
        function [x, u, alpha] = get_next_control(obj) 
            x = reshape(obj.state(1:3), [1, 3]);
            v = obj.brs_planner.get_value(x);
            u1 = [obj.blend_traj(4, obj.cur_timestamp), obj.blend_traj(5, obj.cur_timestamp)];
            u2 = obj.brs_planner.get_avoid_u(x)';
            if strcmp(obj.control_scheme, 'follow') 
                u = u1; 
                alpha = obj.blend_traj(6, obj.cur_timestamp); 
            elseif strcmp(obj.control_scheme, 'switch')
               if obj.use_safety_control || obj.is_unsafe_state(v)
                   obj.use_safety_control = true; 
                   alpha = 0; 
                   u = u2; 
               else 
                   alpha = obj.blend_traj(6, obj.cur_timestamp); 
                   u = u1; 
               end 
            else
               warning("control scheme not supported");  
               return
            end
        end 
        
        %% Resets the robot to a state at the END of a specific mpc plan iteration
        %  Call this function with a monotonically decreasing mpc plan
        %  iteration AFTER running your experiment for DEBUGGING purposes.
        %  Note: the mpc_plan_iteration != obj.cur_timestamp. 
        %  It only makes sense to reset the robot state if we are planning next timestamp, 
        %  Thus, obj.cur_timestamp = (mpc_plan_iteration * obj.num_mpc_steps) + 1
        %  You should be able to call obj.blend_plans() immediately after
        %  this function and achieve the SAME outputs as obj.blend_plans()
        %  from the beginning.
        function reset_state(obj, mpc_plan_iteration)
            timestamp = mpc_plan_iteration * obj.num_mpc_steps; 
            if timestamp < 0 || timestamp >= obj.cur_timestamp
                return; 
            end 
            obj.replan_time_counter = obj.blending.replan_dt; % start with replan
            % begin with a timestamp that is about to replan
            obj.cur_timestamp = mpc_plan_iteration * obj.num_mpc_steps  + 1;
            if timestamp ~= 0
                u = obj.blend_traj(4:5, timestamp)';
                x = obj.blend_traj(1:3, timestamp+1)'; 
                obj.state = [x, u];
            else
                obj.state = [obj.start(1:4)', 0];
            end
            if obj.is_unknown_environment
                obj.unknown_map.reset_state(timestamp+1); 
            end
            if ~isempty(obj.blend_traj)
                obj.blend_traj = obj.blend_traj(:, 1:timestamp); 
            end 
            if ~isempty(obj.switch_traj)
                obj.switch_traj = obj.switch_traj(:, 1:timestamp); 
            end 
            if ~isempty(obj.safety_traj)
                obj.safety_traj = obj.safety_traj(:, 1:timestamp); 
            end 
            if ~isempty(obj.orig_traj)
                obj.orig_traj = obj.orig_traj(:, 1:timestamp); 
            end   
        end 
        
        %% Update state of robot
        function update_state(obj, x, u, alpha) 
            obj.blend_traj(:, obj.cur_timestamp) = [x, u, alpha]'; % old state and new control
            nx = obj.dynSys.updateState(u, obj.dt, x'); % update state
            obj.state = [nx', u]; % new state and old control
            obj.total_exp_time(obj.cur_timestamp) = toc(obj.start_time); 
            obj.cur_timestamp = obj.cur_timestamp + 1; % increase time stamp
            obj.replan_time_counter = obj.replan_time_counter + obj.dt; % increase replan_time
            if obj.is_unknown_environment
                obj.unknown_map.update_map_and_cost(nx); 
            end 
        end 
        
        %% Replan original plans
        function replan_safety_value(obj, plan)
            alpha = obj.blending.alpha;
            new_plan = obj.spline_planner.replan_with_brs_planner(obj.state, ...
                plan{1}, plan{1}, obj.brs_planner, alpha);
            blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
            next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
        end 
        
        function replan_safety_control(obj, plan, safety_plan)
            alpha = obj.blending.alpha;
            new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, ...
                plan{1}, plan{2}, safety_plan(1, :), safety_plan(2, :), alpha);    
            blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
            next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
        end 
        
        function replan_sample_safety_value(obj, plan)
            alphas = flip([0; sort(rand(obj.exp.blending.num_alpha_samples, 1)); 1.0]);
            for i = 1:length(alphas)
              alpha = alphas(i); 
              new_plan = obj.spline_planner.replan_with_brs_planner(obj.state, ...
                  plan{1}, plan{2}, obj.brs_planner, alpha);
              use_zero_blend = (alpha == 0); 
              if obj.is_safe_mpc_plan(new_plan) || use_zero_blend
                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                  next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
                  break
              end
            end
        end 
        
        function replan_sample_safety_control(obj, plan, safety_plan)
            alphas = flip([0; sort(rand(obj.exp.blending.num_alpha_samples, 1)); 1.0]);
            for i = 1:length(alphas)
              alpha = alphas(i); 
              new_plan = obj.spline_planner.replan_with_safety_controls(obj.state, ...
                  plan{1}, plan{2}, safety_plan(1, :), safety_plan(2, :), alpha);    
              use_zero_blend = (alpha == 0); 
              if obj.is_safe_mpc_plan(new_plan) || use_zero_blend
                  blend_alpha = (ones(length(new_plan{1}), 1) * alpha)';
                  next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; blend_alpha];
                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
                  break
              end 
            end        
        end 
        
        function replan_time_vary_alpha_open_loop_safety_control(obj, plan, safety_plan)
            [new_plan, new_alphas] = obj.spline_planner.open_loop_replan_with_value_blending(obj.state, ...
                plan{1}, plan{2}, safety_plan(1, :), safety_plan(2, :), obj.brs_planner, obj.blending.blend_function);
            next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; new_alphas'];  
            % ========== DEBUGGING! =========== %                          
            %obj.plot_safety_score_blended_traj(plan{1}, plan{2}, ... 
            %   safety_plan(1, :), safety_plan(2, :), new_plan, new_alphas);
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
        end 
        
        function replan_time_vary_alpha_closed_loop_safety_control(obj, plan)
            [new_plan, new_alphas] = obj.spline_planner.closed_loop_replan_with_value_blending(obj.state, ...
                plan{1}, plan{2}, obj.brs_planner, obj.blending.blend_function);
            next_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}; new_alphas'];  
            % ========== DEBUGGING! =========== %                          
            % obj.plot_safety_score_blended_traj(plan{1}, plan{2}, ... 
            %    safety_plan(1, :), safety_plan(2, :), new_plan, new_alphas);
            obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), next_plan]; 
        end 
        
                
        function replan_waypoint(obj, plan)
              if obj.is_safe_mpc_plan(plan)
                  new_plan = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
                  new_plan(6, :) = 1;
                  obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), new_plan];
                  return; 
              end
              
              % use original mpc planner until safety score goes below threshold
              mpc_plan = obj.get_mpc_plan(plan); 
              mpc_plan(6, :) = 1; % set alpha prob blending to be 1
              ui = obj.get_first_unsafe_index_in_mpc_plan(plan);
              % safety_vertex = mpc_plan(1:3, ui); 
              switch_blend_plan = mpc_plan(:, 1:ui-1); 
              
              % apply safety controls until safety score goes above a threshold
              obj.dynSys.x = mpc_plan(1:3, ui); 
              while 1 
                  x = reshape(obj.dynSys.x, [1, 3]); 
                  safety_score = obj.brs_planner.get_value(x');
                  reached_safe_point = (safety_score >= obj.blending.replan_level_set);
                  if reached_safe_point
                      break 
                  end 
                  u = obj.brs_planner.get_avoid_u(x)'; 
                  alpha = 1;
                  switch_blend_plan(:, end+1) = [x, u, alpha]; 
                  obj.dynSys.updateState(u, obj.dt, x');
              end 
              intermediate_waypt_vertex = [obj.dynSys.x', u];
              
              % replan through intermediate point above threshold
              new_plan = obj.spline_planner.adaptive_replan_through_waypoint(obj.state, ...
                    intermediate_waypt_vertex, obj.blending.replan_spline_max_num_candidates);
              % obj.plot_triangular_traj(switch_blend_traj, safety_vertex, intermediate_waypt_vertex); % debugging plots
              % obj.plot_spline_cost(2); % debugging plot
              
              % store plans in obj.blend_traj and obj.switch_traj
              if isempty(new_plan)
                new_plan = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}];
              else 
                new_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}];                  
              end
              new_plan(6, :) = 1; % set alpha to 1
              obj.switch_traj = [obj.switch_traj, switch_blend_plan];
              obj.dynSys.x = obj.state(1:3); % restore state
              obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), new_plan];
        end
 
        function replan_safe_traj(obj, orig_plan)
            new_plan = obj.spline_planner.replan_only_safe_traj(obj.state, ...
                obj.brs_planner, obj.blending.zero_level_set, obj.blending.num_mpc_safety_look_ahead); 
            if isempty(new_plan)
                obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), orig_plan]; 
            else
                new_plan = [new_plan{1}; new_plan{2}; new_plan{3}; new_plan{4}; new_plan{5}];
                new_plan(6, :) = 1; % set alpha to 1
                obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), new_plan]; 
            end 
        end 
        
        %% Plan high level mpc plan for obj.horizon time to be taken for obj.num_mpc_steps iterations
        function plan_mpc_controls(obj)
            t_mpc_start = tic;
            if obj.is_unknown_environment
                spline_obs_map = obj.unknown_map.signed_dist_planner;
                obj.spline_planner.set_sd_obs(spline_obs_map); 
                safety_map = obj.unknown_map.signed_dist_safety; 
                tic 
                obj.brs_planner.solve_brs_avoid(safety_map); 
                obj.hjipde_time(end+1) = toc; 
            end 
            obj.replan_time_counter = 0;
            obj.use_safety_control = false; 
            plan = obj.spline_planner.plan(obj.state); 
            one_alphas = ones(length(plan{1}), 1)';
            orig_plan = [plan{1}; plan{2}; plan{3}; plan{4}; plan{5}; one_alphas];
            obj.orig_traj = [obj.orig_traj(:, 1:obj.cur_timestamp-1), orig_plan];
            safety_plan = obj.get_next_safety_plan();
            obj.safety_traj = [obj.safety_traj(:, 1:obj.cur_timestamp-1), safety_plan];
            % Replan entry point
            if isempty(plan)
                return; 
            elseif strcmp(obj.blend_scheme, 'none')
                obj.blend_traj = [obj.blend_traj(:, 1:obj.cur_timestamp-1), orig_plan]; 
            elseif strcmp(obj.blend_scheme, 'safety_value') 
                obj.replan_safety_value(plan); 
            elseif strcmp(obj.blend_scheme, 'safety_control')
                obj.replan_safety_control(plan, safety_plan); 
            elseif strcmp(obj.blend_scheme, 'sample_safety_value')
                obj.replan_sample_safety_value(plan); 
            elseif strcmp(obj.blend_scheme, 'sample_safety_control')
                obj.replan_sample_safety_control(plan, safety_plan); 
            elseif strcmp(obj.blend_scheme, 'time_vary_alpha_open_loop')
                obj.replan_time_vary_alpha_open_loop_safety_control(plan, safety_plan); 
            elseif strcmp(obj.blend_scheme, 'time_vary_alpha_closed_loop')
                obj.replan_time_vary_alpha_closed_loop_safety_control(plan); 
            elseif strcmp(obj.blend_scheme, 'replan_waypoint')
                obj.replan_waypoint(plan); 
            elseif strcmp(obj.blend_scheme, 'replan_safe_traj')
                obj.replan_safe_traj(orig_plan); 
            else 
                warning("blending scheme not supported"); 
                return 
            end 
            obj.verbose_plot(2);     % plot metrics and robot path
            obj.plot_unknown_maps(2); % unknown map debugging
            %obj.plot_spline_replan(2); % spline replan debugging
            %obj.plot_spline_cost(2); % spline planner debugging
            obj.mpc_plan_time(end+1) = toc(t_mpc_start); 
        end 
        
        %% Helper Functions
        function d = l2_dist(obj, x1, x2, y1, y2)
            d = ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
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
        
                
        function mpc_plan = get_mpc_plan(obj, plan)
            mpc_plan = [plan{1}(1:obj.num_mpc_steps); ...
                        plan{2}(1:obj.num_mpc_steps); ...
                        plan{3}(1:obj.num_mpc_steps); 
                        plan{4}(1:obj.num_mpc_steps); 
                        plan{5}(1:obj.num_mpc_steps)];
        end 
        
        function safety_scores = get_mpc_plan_safety_scores(obj, plan)
            safety_scores = zeros(obj.num_mpc_steps, 1);             
            for i = 1:obj.num_mpc_steps
                x = [plan{1}(i), plan{2}(i), plan{3}(i)]; 
                safety_scores(i) = obj.brs_planner.get_value(x); 
            end 
        end 
        
        function safety_scores = get_plan_safety_scores(obj, plan)
            safety_scores = zeros(obj.num_mpc_steps, 1);  
            xs = plan{1}; ys = plan{2}; ths = plan{3};
            for i = 1:length(xs)
                x = [xs(i), ys(i), ths(i)]; 
                safety_scores(i) = obj.brs_planner.get_value(x); 
            end 
        end 
        
        function is_safe = is_safe_mpc_plan(obj, plan)
            xs = plan{1}; ys = plan{2}; ths = plan{3};
            for i=1:obj.num_mpc_steps
                pos = [xs(i), ys(i), ths(i)];
                if obj.brs_planner.get_value(pos) <= obj.blending.zero_level_set
                    is_safe = false; 
                    return 
                end 
            end 
            is_safe = true;                           
        end        
        
        function is_safe = is_safe_plan(obj, plan)
            xs = plan{1}; ys = plan{2}; ths = plan{3};
            for i=1:length(xs)
                pos = [xs(i), ys(i), ths(i)];
                if obj.brs_planner.get_value(pos) <= obj.blending.zero_level_set
                    is_safe = false; 
                    return 
                end 
            end 
            is_safe = true;                           
        end
        
        function plan_a_safer = compare_plans(obj, plan_a, plan_b)
            % Return true if plan a is safer than plan b
            if isempty(plan_a)
                plan_a_safer = false; 
                return
            elseif isempty(plan_b)
                plan_a_safer = true;
                return;
            end 
            sa = sum(obj.get_plan_safety_scores(plan_a)); 
            sb = sum(obj.get_plan_safety_scores(plan_b)); 
            plan_a_safer = sa < sb;                  
        end
        
        function plan_a_safer = compare_mpc_plans(obj, plan_a, plan_b)
            % Return true if plan a is safer than plan b
            if isempty(plan_a)
                plan_a_safer = false; 
                return
            elseif isempty(plan_b)
                plan_a_safer = true;
                return;
            end 
            sa = sum(obj.get_mpc_plan_safety_scores(plan_a)); 
            sb = sum(obj.get_mpc_plan_safety_scores(plan_b)); 
            plan_a_safer = sa < sb;                  
        end
        
        function unsafe_index = get_first_unsafe_index_in_plan(obj, plan)
            ss = obj.get_plan_safety_scores(plan); 
            unsafe_index = find(ss < 0, 1, 'first'); 
            if isempty(unsafe_index)
                unsafe_index = -1; 
            end 
        end 
        
        function unsafe_index = get_first_unsafe_index_in_mpc_plan(obj, plan)
            ss = obj.get_mpc_plan_safety_scores(plan); 
            unsafe_index = find(ss < 0, 1, 'first'); 
            if isempty(unsafe_index)
                unsafe_index = -1; 
            end 
        end 
        
        function safety_plan = get_next_safety_plan(obj)
            obj.dynSys.x = obj.state(1:3); % get current state
            safety_plan = zeros(6, 0);
            safety_state = obj.state; 
            alpha = 0; 
            for i = 1:obj.num_waypts
              x = reshape(safety_state(1:3), [1, 3]);
              u = obj.brs_planner.get_avoid_u(x)';
              obj.dynSys.updateState(u, obj.dt, x'); 
              safety_plan(:, i) = [x, u, alpha]; %old state new control
              safety_state = [obj.dynSys.x', u]; % new state and new control
            end 
            obj.dynSys.x = obj.state(1:3); % restore current state
        end     
        
        function save_state(obj)
            obj.blend_traj(1:3, obj.cur_timestamp) = obj.state(1:3); % update last position
            if obj.reached_goal() 
                obj.termination_state = 0;
            elseif obj.collided_with_obstacle()
                obj.termination_state = 1;
            elseif obj.reached_max_timestamps()
                obj.termination_state = 2;
            else
                obj.termination_state = -1;
            end 
            save_planner_file = sprintf("%s/final_state.mat", obj.output_folder);
            save(save_planner_file, 'obj');
        end 
        
        %% Plotting functions
        function verbose_plot(obj, threshold)
            if obj.plot_level < threshold
                return;
            end 
            obj.plot_metrics(); 
            obj.plot_planners();
        end 
        
        function plot_metrics(obj)
            if isempty(obj.blend_traj)
                return
            end 
            obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
            figure(10);
            clf;
            hold on; 
            set(gcf,'Position', [10 10 800 1200])
            subplot(6, 2, 1); 
            plot(1:length(obj.scores.lin_vel), obj.scores.lin_vel, 'bo--');
            title("Linear Velocity");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(6, 2, 2); 
            plot(1:length(obj.scores.ang_vel), obj.scores.ang_vel, 'bo--');
            title("Angular Velocity");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(6, 2, 3); 
            plot(1:length(obj.scores.lin_accel), obj.scores.lin_accel, 'bo--');
            title("Linear Accel");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(6, 2, 4); 
            plot(1:length(obj.scores.ang_accel), obj.scores.ang_accel, 'bo--');
            title("Angular Accel");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(6, 2, 5); 
            plot(1:length(obj.scores.lin_jerk), obj.scores.lin_jerk, 'bo--');
            title("Linear Jerk");
            xlabel("iteration");
            ylabel("mps"); 
            subplot(6, 2, 6); 
            plot(1:length(obj.scores.ang_jerk), obj.scores.ang_jerk, 'bo--');
            title("Angular Jerk");
            xlabel("iteration");
            ylabel("rps"); 
            subplot(6, 2, 7); 
            plot(1:length(obj.scores.safety_score), obj.scores.safety_score, 'bo--');
            title("Safety Score");
            xlabel("iteration");
            ylabel("brs value function"); 
            subplot(6, 2, 8); 
            plot(1:length(obj.blend_traj(6, :)), obj.blend_traj(6, :), 'bo--');
            title("Blend Probability");
            xlabel("iteration");
            ylabel("alpha");
            subplot(6, 2, 9); 
            plot(1:length(obj.scores.dist_to_opt_traj), obj.scores.dist_to_opt_traj, 'bo--');
            title("Dist to Opt Traj");
            xlabel("iteration");
            ylabel("meters"); 
            subplot(6, 2, 10); 
            plot(1:length(obj.scores.dist_to_goal), obj.scores.dist_to_goal, 'bo--');
            title("Dist to Goal");
            xlabel("iteration");
            ylabel("meters"); 
            subplot(6, 2, 11); 
            plot(1:obj.cur_timestamp-1, obj.total_exp_time(1:obj.cur_timestamp-1), 'bo--');
            title("Total Experiment Time");
            xlabel("timestamp");
            ylabel("seconds"); 
            subplot(6, 2, 12); 
            plot(1:length(obj.mpc_plan_time), obj.mpc_plan_time, 'bo--');
            title("MPC Planning Time");
            xlabel("iteration");
            ylabel("seconds"); 
            if obj.exp.save_plot
                savefigpath = sprintf("%s/metrics.fig", obj.output_folder);
                savefig(savefigpath); 
            end
            hold off;    
        end
        
        function plot_triangular_traj(obj, traj, v1, v2)
            figure(6);
            hold on;
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0]);
            plot_traj(traj(1, :), traj(2, :), traj(3, :), 'red', 'spline');
            scatter(v1(1), v1(2), 30, 'bo'); 
            scatter(v2(1), v2(2), 30, 'bo');
            hold off;
        end 
        
        function plot_safety_score_blended_traj(obj, plan_x, plan_y, safe_x, safe_y, blended_plan, alphas)
            figure(7);
            %clf(7);
            hold on;
            
            % title(strcat('Simulation at real-time: ', num2str(sim_tstep), ' s'));
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
            hold off;
        end 
        
       
        function plot_planners(obj)
            figure(5);
            clf;
            hold on; 
            set(gcf,'Position',[10 10 1000 800])
            % plot environment, goal, and start
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'occ_fmm_map', 'LineWidth', 1, 'color', 'blue');
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal_shape', 'color', 'red');
            scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
            scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
            % plot zero level set value function
            zls = obj.blending.zero_level_set; 
            name = sprintf("BRS (theta=%.2f, levelset=%.2f)", obj.state(3), zls);
            [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], obj.state(3));
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', name, 'color', '#CC1FCB');
            % plot mpc spline traj
            if ~isempty(obj.orig_traj)
                mpc_spline_xs = obj.orig_traj(1, :); 
                mpc_spline_ys = obj.orig_traj(2, :); 
                mpc_spline_ths = obj.orig_traj(3, :); 
                plot_traj(mpc_spline_xs, mpc_spline_ys, mpc_spline_ths, 'red', 'orig traj');   
            end 
            % plot switch mpc traj
            if ~isempty(obj.switch_traj)
                switch_xs = obj.switch_traj(1, :); 
                switch_ys = obj.switch_traj(2, :); 
                switch_ths = obj.switch_traj(3, :); 
                plot_traj(switch_xs, switch_ys, switch_ths, 'blue', 'switch traj');   
            end 
            % plot safety_traj
            if ~isempty(obj.safety_traj)
                safety_spline_xs = obj.safety_traj(1, :); 
                safety_spline_ys = obj.safety_traj(2, :); 
                safety_spline_ths = obj.safety_traj(3, :); 
                plot_traj(safety_spline_xs, safety_spline_ys, safety_spline_ths, 'magenta', 'safety traj');    
            end
            % plot blending traj
            if ~isempty(obj.blend_traj)
                blend_xs = obj.blend_traj(1, :); 
                blend_ys = obj.blend_traj(2, :); 
                blend_ths = obj.blend_traj(3, :); 
                use_avoid_probs = obj.blend_traj(6, :);
                blend_name = obj.blend_scheme;
                plot_traj_probs(blend_xs, blend_ys, blend_ths, use_avoid_probs, blend_name);
            end 
            % plot reach avoid traj
            reach_avoid_xs = obj.reach_avoid_planner.opt_traj(1, :);
            reach_avoid_ys = obj.reach_avoid_planner.opt_traj(2, :);
            reach_avoid_ths = obj.reach_avoid_planner.opt_traj(3, :);
            plot_traj(reach_avoid_xs, reach_avoid_ys, reach_avoid_ths, 'green', 'reach avoid');
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
            if ~isempty(obj.blend_traj)
                obj.scores = objectives(obj.blend_traj, obj.reach_avoid_planner.opt_traj, obj.brs_planner, obj.dt, obj.goal);
                aljs = sprintf("avg_lin_jerk: %.5f", obj.scores.avg_lin_jerk);
                aajs = sprintf("avg_ang_jerk: %.5f", obj.scores.avg_ang_jerk);
                adgs = sprintf("avg_dist_to_goal: %.5f", obj.scores.avg_dist_to_goal); 
                assh = sprintf("avg_safety_score: %.5f", obj.scores.avg_safety_score); 
                adot = sprintf("avg_dist_to_opt_traj: %.5f", obj.scores.avg_dist_to_opt_traj); 
                obj.objective_str = sprintf('%s\n%s\n%s\n%s\n%s', aljs, aajs, adgs, assh, adot); 
                annotation('textbox', [.7 .90 1 .0],'String', obj.objective_str, 'FitBoxToText', 'on', 'Interpreter', 'None');
            end 
            colorbar;
            if obj.exp.save_plot
                savefigpath = sprintf("%s/planners_fig/planners_%d.fig", obj.output_folder, obj.cur_timestamp);
                savefig(savefigpath); 
                png_path = sprintf("%s/planners_png/planners_%d.png", obj.output_folder, obj.cur_timestamp);
                saveas(gcf, png_path);
            end 
            hold off;    
        end 
        
        function plot_spline_replan(obj, threshold)
            if obj.plot_level < threshold 
                return 
            end 
            savefigpath = '';
            if obj.exp.save_plot
                savefigpath = sprintf("%s/replan_traj_timestamp_%d.fig", obj.output_folder, obj.cur_timestamp);  
            end 
            obj.spline_planner.plot_replan_scores(savefigpath);
        end 
        
        function plot_unknown_maps(obj, threshold)
            if obj.plot_level < threshold 
                return 
            end 
            if ~obj.is_unknown_environment
                return 
            end 
            figure(2); 
            set(gcf,'Position', [10 10 1200 1000])
            clf
            % Plot safety occupancy map
            subplot(2, 2, 1); 
            bx = obj.blend_traj(1, 1:obj.cur_timestamp);
            by = obj.blend_traj(2, 1:obj.cur_timestamp);
            bt = obj.blend_traj(3, 1:obj.cur_timestamp);
            hold on;
            contourf(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.unknown_map.occupancy_map_safety, [0 0], 'DisplayName', 'safety', 'color', 'blue');
            plot_traj(bx, by, bt, 'red', 'orig traj'); 
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Occ Map Safety");
            hold off;
            % Plot safety signed dist map
            subplot(2, 2, 2); 
            hold on;
            contourf(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.unknown_map.safety_fmm, 'DisplayName', 'safety');
            plot_traj(bx, by, bt, 'red', 'orig traj'); 
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Signed Dist Safety");
            hold off;
            % Plot planner occupancy map
            subplot(2, 2, 3); 
            hold on;
            plot_traj(bx, by, bt, 'red', 'orig traj'); 
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle', 'color', 'black');
            zls = obj.blending.zero_level_set; 
            name = sprintf("BRS (theta=%.2f, levelset=%.2f)", obj.state(3), zls);
            [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], obj.state(3));
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', name, 'color', '#CC1FCB');
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.unknown_map.occupancy_map_planner, [0 0], 'DisplayName', 'planner', 'color', 'blue');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Occ Map Planner");
            hold off;
            % Plot safety signed dist map
            subplot(2, 2, 4); 
            hold on;
            contourf(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.unknown_map.signed_dist_planner, 'DisplayName', 'planner');
            plot_traj(bx, by, bt, 'red', 'orig traj'); 
            contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Signed Dist Planner");
            hold off;
            if obj.exp.save_plot
                savefigpath = sprintf("%s/unknown_map_%d.fig", obj.output_folder, obj.cur_timestamp);
                savefig(savefigpath); 
            end
        end 
        
        function plot_spline_cost(obj, threshold)
            if obj.plot_level < threshold 
                return 
            end 
            savefigpath = '';
            if obj.exp.save_plot
                savefigpath = sprintf("%s/spline_cost_timestamp_%d", obj.output_folder, obj.cur_timestamp);  
            end 
            obj.spline_planner.plot_spline_costs(savefigpath);
        end 
    end
end

