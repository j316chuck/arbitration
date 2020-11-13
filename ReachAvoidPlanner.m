classdef ReachAvoidPlanner < handle
    %ReachAvoidPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        grid %3d
        schemeData
        dynSys
        tau
        xstart
        xgoal 
        goal_map
        obstacle_map
        data
        valueFun
        data_tau
        opt_traj 
        opt_traj_tau
        extraArgs
    end
    
    methods
        %% Constructs Reach Avoid Planner.
        function obj = ReachAvoidPlanner(grid_3d, schemeData, tau)
            obj.grid = grid_3d;
            obj.schemeData = schemeData; 
            obj.dynSys = schemeData.dynSys;
            obj.tau = tau;
            % Extra Args
            obj.extraArgs.visualize.valueSet = 1;
            obj.extraArgs.visualize.initialValueSet = 1;
            obj.extraArgs.visualize.figNum = 1; %set figure number
            obj.extraArgs.visualize.deleteLastPlot = true;
        end
        
        %% Solves the HJPIDE reach avoid problem
        function solve_reach_avoid(obj, xstart, xgoal, goal_map, obstacle_map)
            if ~isequal(size(goal_map), obj.grid.N')
                fprintf("Shape of goal map is not equal to the grid shape");
            end 
            if ~isequal(size(obstacle_map), obj.grid.N')
                fprintf("Shape of obstacle_map is not equal to the grid shape");
            end
            obj.xstart = xstart;
            obj.xgoal = xgoal;
            obj.goal_map = goal_map;
            obj.obstacle_map = obstacle_map;
            obj.dynSys.x = xstart;
            obj.extraArgs.targets = obj.goal_map;
            obj.extraArgs.obstacles = obstacle_map;
            obj.extraArgs.stopInit = xstart; 
            obj.extraArgs.plotData.plotDims = [1 1 0];
            obj.extraArgs.plotData.projpt = xstart(3);
             % Solve 
            [obj.data, obj.data_tau, ~] = ...
                HJIPDE_solve(obj.goal_map, obj.tau, obj.schemeData, 'minVWithL', obj.extraArgs);          
            obj.valueFun = obj.data(:, :, :, end);
        end 
        
        %% Calculates the Optimal Trajectory
        function traj = get_optimal_trajectory(obj)
            if ~obj.valueFun 
                fprintf("No value function yet");
            end
            obj.extraArgs.projDim = [1 1 0];
            [obj.opt_traj, obj.opt_traj_tau] = computeOptTraj(obj.grid, ... 
                                flip(obj.data,4), obj.data_tau, obj.dynSys, obj.extraArgs);
            traj = obj.opt_traj;
        end

        function plot_traj(obj)
            figure(5);
            clf; 
            hold on;
            plot(obj.xstart(1), obj.xstart(2), 'rx');
            plot(obj.xgoal(1), obj.xgoal(2), 'bo');
            
            traj = obj.opt_traj;          
            xs = traj(1, :); 
            ys = traj(2, :); 
            ths = traj(3, :);
            s = scatter(xs, ys, 'r', 'filled');
            grid_xs = obj.grid.xs;
            colors = [linspace(0.1, 0.9, length(grid_xs))', zeros([length(grid_xs),1]), zeros([length(grid_xs),1])];
            s.CData = colors; 
            q = quiver(xs, ys, cos(ths), sin(ths), 'Color', colors(end,:));
            q.ShowArrowHead = 'off';
            q.AutoScale = 'off';
            q.AutoScaleFactor = 0.5;
            hold off;
        end
    end
end

