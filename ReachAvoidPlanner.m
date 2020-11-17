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
            if ~isequal(schemeData.uMode, 'min')
                error("OptCtrl need to minimize distance to goal");
            end
           if ~isequal(schemeData.dMode, 'max')
                error("OptDistb needs to maximize distance to goal");
            end
            obj.grid = grid_3d;
            obj.schemeData = schemeData; 
            obj.dynSys = schemeData.dynSys;
            obj.tau = tau;
            % Extra Args
            obj.extraArgs.visualize = true;
        end
        
        %% Solves the HJPIDE reach avoid problem
        function [traj] = solve_reach_avoid(obj, xstart, xgoal, goal_map, obstacle_map, optTrajDt)
            if ~isequal(size(goal_map), obj.grid.N')
                error("Shape of goal map is not equal to the grid shape");
            end 
            if ~isequal(size(obstacle_map), obj.grid.N')
                error("Shape of obstacle_map is not equal to the grid shape");
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
            obj.extraArgs.projDim = [1 1 0];
            % Solve 
            [obj.data, obj.data_tau, ~] = ...
                HJIPDE_solve(obj.goal_map, obj.tau, obj.schemeData, 'minVWithL', obj.extraArgs);          
            obj.valueFun = obj.data(:, :, :, end);
            % compute opt traj;
            obj.extraArgs.optTrajDt = optTrajDt;
            [obj.opt_traj, obj.opt_traj_tau] = computeOptTraj(obj.grid, ... 
                                flip(obj.data,4), obj.data_tau, obj.dynSys, obj.extraArgs);
            traj = obj.opt_traj;
        end 
       

        function plot_traj(obj)
            figure(5);
            hold on;
            scatter(obj.xstart(1), obj.xstart(2), 100, 'black', 'o', 'filled');
            scatter(obj.xgoal(1), obj.xgoal(2), 100, 'red', 'o', 'filled');
            
            traj = obj.opt_traj;          
            xs = traj(1, :); 
            ys = traj(2, :); 
            ths = traj(3, :);
            s = scatter(xs, ys, 'green', 'filled');
            grid_xs = obj.grid.xs;
            colors = [linspace(0.1, 0.9, length(grid_xs))', zeros([length(grid_xs),1]), zeros([length(grid_xs),1])];
            s.CData = colors; 
            q = quiver(xs, ys, cos(ths), sin(ths), 'Color', 'g');
            q.ShowArrowHead = 'off';
            q.AutoScale = 'off';
            q.AutoScaleFactor = 0.1;
            hold off;
        end
    end
end

