classdef BRSAvoidPlanner < handle
    %BRSAvoidPLANNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        grid %3d
        schemeData
        dynSys
        tau
        obstacle_map % negative inside obstacle, positive outside
        data
        data_tau
        valueFun
        derivValueFun
        dt 
        extraArgs
    end
    
    methods
        %% Constructs BRS Avoid Planner.
        function obj = BRSAvoidPlanner(grid_3d, schemeData, tau, dt)
            if ~isequal(schemeData.uMode, 'max')
                error("OptCtrl need to maximize distance to obstacle");
            end
           if ~isequal(schemeData.dMode, 'min')
                error("OptDistb needs to minimize distance to obstacle");
            end
            obj.grid = grid_3d;
            obj.schemeData = schemeData; 
            obj.dynSys = schemeData.dynSys;
            obj.tau = tau;
            obj.dt = dt; 
            % Extra Args
            obj.extraArgs.quiet = true;
            obj.extraArgs.visualize.figNum = 15;
            obj.extraArgs.visualize.deleteLastPlot = true;
            obj.extraArgs.visualize.viewGrid = false;
            obj.extraArgs.visualize.initialValueSet = 1;
            obj.extraArgs.visualize.valueSet = 1;
            obj.extraArgs.plotData.plotDims = [1 1 0];
            obj.extraArgs.plotData.projpt = 0;
            obj.extraArgs.stopConvergeTTR = false;
        end
        
        %% Solves the brs avoid problem
        function solve_brs_avoid(obj, obstacle_map)
            if ~isequal(size(obstacle_map), obj.grid.N')
                error("Shape of obstacle_map is not equal to the grid shape");
            end
            obj.obstacle_map = obstacle_map;
            % Solve 
            [obj.data, obj.data_tau, ~] = ...
                HJIPDE_solve(obj.obstacle_map, obj.tau, obj.schemeData, 'minVOverTime', obj.extraArgs);   
            obj.valueFun = obj.data(:, :, :, end);
            obj.derivValueFun = computeGradients(obj.grid, obj.valueFun);
        end 
       

        function [uOpt] = get_avoid_u(obj, x)
            % Value of the derivative at that particular state
            current_deriv = eval_u(obj.grid, obj.derivValueFun, x);
            % Get the optimal control to apply at this state
            uOpt = obj.dynSys.optCtrl(obj.data_tau(end), x, current_deriv, obj.schemeData.uMode, NaN);
        end
        
        function [value] = get_value(obj, x)
            value = eval_u(obj.grid, obj.valueFun, x);
        end
        
        function [state] = use_avoid_control(obj, x)
            uOpt = obj.get_avoid_u(x); 
            obj.dynSys.updateState(uOpt, obj.dt, x); 
            state = obj.dynSys.x; 
        end
        
        % Gets the min and max values in the value function
        function [min_val, max_val] = get_min_and_max_vals(obj)
            min_val = min(obj.valueFun, [], 'all');
            max_val = max(obj.valueFun, [], 'all');
        end
    end
end

