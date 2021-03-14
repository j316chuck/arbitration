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
        updateMethod
    end
    
    methods
        %% Constructs BRS Avoid Planner.
        function obj = BRSAvoidPlanner(grid_3d, schemeData, tau, dt, updateMethod)
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
            obj.valueFun = [];
            obj.obstacle_map = []; 
            % Extra Args
            obj.extraArgs.quiet = true;
            obj.extraArgs.visualize.figNum = 15;
            obj.extraArgs.visualize.deleteLastPlot = true;
            obj.extraArgs.visualize.viewGrid = false;
            obj.extraArgs.visualize.initialValueSet = 1;
            obj.extraArgs.visualize.valueSet = 1;
            obj.extraArgs.plotData.plotDims = [1 1 0];
            obj.extraArgs.plotData.projpt = 0;
            obj.extraArgs.convergeThreshold = 2e-2;
            obj.extraArgs.stopConverge = true;
            obj.extraArgs.stopConvergeQSize = 10; 
            obj.extraArgs.quiet = false; 
            % set updateMethod
            obj.updateMethod = updateMethod; % HJIPDE, warm_start, local_q 
        end
        
        %% Solves the brs avoid problem
        function solve_brs_avoid(obj, obstacle_map)
            if strcmp(obj.updateMethod, 'HJIPDE')
                obj.solve_brs_avoid_HJIPDE(obstacle_map); 
            elseif strcmp(obj.updateMethod, 'warm_start')
                obj.solve_brs_avoid_warm_start(obstacle_map); 
            elseif strcmp(obj.updateMethod, 'local_q')
                obj.solve_brs_avoid_local_q(obstacle_map); 
            else
                warning("invalid solver %s", obj.updateMethod); 
            end 
        end 
        
        function solve_brs_avoid_HJIPDE(obj, new_obstacle_map)
            if ~isequal(size(new_obstacle_map), obj.grid.N')
                error("Shape of obstacle_map is not equal to the grid shape");
            end
            obj.obstacle_map = new_obstacle_map;
            % Solve 
            obj.extraArgs.targets = new_obstacle_map;  
            [obj.data, obj.data_tau, ~] = ...
                HJIPDE_solve(obj.obstacle_map, obj.tau, obj.schemeData, 'minVWithL', obj.extraArgs);
            obj.valueFun = obj.data(:, :, :, end);
            obj.derivValueFun = computeGradients(obj.grid, obj.valueFun);
        end 
       
        %% Solves the brs avoid problem local Q
        function solve_brs_avoid_local_q(obj, new_obstacle_map)
            if ~isequal(size(new_obstacle_map), obj.grid.N')
                error("Shape of obstacle_map is not equal to the grid shape");
            end
            if isempty(obj.valueFun) % first computation default to original HJIPDE
                obj.solve_brs_avoid_HJIPDE(new_obstacle_map); 
                return;
            else 
                data0 = obj.valueFun;
                lCurr = shapeIntersection(new_obstacle_map, obj.obstacle_map);
                lxOld = obj.obstacle_map;
            end
            updateEpsilon = obj.extraArgs.convergeThreshold; 
            obj.extraArgs.targets = lCurr;
            [obj.data, obj.data_tau, ~] = ...
                HJIPDE_solve_localQ(data0, lxOld, lCurr, updateEpsilon, obj.tau, obj.schemeData, 'minVWithL', obj.extraArgs);   
            obj.obstacle_map = new_obstacle_map; 
            obj.valueFun = obj.data(:, :, :, end);
            obj.derivValueFun = computeGradients(obj.grid, obj.valueFun);
        end 
        
        %% Solves the brs avoid problem warm start
        function solve_brs_avoid_warm_start(obj, new_obstacle_map)
            if ~isequal(size(new_obstacle_map), obj.grid.N')
                error("Shape of obstacle_map is not equal to the grid shape");
            end
            % Solve 
            warmStart = true;
            if isempty(obj.valueFun)
                data0 = new_obstacle_map; 
                lCurr = new_obstacle_map; 
                lxOld = []; 
            else 
                data0 = obj.valueFun;
                lCurr = shapeIntersection(new_obstacle_map, obj.obstacle_map);
                lxOld = obj.obstacle_map;
            end
            obj.extraArgs.targets = lCurr;
            [obj.data, obj.data_tau, extraOuts] = ...
                HJIPDE_solve_warm(data0, lxOld, lCurr, obj.tau, obj.schemeData, 'minVWithL', warmStart, obj.extraArgs);  
            %obj.plot_q_size(extraOuts.QSizes); 
            obj.obstacle_map = new_obstacle_map; 
            obj.valueFun = obj.data(:, :, :, end);
            obj.derivValueFun = computeGradients(obj.grid, obj.valueFun);
        end 
        
        function plot_q_size(obj, QSizes)
            figure(11); 
            plot(1:length(QSizes), QSizes, 'ro--', 'DisplayName', 'Q size'); 
            xlabel("num iterations"); 
            ylabel("Q size"); 
            title("Q size converge plot");
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

