classdef SafetyModule < handle
    % AVOIDSET Computes and stores an avoid set.
    % Find the set of all states that can avoid a given set of	
    % states despite the disturbance for a time	duration of	T
    % 
    % min_u max_d min_{t \in [0,T]} l(x(t))
    %   s. t.   \dot{x} = f(x,u,d,t)
    %           L = {x : l(x) <= 0}
    
    properties
        % Computation grid & dynamics
        grid            % (obj) Computation grid struct
        dt              % (float) Timestep in discretization
        computeTimes    % (float arr) Stores the computation times
        dynSys          % (obj) Dynamical system (dubins car)
        uMode           % (string) is control max or min-ing l(x)? 
        dMode           % (string) is disturbance max or min-ing l(x)?
        
        % Cost function representation
        lCurr           % (float arr) Current cost function representation
                        %             (constructed from measurements)
        
        % Value function computation information
        valueFun        % (float arr) Stores most recent converged value function 
        lastValueFun    % (float arr) Stores the last time stamp of the value function
        derivValueFun   % (float arr) Stores the derivative of the last value function
        timeDisc        % (float arr) Discretized time vector          
        schemeData      % (struct) Used to specify dyn, grid, etc. for HJIPDE_solve()
        HJIextraArgs    % (struct) Specifies extra args to HJIPDE_solve()
        updateEpsilon   % (float) Value change threshold used in local update rule
        warmStart       % (bool) if we want to warm start with prior V(x)
        firstCompute    % (bool) flag to see if this is the first time we have done computation
        updateMethod    % (string) what kind of solution method to use
        envType         % (string) what kind of environment are we doing safety for?
        
        % Utility variables and flags
        valueFunCellArr % (cell arr) stores sequence of V(x) 
        lxCellArr       % (cell arr) stores sequence of l(x)
        QSizeCellArr    % (cell arr) all queue size achieved for each timestep
        fovCellArr      % (cell arr) stores the results of FMM
        solnTimes       % (array) total time to compute solution at each step (in seconds)
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar or Plane4D dynamics!
        function obj = SafetyModule(grid, dynSys, uMode, dMode, dt, ...
                updateEpsilon, warmStart, updateMethod, tau)
            % Setup computation grid.
            obj.grid = grid;
            % Setup dynamical system.
            obj.dynSys = dynSys;
            obj.uMode = uMode;
            % Setup distrubance if we are computing with it.
            if ~isempty(dMode)
                obj.dMode = dMode;
            end
            
            obj.dt = dt;
            obj.updateEpsilon = updateEpsilon;
            obj.warmStart = warmStart;
            % Specify which update method we are using.
            obj.updateMethod = updateMethod;
            % Set the time discretization
            obj.timeDisc = tau; 

            obj.computeTimes = [];
            
            % Store the current estimate of the cost function (from
            % sensing).
            obj.lCurr = [];
            
            % Before the problem starts, dont initialize the value
            % function.
            obj.valueFun = NaN;
            obj.lastValueFun = NaN; 
            obj.derivValueFun = NaN; 
            
            % flag to tell us if this is the first time we have computed
            % lCurr or valueFun
            obj.firstCompute = true;
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dynSys;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = obj.uMode;
            if ~isempty(dMode)
                obj.schemeData.dMode = obj.dMode;
            end
            
            % Save out sequence of value functions as system moves through
            % space as well as the cost functions and the max Q size (if using Q method). 
            obj.valueFunCellArr = [];
            obj.lxCellArr = [];
            obj.QSizeCellArr = [];
            obj.solnTimes = [];
            obj.fovCellArr = [];
            
            % since we have a finite compute grid, we may not want to 
            % trust values near the boundary of grid
            obj.HJIextraArgs.ignoreBoundary = 0; 
            obj.HJIextraArgs.quiet = true;
            
            % Convergence information
            if strcmp(obj.updateMethod, 'HJI')
                obj.HJIextraArgs.stopConverge = 1;
                obj.HJIextraArgs.convergeThreshold = obj.updateEpsilon;
            elseif strcmp(obj.updateMethod, 'localQ')
                obj.HJIextraArgs.stopConverge = 0;
                if obj.grid.dim == 3
                    obj.schemeData.hamFunc = @dubins3Dham_localQ;
                    obj.schemeData.partialFunc = @dubins3Dpartial_localQ;
                elseif obj.grid.dim == 4 %4D 
                    obj.schemeData.hamFunc = @plane4Dham_localQ;
                    obj.schemeData.partialFunc = @plane4Dpartial_localQ;
                else
                    error('I cannot run safety computation with a %dD system!', obj.grid.dim);
                end
            else
                msg = strcat('Your update method: ', obj.updateMethod, ... 
                    'is not a valid option.');
                error(msg);
            end
            
            fprintf('------ Avoid Set Problem Setup -------\n');
            fprintf('   dynamical system: %dD\n', length(obj.grid.N));
            fprintf('   update method: %s\n', obj.updateMethod);
            fprintf('   warm start: %d\n', obj.warmStart);
            fprintf('   stopConverge: %d\n', obj.HJIextraArgs.stopConverge);
            fprintf('   updateEpsilon: %.3f\n', obj.updateEpsilon);
            fprintf('--------------------------------------\n');
            
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   occuMap             - (2D array) of occupancies in environment
        %                          -1 obs, +1 free
        %   gMap                - grid structure associated with occuMap
        % Outputs:
        %   dataOut             - infinite-horizon (converged) value function 
        function dataOut = computeAvoidSet(obj, signedDist)
            % Store the signed distance based on sensing info 
            % (for plotting, analysis etc.)
            %obj.fovCellArr{end+1} = signedDist;
            
            % ------------- CONSTRUCT l(x) ----------- %
            lxOld = obj.lCurr;
            if isempty(obj.lCurr) %|| strcmp(obj.envType, 'slam')
                % SLAM always gives us the full history of what the system 
                % has seen to be free, so just record that corresponding
                % signed distance function.
                obj.lCurr = signedDist;
            else
                obj.lCurr = shapeIntersection(signedDist, obj.lCurr);
            end
            
            % ------------- CONSTRUCT V(x) ----------- %
            if obj.firstCompute
                % First time we are doing computation, set data0 to lcurr
                % no matter our update method. 
                data0 = obj.lCurr;
            else
                if obj.warmStart
                    % If we are warm starting, use the old value function
                    % as initial V(x) and then the true/correct l(x) in targets
                    if obj.grid.dim == 3
                        % if our system is 3D
                        data0 = obj.valueFun(:,:,:,end);
                    elseif obj.grid.dim == 4
                        % if our system is 4D
                        data0 = obj.valueFun(:,:,:,:,end);
                    else
                        error('Cannot update safe set for %dD system!', obj.grid.dim);
                    end
                else
                    data0 = obj.lCurr;
                end
            end

            % Make sure that we store the true cost function 
            % so we can do min with L.
            obj.HJIextraArgs.targets = obj.lCurr;
            
            % We can use min with l regardless of if we are warm
            % starting (but we need to set targets = l!) 
            minWith = 'minVWithL';
            
            % ------------ Compute value function ---------- % 
            tic 
            if obj.firstCompute 
                firstHJIextraArgs = obj.HJIextraArgs;
                firstHJIextraArgs.stopConverge = 1;
                firstHJIextraArgs.convergeThreshold = obj.updateEpsilon;
                if obj.grid.dim == 3
                    firstHJIextraArgs.visualize.plotData.plotDims = [1 1 0];
                    firstHJIextraArgs.visualize.plotData.projpt = (0);
                elseif obj.grid.dim == 4
                    firstHJIextraArgs.visualize.plotData.plotDims = [1 1 0 0];
                    firstHJIextraArgs.visualize.plotData.projpt = [0 0.5];
                else
                    error('Unsure what states to project for visualization for %dD system.', ...
                        obj.grid.dim);
                end
                firstHJIextraArgs.visualize.valueSet = 1;
                firstWarmStart = false;
                [dataOut, tau, extraOuts] = ...
                HJIPDE_solve_warm(data0, lxOld, obj.lCurr, ...
                  obj.timeDisc, obj.schemeData, minWith, ...
                  firstWarmStart, firstHJIextraArgs);
            elseif strcmp(obj.updateMethod, 'HJI') 
                % Use typical HJI solver (with or without warm start).
                %obj.HJIextraArgs.targetFunction = zeros(size(data0)); 
                %[dataOut, tau, extraOuts] = ...
                %HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, ...
                %   minWith, obj.HJIextraArgs);
                [dataOut, tau, extraOuts] = ...
                HJIPDE_solve_warm(data0, lxOld, obj.lCurr, ...
                   obj.timeDisc, obj.schemeData, minWith, ...
                   obj.warmStart, obj.HJIextraArgs);    
            elseif strcmp(obj.updateMethod, 'localQ')
                % Use Q-based algorithm with initial Q constructed
                % *locally* near newly sensed regions.
                [dataOut, tau, extraOuts] = ...
                  HJIPDE_solve_localQ(data0, lxOld, obj.lCurr, ...
                    obj.updateEpsilon, obj.timeDisc, obj.schemeData, ...
                    minWith, obj.HJIextraArgs);
            end
            total_compute_t = toc;

            % only save out the final, 'converged' value function
            if obj.grid.dim == 3
                obj.lastValueFun = dataOut(:, :, :, end); 
                obj.valueFunCellArr{end+1} = obj.lastValueFun;
            else
                obj.lastValueFun = dataOut(:, :, :, :, end); 
                obj.valueFunCellArr{end+1} = obj.lastValueFun;
            end
            obj.derivValueFun = computeGradients(obj.grid, obj.lastValueFun);
            obj.lxCellArr{end+1} = obj.lCurr;
            
            % Update internal variables.
            obj.valueFun = dataOut;
            obj.computeTimes = tau;
            obj.solnTimes = [obj.solnTimes, total_compute_t];
            if exist('extraOuts', 'var') && isfield(extraOuts, 'QSizes')
                obj.QSizeCellArr{end+1} = extraOuts.QSizes;
            else
                warning('No field extraOuts.QSizes!');
            end
            
            % We've computed our first BRT.
            obj.firstCompute = false;
        end
        
        %% Checks if state x is at the safety boundary. If it is, returns
        %  the optimal safety control to take. 
        function [uOpt, onBoundary] = checkAndGetSafetyControl(obj, x, tol)
            % Grab the value at state x from the most recent converged 
            % value function.
            value = obj.get_value(x); 
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                uOpt = obj.get_avoid_u(x);  
                onBoundary = true;
            else
                uOpt = zeros(2, 1);
                onBoundary = false;
            end
        end
        
        function [value] = get_value(obj, x)
            value = eval_u(obj.grid, obj.lastValueFun, x);
        end
        
        function [state] = use_avoid_control(obj, x)
            uOpt = obj.get_avoid_u(x); 
            obj.dynSys.updateState(uOpt, obj.dt, x); 
            state = obj.dynSys.x; 
        end
        
        % Gets the min and max values in the value function
        function [min_val, max_val] = get_min_and_max_vals(obj)
            min_val = min(obj.lastValueFun, [], 'all');
            max_val = max(obj.lastValueFun, [], 'all');
        end
        
        %% Gets the optimal control to apply at state x.
        function uOpt = get_avoid_u(obj, x)
            current_deriv = eval_u(obj.grid, obj.derivValueFun, x);
            % NOTE: need all 5 arguments (including NaN's) to get 
            % correct optimal control!
            uOpt = obj.dynSys.optCtrl(NaN, x, current_deriv, obj.uMode, NaN);
        end
        
    end
end

