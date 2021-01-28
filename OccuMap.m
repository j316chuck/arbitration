classdef OccuMap < handle
    % OccuMap represents a 2D occupancy map of an environment.
    properties
        grid            % (obj) Computation grid struct
        grid_2d         % 2D projection of grid struct
        occ_map         % +1 for free, 0 for obstacle 
        binary_occ_map  % +1 for obstacle, -1 for free
        signed_obs_map  % FMM output of binary_occ_map
        masked_obs_map  % 0 for free, positive for obstacle
        first_compute   % (bool) flag to see if this is the first time we have done computation
        lReal           % (float arr) Cost function representing TRUE environment
        lBoundary       % (floar arr) Boundary obstacle for augmentation. 
        boundLow        % (x,y) lower corner of boundary obstacle (for numerics)
        boundUp         % (x,y) upper corner of boundary obstacle (for numerics)
        
        gFMM                    % Computation grid struct for FMM
        signed_dist_safety      % signed distance function representing sensed environment
        planner_obs_map         % 
        occupancy_map_safety    % (array) occupancy map used by safety module with -1 (obs) +1 (free)
        occupancy_map_plan      % (array) occupancy map ysed by planning with -1 (obs) +1 (free)        
        occuMapSafeCellArr      % (cell arr) saves out the set of sensed states.
        occuMapPlanCellArr      % (cell arr) saves out the occupancy map used by planner.
        trajectory              % (2 x N) array stores the 2D positions of the robot
    end
    
    methods
        %% Constructor
        % Inputs:
        %   grid        -- (struct) computation grid struct with min/max,
        %                   dimensions, etc.
        %   grid_2d    -- (struct) computation grid struct with min/max,
        %                   dimensions, etc.
        %   occ_map     -- (arr) 2D array representing obstacle map of
        %                   environment +1 for free, 0 for obstacle
        function obj = OccuMap(grid, grid_2d, occ_map)
            % Computation grid
            obj.grid = grid;
            obj.grid_2d = grid_2d; 
            obj.occ_map = occ_map; % +1 for free, 0 for obstacle 
            obj.binary_occ_map = 2 * occ_map - 1; % +1 for free -1 for occupied
            fmm_map = compute_fmm_map(grid_2d, binary_occ_map);
            obj.signed_obs_map = -fmm_map; % positive inside obstacle, negative outside
            obj.masked_obs_map = obj.signed_obs_map .* (1-occ_map); % positive inside obstacle, 0 outside
            obj.first_compute = true;
            % obj.augment_map(); 
            
            % Create ground-truth 
            if obj.grid.dim == 3
                obj.lReal = repmat(signed_obs_map, 1, 1, obj.grid.N(3));
            elseif obj.grid.dim == 4
                obj.lReal = repmat(signed_obs_map, 1, 1, obj.grid.N(3), obj.grid.N(4));
            else
                error('environment is not supported right now for %dD system.\n', obj.grid.dim);
            end
            
            % Obstacle maps
            obj.gFMM                 = [];
            obj.signed_dist_safety   = [];
            obj.occupancy_map_safety = [];
            obj.occupancy_map_plan   = ones(transpose(obj.grid.N(1:2))); % everything is initially free for the planner
            obj.occuMapSafeCellArr   = {};
            obj.occuMapPlanCellArr   = {};
            obj.trajectory           = []; 
        end
        
        function augment_map(obj)
            % For numerics -- add a 1-grid-cell-sized obstacle along the 
            % edge of the compute grid. 
            offsetX = (obj.grid.max(1) - obj.grid.min(1))/obj.grid.N(1);
            offsetY = (obj.grid.max(2) - obj.grid.min(2))/obj.grid.N(2);
            if obj.grid.dim == 3
                obj.boundLow = [obj.grid.min(1)+offsetX, obj.grid.min(2)+offsetY, obj.grid.min(3)];
                obj.boundUp = [obj.grid.max(1)-offsetX, obj.grid.max(2)-offsetY, obj.grid.max(3)];
                % NOTE: need to negate the default shape function to make sure
                %       compute region is assigned (+) and boundary obstacle is (-)
                obj.lBoundary = -shapeRectangleByCorners(obj.grid,obj.boundLow,obj.boundUp);
            elseif obj.grid.dim == 4
                obj.boundLow = [obj.grid.min(1)+offsetX, obj.grid.min(2)+offsetY];
                obj.boundUp = [obj.grid.max(1)-offsetX, obj.grid.max(2)-offsetY];
                [g2D, ~] = proj(obj.grid, obj.grid.xs{1}, [0 0 1 1], [0 0]);
                obj.lBoundary = -shapeRectangleByCorners(g2D,obj.boundLow,obj.boundUp);
                obj.lBoundary = repmat(obj.lBoundary, 1, 1, obj.grid.N(3), obj.grid.N(4));
            else
                error('Cannot construct bounds for %dD system!', obj.grid.dim);
            end
            % Incorporate boundary obstacle into the ground-truth obstacle
            obj.lReal = shapeUnion(obj.lReal, obj.lBoundary);
        end 
        
        function plot_occ_map(obj)
            figure(2); 
            hold on;
            
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            title("Occ map"); 
        end
        
        function plot_traj(obj, xs, ys, ths, color, name)
        end 
        %% Based on sensing, update occupancy grids and signed distances.
        % Inputs:
        %   senseData      - [vector] if rectangle sensing region, 
        %                    (x,y) coords of lower left sensing box and
        %                    (x,y) coords of upper right sensing box.
        %                    [vector] if circle sensing region, 
        %                    (x,y) coords of center and radius
        %                    [array] if environment type is SLAM, 
        %                    then senseData contains the full new occupancy
        %                    map generated from SLAM.
        %   senseShape [string]     - type of sensing region. 
        %                             options: 'rectangle', 'circle',
        %                                      'lidar', 'camera'
        function updateMapAndCost(obj, senseData, senseShape)
            % Construct cost function for region outside sensing radius
            % or 'known' environment.
           if strcmp(senseShape, 'circle') % if circular sensing region
                center = senseData{1}(1:2);
                radius = senseData{2}(1);

                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)
                if obj.grid.dim == 3
                    sensingShape = -shapeCylinder(obj.grid, 3, center, radius);
                    [~, dataSense] = proj(obj.grid, sensingShape, [0 0 1], 0);
                else % 4D
                    sensingShape = -shapeCylinder(obj.grid, [3,4], center, radius);
                    [~, dataSense] = proj(obj.grid, sensingShape, [0 0 1 1], [0, 0]);
                end
                %obj.sensed_region = max(obj.sensed_region, sign(dataSense));

                % Update occupancy grid with newly sensed obstacles.
                sensedIndicies = find(dataSense > 0);
                obj.occupancy_map_plan(sensedIndicies) = obj.occupancy_map_safety(sensedIndicies); 
                %[~,realObs] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                %obj.occupancy_map_plan(find((obj.sensed_region > 0).*(realObs < 0))) = -1;

                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                if obj.grid.dim == 3
                    [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                else
                    [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1 1], [0, 0]);
                end

                % also: subtract small epsilon in case we get zero's
                epsilon = 1e-6;
                obj.occupancy_map_safety = sign(dataFMM-epsilon);

            elseif strcmp(senseShape, 'camera') % if camera sensing region
                % It is assumed that the obstacle is only position
                % dependent in this computation.

                % If we are in simulation and this is the first computation, 
                % use a circle; otherwise use a camera
                if obj.first_compute 

                  % Record which states we have sensed. 
                  % (+1 sensed, -1 unsensed)
                  if obj.grid.dim == 3
                    sensingShape = -shapeCylinder(obj.grid, 3, senseData{1}(1:2), senseData{2}(2));
                    [g2D, dataSense] = proj(obj.grid, sensingShape, [0 0 1], 0);
                  else % 4D
                    sensingShape = -shapeCylinder(obj.grid, [3,4], senseData{1}(1:2), senseData{2}(2));
                    [g2D, dataSense] = proj(obj.grid, sensingShape, [0 0 1 1], [0, 0]);
                  end
                  %obj.sensed_region = max(obj.sensed_region, sign(dataSense));

                  % Union the sensed region with the actual obstacle.
                  unionL = shapeUnion(sensingShape, obj.lReal);
                  % Project the slice of unionL and create an occupancy map
                  if obj.grid.dim == 3
                    [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                  else
                    [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1 1], [0, 0]);
                  end

                  % also: subtract small epsilon in case we get zero's
                  epsilon = 1e-6;
                  obj.occupancy_map_safety = sign(dataFMM-epsilon);

                  % Do the actual camera sensing next time.
                  obj.first_compute = false;

                  % Update occupancy grid with newly sensed obstacles.
                  sensedIndicies = find(dataSense > 0);
                  obj.occupancy_map_plan(sensedIndicies) = obj.occupancy_map_safety(sensedIndicies); 
                else
                  % Project the slice of obstacle
                  if obj.grid.dim == 3
                    [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                  else
                    [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1 1], [0, 0]);
                  end
                  [obj.occupancy_map_safety, dataSense] = ...
                      generate_camera_sensing_region(obj.gFMM, obsSlice, ...
                      senseData{2}(1), senseData{1}(1:2), senseData{1}(3), senseData{2}(3));

                  % Record which states we have sensed. 
                  % (+1 sensed, -1 unsensed)                  
                  %obj.sensed_region = max(obj.sensed_region, dataSense);

                  % Update occupancy grid with newly sensed obstacles.
                  obj.occupancy_map_plan = -dataSense;
                  sensedIndicies = find(dataSense > 0);
                  obj.occupancy_map_plan(sensedIndicies) = obj.occupancy_map_safety(sensedIndicies);
                end
            elseif strcmp(senseShape, 'lidar') % if lidar sensing region
                % It is assumed that the obstacle is only position
                % dependent in this computation.
                % Project the slice of obstacle
                if obj.grid.dim == 3
                    [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                else
                    [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1 1], [0, 0]);
                end
                [obj.occupancy_map_safety, dataSense] = ...
                    generate_lidar_sensing_region(obj.gFMM, obsSlice, ...
                    senseData{2}(1), senseData{1}(1:2));

                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)                  
                %obj.sensed_region = max(obj.sensed_region, dataSense);

                % Update occupancy grid with newly sensed obstacles.
                sensedIndicies = find(dataSense > 0);
                obj.occupancy_map_plan(sensedIndicies) = obj.occupancy_map_safety(sensedIndicies); 
            else
               error('Unrecognized sensor type');
            end
 
            % Save out the occupancy maps for plotting.
            obj.occuMapSafeCellArr{end+1} = obj.occupancy_map_safety;
            obj.occuMapPlanCellArr{end+1} = obj.occupancy_map_plan;
            obj.trajectory = [obj.trajectory, senseData{1}(1:2)]; 
            
            % We will use the FMM code to get the signed distance function. 
            % Since the FMM code works only on 2D, we will take a slice of 
            % the grid, compute FMM, and then project it back to a 3D array.
            unionL_2D_FMM = compute_fmm_map(obj.gFMM, obj.occupancy_map_safety);

            if obj.grid.dim == 3
                obj.signed_dist_safety = repmat(unionL_2D_FMM, 1, 1, obj.grid.N(3));
            else
                obj.signed_dist_safety = repmat(unionL_2D_FMM, 1, 1, obj.grid.N(3), obj.grid.N(4));
            end
        end
    end    
end

