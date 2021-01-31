classdef OccuMap < handle
    % OccuMap represents a 2D occupancy map of an environment.
    properties
        grid            % (obj) Computation grid struct
        grid_2d         % 2D projection of grid struct
        masked_free_map % 0 for obstacle, +1 for free
        masked_occ_map  % +1 for obstacle, 0 for free
        binary_occ_map  % +1 for free, -1 for obstacle
        
        first_compute           % (bool) flag to see if this is the first time we have done computation
        occupancy_map_safety    % (array) occupancy map used by safety module with -1 (obs) +1 (free), default -1
        occupancy_map_planner   % (array) occupancy map used by planning with -1 (obs) +1 (free), default +1
        occuMapSafeCellArr      % (cell arr) saves out the set of sensed states
        occuMapPlanCellArr      % (cell arr) saves out the occupancy map used by planner
        trajectory              % (3 x N) array stores the positions of the robot path
        safety_fmm              % 2D grid, negative inside obstacle, positive outside, fmm map of the occupancy_map_safety 
        planner_fmm             % 2D grid, negative inside obstacle, positive outside, fmm map of the occupancy_map_planner
        signed_dist_safety      % 3D grid, negative inside obstacle, positive outside, signed distance function of sensed safety environment used by HJIPDE
        signed_dist_planner     % 2D grid, positive inside obstacle, negative outside, signed distance function of planner environment used by spline planner
        
        sensor_shape;       % (string) camera, circle, or lidar. 
        sensor_radius;      % (float) radius of camera, circle, or lidar scans
        sensor_fov;         % (float) camera fov in radians
        far_plane;          % (float) the farthest the camera can sense
    end
    
    methods
        %% Constructor
        % Inputs:
        %   grid        -- (struct) computation grid struct with min/max,
        %                   dimensions, etc.
        %   grid_2d     -- (struct) computation grid struct with min/max,
        %                   dimensions, etc.
        %   occ_map     -- (arr) 2D array representing obstacle map of
        %                   environment +1 for free, 0 for obstacle
        function obj = OccuMap(grid, grid_2d, occ_map, extraArgs)
            % Computation grid
            obj.grid = grid;
            obj.grid_2d = grid_2d; 
            obj.masked_free_map = occ_map;        % 0 for obstacle, +1 for free
            obj.masked_occ_map = 1 - occ_map;     % +1 for obstacle, 0 for free
            obj.binary_occ_map = 2 * occ_map - 1; % +1 for free, -1 for obstacle
            
            % Obstacle maps
            obj.first_compute = true;
            obj.occupancy_map_safety  = -ones(obj.grid.N(1:2)');
            obj.occupancy_map_planner = ones(obj.grid.N(1:2)'); % everything is initially free for the planner
            obj.occuMapSafeCellArr    = {};
            obj.occuMapPlanCellArr    = {};
            obj.trajectory            = zeros(3, 0); 
            obj.safety_fmm            = []; 
            obj.planner_fmm           = [];
            obj.signed_dist_safety    = [];
            obj.signed_dist_planner   = []; 
            
            % Sensor args
            obj.sensor_shape = extraArgs.sensor_shape;
            if strcmp(obj.sensor_shape, 'circle')
                obj.sensor_radius = extraArgs.sensor_radius;
            elseif strcmp(obj.sensor_shape, 'camera')
                obj.sensor_radius = extraArgs.sensor_radius;
                obj.sensor_fov = extraArgs.sensor_fov; 
                obj.far_plane = extraArgs.far_plane; 
            elseif strcmp(obj.sensor_shape, 'lidar')
                obj.sensor_radius = extraArgs.sensor_radius;
            end 
        end
        
        %% Based on sensing, update occupancy grids and signed distances.
        % Inputs:
        %   robot_pos  [x, y, theta]
        function updateMapAndCost(obj, robot_pos)
            % Construct cost function for region outside sensing radius
            % or 'known' environment. +1 sensed, -1 unsensed 
            if strcmp(obj.sensor_shape, 'lidar') || obj.first_compute || strcmp(obj.sensor_shape, 'circle')
                [occ_grid, sensed_region] = generate_lidar_sensing_region(robot_pos(1:2), ...
                        obj.grid_2d, obj.binary_occ_map, obj.sensor_radius); 
            elseif strcmp(obj.sensor_shape, 'camera') 
                [occ_grid, sensed_region] = generate_camera_sensing_region(robot_pos, ...
                obj.grid_2d, obj.binary_occ_map, obj.sensor_fov, obj.far_plane);
            else
               error('Unrecognized sensor type');
            end
           
            % Update occupancy grid with newly sensed obstacles.
            sensed_free_inds = find(occ_grid > 0);
            obj.occupancy_map_safety(sensed_free_inds) = 1; % set safety occupied to free
            sensed_obs_inds = find((sensed_region > 0) .* obj.masked_occ_map); 
            obj.occupancy_map_planner(sensed_obs_inds) = -1; % set planner free to occupied
            obj.first_compute = false; 
 
            % Save out the occupancy maps and trajectories for plotting
            obj.occuMapSafeCellArr{end+1} = obj.occupancy_map_safety;
            obj.occuMapPlanCellArr{end+1} = obj.occupancy_map_planner;
            obj.trajectory = [obj.trajectory, reshape(robot_pos(1:3), [3, 1])]; 

            % Compute the signed dist maps
            obj.safety_fmm = compute_fmm_map(obj.grid_2d, obj.occupancy_map_safety);
            obj.planner_fmm = compute_fmm_map(obj.grid_2d, obj.occupancy_map_planner); 
            obj.signed_dist_safety = repmat(obj.safety_fmm, 1, 1, obj.grid.N(3));
            obj.signed_dist_planner = -obj.planner_fmm .* (obj.occupancy_map_planner < 0);
            %obj.plot_occ_map(); 
        end
        
        function plot_occ_map(obj)
            figure(2); 
            set(gcf,'Position', [10 10 1200 1000])
            clf
            % Plot safety occupancy map
            subplot(2, 2, 1); 
            hold on;
            contourf(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.occupancy_map_safety, [0 0], 'DisplayName', 'safety', 'color', 'blue');
            plot_traj(obj.trajectory(1, :), obj.trajectory(2, :), obj.trajectory(3, :), 'red', 'trajectory');
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Occ Map Safety");
            hold off;
            % Plot safety signed dist map
            subplot(2, 2, 2); 
            hold on;
            contourf(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.safety_fmm, 'DisplayName', 'safety');
            plot_traj(obj.trajectory(1, :), obj.trajectory(2, :), obj.trajectory(3, :), 'red', 'trajectory');
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Signed Dist Safety");
            hold off;
            % Plot planner occupancy map
            subplot(2, 2, 3); 
            hold on;
            contourf(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.occupancy_map_planner, [0 0], 'DisplayName', 'planner', 'color', 'blue');
            plot_traj(obj.trajectory(1, :), obj.trajectory(2, :), obj.trajectory(3, :), 'red', 'trajectory');
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Occ Map Planner");
            hold off;
            % Plot safety signed dist map
            subplot(2, 2, 4); 
            hold on;
            contourf(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.signed_dist_planner, 'DisplayName', 'planner');
            plot_traj(obj.trajectory(1, :), obj.trajectory(2, :), obj.trajectory(3, :), 'red', 'trajectory');
            contour(obj.grid_2d.xs{1}, obj.grid_2d.xs{2}, obj.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
            xlabel("x(m)"); 
            ylabel("y(m)"); 
            legend('Location', 'NorthWest', 'Interpreter', 'None');
            colorbar; 
            title("Signed Dist Planner");
            hold off;
        end
    end    
end

