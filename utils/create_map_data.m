function exp = create_map_data(params)
    %% Read in Map Data
    exp.map_basename = params.map_basename; 
    exp.map_yaml = params.map_yaml;
    exp.map_name = params.map_name; 
    map_data = yaml.ReadYaml(exp.map_yaml);
    obs_data_2d = imread(exp.map_name); 
    obs_data_2d = double(obs_data_2d) / 255;
    exp.map_data = map_data;
    exp.obs_data = obs_data_2d;
    exp.grid_size = params.grid_size; 

    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [map_data.origin{1}, map_data.origin{2}];
    gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img);
    exp.grid_img = grid_img;
   
    %% Define 3D grid
    gmin_3d = params.gmin_3d;  % Lower corner of computation domain
    gmax_3d = params.gmax_3d;  % Upper corner of computation domain
    gnum_3d = params.gnum_3d;  % Number of grid points per dimension
    pdDim = 3;                 % 3rd dimension is periodic
    grid_3d = createGrid(gmin_3d, gmax_3d, gnum_3d, pdDim);
    exp.grid_3d = grid_3d; 
    
    %% Create 2D grid
    gmin_2d = params.gmin_3d(1:2);
    gmax_2d = params.gmax_3d(1:2);
    gnum_2d = params.gnum_3d(1:2);
    grid_2d = createGrid(gmin_2d, gmax_2d, gnum_2d);
    exp.grid_2d = grid_2d;
   
    %% Create obstacle maps
    % create an array of points at which to interpolate old image
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, gnum_2d');
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +1 for free 0 for occupied
    exp.known_occ_map = occ_map; 
    % +1 for free -1 for occupied 
    binary_occ_map = 2 * occ_map - 1;
    exp.binary_occ_map = binary_occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, binary_occ_map);
    % positive inside obstacle, zero within free space
    masked_obs_map = -signed_obs_map .* (1-occ_map);
    % fmm map, negative inside obstacle, positive inside free space
    exp.obs_map = signed_obs_map;
    exp.masked_obs_map = masked_obs_map;
    exp.obstacle = repmat(exp.obs_map, 1, 1, exp.grid_3d.N(3));
    
    %% Create occupancy map data structure
    exp.sensorArgs.sensor_shape = 'camera'; 
    exp.sensorArgs.sensor_radius = 2;
    exp.sensorArgs.sensor_fov = pi/6; 
    exp.sensorArgs.far_plane = 20; 
    exp.unknown_occ_map = OccuMap(grid_3d, grid_2d, occ_map, exp.sensorArgs); 
end 