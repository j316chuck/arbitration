function [params] = load_env()
    %% Read in Map Data
    map_name = './maps/bookstore.png';
    map_yaml = './maps/bookstore.yaml';
    map_data = yaml.ReadYaml(map_yaml);
    obs_data_2d = imread(map_name); 
    obs_data_2d = double(obs_data_2d) / 255;
    params.map_data = map_data;
    params.obs_data = obs_data_2d;

    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [map_data.origin{1}, map_data.origin{2}];
    gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img);
    params.grid_img = grid_img;
   
    % create grid representing the new image size
    gmin_2d = [-10; -10];
    gmax_2d = [5.4; 5.4];
    gnum_2d = [41; 41];
    grid_2d = createGrid(gmin_2d, gmax_2d, gnum_2d);
    params.grid_2d = grid_2d;
    
    % Define 3D grid
    gmin_3d = [gmin_2d; -pi];   % Lower corner of computation domain
    gmax_3d = [gmax_2d; pi];    % Upper corner of computation domain
    gnum_3d = [gnum_2d; 16];    % Number of grid points per dimension
    pdDim = 3;                  % 3rd dimension is periodic
    grid_3d = createGrid(gmin_3d, gmax_3d, gnum_3d, pdDim);
    params.grid_3d = grid_3d; 
    
    % create an array of points at which to interpolate old image
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, gnum_2d');
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +0 for free 1 for occupied
    params.occ_map = occ_map; 
    % +1 for free -1 for occupied 
    binary_occ_map = 1 - 2 * occ_map;
    params.binary_occ_map = binary_occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, binary_occ_map);
    masked_obs_map = signed_obs_map .* occ_map;
    params.obs_map = signed_obs_map;
    params.masked_obs_map = masked_obs_map;
    
    % reach avoid dynSys
    xstart = [1; 1; 0];
    wMax = 1;
    vRange = [1, 1.5];
    dMax = [0.1; 0.1];
    reachAvoidDynSys = Plane(xstart, wMax, vRange, dMax); 
    params.reachAvoidDynSys = reachAvoidDynSys; 
    
    % avoid brs dynSys
    xstart = [1; 1; 0];
    wMax = 1;
    vRange = [1, 1.5];
    dMax = [0.1; 0.1];
    avoidBrsDynSys = Plane(xstart, wMax, vRange, dMax); 
    params.avoidBrsDynSys = avoidBrsDynSys; 
    
    % spline Planner DynSys
    xstart = [1; 1; 0];
    wMax = 1;
    vRange = [1, 1.5];
    dMax = [0.1; 0.1];
    splineDynSys = Plane(xstart, wMax, vRange, dMax); 
    params.splineDynSys = splineDynSys; 
    
    % reach avoid schemeData
    reachAvoidSchemeData.dynSys = params.reachAvoidDynSys;
    reachAvoidSchemeData.grid = grid_3d;
    reachAvoidSchemeData.uMode = 'min';
    reachAvoidSchemeData.dMode = 'max';
    params.reachAvoidSchemeData = reachAvoidSchemeData;
    
    % avoid brs schemeData
    avoidBrsSchemeData.dynSys = params.avoidBrsDynSys;
    avoidBrsSchemeData.grid = grid_3d;
    avoidBrsSchemeData.uMode = 'max';
    avoidBrsSchemeData.dMode = 'min';
    params.avoidBrsSchemeData = avoidBrsSchemeData; 
    
end 