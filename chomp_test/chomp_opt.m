clear all
close all

%% Create start and goal traj
num_waypts = 30;
N = 151;    % discrete number of states in x and y dimensions.  
sx = 120; %120; %110;    % start x-state
sy = 60; %60; %30;   % start y-state
gx = 60; %60; %70;    % goal x-state
gy = 60; %120; %90;    % goal y-state

%% Compute the signed distance to the obstacles.
grid_min = [1,1];
grid_max = [N,N];
n_disc = [N,N];
g = createGrid(grid_min, grid_max, n_disc);

% Obstacle based on SHAPES.
% obs_cost = shapeCylinder(g, [], [50,60], 60);

% Obstacle based on occupancy map IMAGE.
obs_cost = -1 * load_occ_map(grid_min, grid_max, n_disc);

% Plotting...
figure(1)
hold on
imagesc(obs_cost)
contour(g.xs{1}, g.xs{2}, obs_cost, [0,0.01], 'r');
colorbar

%% Compute gradient in x and y dimensions of the obstacle cost. 
grad_x = diff(obs_cost,1,1);
grad_y = diff(obs_cost,1,2);

%% Create straight line traj as initial plan.
traj_p = straight_traj(sx,sy,gx,gy,num_waypts);
figure(1)
hold on
plot(traj_p(1,:),traj_p(2,:),'k');
scatter(sx, sy, 'g', 'filled');
scatter(gx, gy, 'r', 'filled');

%% Compute the optimal trajectory!
step_size = 0.3;
smoothness_param = 0.2;             % scale of smoothness
obstacle_param = 0.1;               % scale of value function 
num_iter = 150;                     % number of iterations of alg. 

% Setup initial trajectory and store sequence of trajs (for debugging).
traj = traj_p;
traj_progress = [traj];

% Make finite difference matrix.
N = size(traj_p,2);
K = zeros(N-1,N-1); % here, we let the endpoint be free. 
for i=1:N-1
    K(i,i) = 1;
    K(i+1,i) = -1;
end

%% Create the matrix used for the norm computation.
% OPTION 1: finite-difference based norm!
A = K' * K;

% OPTION 2: make the matrix just the identity!
% A = eye(size(A));

% Precompute matrix inverse.
Ainv = inv(A);

%% Start iteratively computing optimum!
for it = 1:num_iter
    % Smoothness gradient
    smoothness_grad = zeros(2,N);
    for i=2:N-1
        smoothness_grad(:,i) = 2*traj(:,i)-traj(:,i-1)-traj(:,i+1);
    end
    smoothness_grad = smoothness_grad(:,2:N);
    
    % Obstacle (i.e. value function) gradient
    obstacle_grad = zeros(2,N);
    for i=2:N-1
        obstacle_grad(1,i) = grad_x(round(traj(1,i)),round(traj(2,i)));
        obstacle_grad(2,i) = grad_y(round(traj(1,i)),round(traj(2,i)));
    end
    obstacle_grad = obstacle_grad(:,2:N);
    
    % compute the gradient.
    grad = obstacle_param*smoothness_grad + smoothness_param*obstacle_grad;
    
    % Multiply gradient with the norm matrix
    delta_x = Ainv * grad(1,:)';
    delta_y = Ainv * grad(2,:)';
    
    % Do the update!
    traj(:,2:N) = traj(:,2:N) - step_size * [delta_x'; delta_y'];
    
    % track trajectory progress
    traj_progress = [traj_progress; traj];
end

% Final plotting.
hold on;
plot(traj_p(1,:), traj_p(2,:), 'k-o');
plot(traj(1,:), traj(2,:), 'r-o');

set(gcf, 'color', 'w')
set(gcf, 'position', [100,100,800,800])

%% Loads occupoancy map & makes signed dist 
%  negative inside obstacle, positive inside free space
function obs_map = load_occ_map(gmin_2d, gmax_2d, gnum_2d)
    %map_basename = 'test';
    repo = what('arbitration');
    %map_yaml = strcat(repo.path, '/chomp_test/test.yaml');
    map_name = strcat(repo.path, '/chomp_test/test.png');
    
    % read in map.
    obs_data_2d = imread(map_name); 
    obs_data_2d = double(obs_data_2d) / 255;
    
    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [1, 1];
    res = 1;
    gmax_img = gmin_img + res * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img);
    
    % create grid representing the new image size
    grid_2d = createGrid(gmin_2d, gmax_2d, gnum_2d);

    % create an array of points at which to interpolate old image
    free_thresh = 0.0039;
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = obs_data_2d; %eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, gnum_2d);
    occ_map(occ_map < free_thresh) = 0;
    occ_map(occ_map >= free_thresh) = 1; 
    % +1 for free -1 for occupied 
    binary_occ_map = 2 * occ_map - 1;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, binary_occ_map);
    % positive inside obstacle, zero within free space
    masked_obs_map = -signed_obs_map .* (1-occ_map);
    
    % fmm map, negative inside obstacle, positive inside free space
    obs_map = signed_obs_map;
    masked_obs_map = masked_obs_map;
end

%% Creates straight line trajectory with num_waypts.
function traj = straight_traj(sx, sy, gx, gy, num_waypoints)

traj = zeros(2,num_waypoints);
traj(1,1)=sx;
traj(2,1)=sy;
dist_x = gx-sx;
dist_y = gy-sy;

for i=2:num_waypoints
    traj(1,i)=traj(1,i-1)+dist_x/(num_waypoints-1);
    traj(2,i)=traj(2,i-1)+dist_y/(num_waypoints-1);
end

end