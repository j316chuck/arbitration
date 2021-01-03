function [starts, goals] = generate_nav_goals(N, min_dist, regen_points)
    if regen_points
        p = default_hyperparams();
        generate_nav_points(p);
    end 
    load('./data/sampled_points.mat');
    xs = sampled_points(1, :);
    ys = sampled_points(2, :);
    ths = sampled_points(3, :);
    Ns = size(xs, 2);
    starts = [];
    goals = [];
    counts = 0;
    while counts < N
        s = randperm(Ns);
        g = randperm(Ns);
        si = s(1:N-counts);
        gi = g(1:N-counts);
        sx = xs(si);
        sy = ys(si);
        st = ths(si); 
        gx = xs(gi);
        gy = ys(gi);
        gt = ths(gi);        
        d = l2_dist(sx, gx, sy, gy); 
        valid_ind = d >= min_dist;
        sv = ones(sum(valid_ind), 1) * 0.01;
        ev = zeros(sum(valid_ind), 1) * 0.01;
        nstarts = [sx(valid_ind)', sy(valid_ind)', st(valid_ind)', sv];
        ngoals = [gx(valid_ind)', gy(valid_ind)', gt(valid_ind)', ev];
        starts = [starts; nstarts];
        goals = [goals; ngoals];
        counts = size(starts, 1);
    end 
    save('./data/sampled_goals.mat', 'starts', 'goals');
end 
 
function d = l2_dist(x1, x2, y1, y2)
    d =  ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
end 

function [sampled_points] = generate_nav_points(params)
    clf;
    exp.map_basename = params.map_basename; 
    exp.map_yaml = params.map_yaml;
    exp.map_name = params.map_name; 
    map_data = yaml.ReadYaml(exp.map_yaml);
    obs_data_2d = imread(exp.map_name); 
    obs_data_2d = double(obs_data_2d) / 255;
    exp.map_data = map_data;
    exp.obs_data = obs_data_2d;

    %% Resize the occupancy grid to fit the desired grid size
    % create grid representing the raw image 
    gmin_img = [map_data.origin{1}, map_data.origin{2}];
    gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
    gnum_img = size(obs_data_2d);
    grid_img = createGrid(gmin_img, gmax_img, gnum_img);
    exp.grid_img = grid_img;
    
    
    gmin_3d = params.gmin_3d;  % Lower corner of computation domain
    gmax_3d = params.gmax_3d;    % Upper corner of computation domain
    gnum_3d = params.gnum_3d;    % Number of grid points per dimension
    pdDim = 3;                  % 3rd dimension is periodic
    grid_3d = createGrid(gmin_3d, gmax_3d, gnum_3d, pdDim);
    exp.grid_3d = grid_3d; 
    
    % create grid representing the new image size
    gmin_2d = params.gmin_3d(1:2);
    gmax_2d = params.gmax_3d(1:2);
    gnum_2d = params.gnum_3d(1:2);
    grid_2d = createGrid(gmin_2d, gmax_2d, gnum_2d);
    exp.grid_2d = grid_2d;
    
    pts = [grid_2d.xs{1}(:), grid_2d.xs{2}(:)];
    % interpolate old map into newly-sized map
    occ_map = eval_u(grid_img, obs_data_2d, pts);
    occ_map = reshape(occ_map, gnum_2d');
    occ_map(occ_map < map_data.free_thresh) = 0;
    occ_map(occ_map >= map_data.free_thresh) = 1; 
    % +0 for free 1 for occupied
    exp.occ_map = occ_map; 
    % +1 for free -1 for occupied 
    binary_occ_map = 1 - 2 * occ_map;
    exp.binary_occ_map = binary_occ_map;
    % compute fmm map
    signed_obs_map = compute_fmm_map(grid_2d, binary_occ_map);
    masked_obs_map = signed_obs_map .* occ_map;
    exp.obs_map = signed_obs_map;
    exp.masked_obs_map = masked_obs_map;
    
    % sample points
    N = 1000;
    low_thresh = 0;
    high_thresh = 100;
    counts = 0;
    sampled_x = [];
    sampled_y = [];
    hold on;
    while counts < N
        x = sample_1d(gmin_3d(1), gmax_3d(1), N-counts);
        y = sample_1d(gmin_3d(2), gmax_3d(2), N-counts);
        pts = [x(:), y(:)];
        %u = eval_u(exp.grid_2d, exp.obs_map, pts);
        %valid_ind = logical((low_thresh <= u) .* (u <= high_thresh));

        u = eval_u(exp.grid_2d, exp.binary_occ_map, pts);
        valid_ind = u < low_thresh;
        invalid_ind = ~valid_ind;
        %scatter(x(valid_ind), y(valid_ind), 10, 'g');
        %scatter(x(invalid_ind), y(invalid_ind), 10, 'b');
        sampled_x = [sampled_x, x(valid_ind)'];
        sampled_y = [sampled_y, y(valid_ind)'];
        %scatter(sampled_x, sampled_y, 20, 'ro');
        counts = size(sampled_x, 2);
    end 
    sampled_t = sample_1d(gmin_3d(3), gmax_3d(3), N);
    
    sampled_points = [sampled_x; sampled_y; sampled_t'];
    
    % plot sampled points
    figure(5); 
    hold on 
    set(gcf,'Position',[10 10 1000 800])
    contour(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle');
    % plot points 
    probs = rand(size(sampled_x, 2));
    plot_traj(sampled_x, sampled_y, sampled_t', probs, 'sampled points') 
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    legend('Location', 'NorthWest');
    title('Sampled Points');
    save('./data/sampled_points.mat', 'sampled_points')
    savefig('./data/sampled_points.fig');
end 


function plot_traj(xs, ys, ths, probs, name)
    s = scatter(xs, ys, 15, 'black', 'filled');
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths));
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.DisplayName = name;
    q.AutoScaleFactor = 0.3;
end 

function set_quiver_colors(q, probs)
    %// Get the current colormap
    currentColormap = colormap(gca);

    %// Now determine the color to make each arrow using a colormap
    [~, ~, ind] = histcounts(probs, size(currentColormap, 1));

    %// Now map this to a colormap to get RGB
    cmap = uint8(ind2rgb(ind(:), currentColormap) * 255);
    cmap(:,:,4) = 255;
    cmap = permute(repmat(cmap, [1 3 1]), [2 1 3]);

    %// We repeat each color 3 times (using 1:3 below) because each arrow has 3 vertices
    set(q.Head, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:3,:,:), [], 4).');   %'

    %// We repeat each color 2 times (using 1:2 below) because each tail has 2 vertices
    set(q.Tail, ...
        'ColorBinding', 'interpolated', ...
        'ColorData', reshape(cmap(1:2,:,:), [], 4).');
end 
    

function [x] = sample_1d(a, b, N)
    x = a + (b-a) .* rand(N, 1);
end 
