function [starts, goals] = generate_nav_goals(N, min_dist, regen_points, seed_num)
    %% Generate points
    rng(seed_num);
    if regen_points
        p = default_hyperparams();
        exp = load_exp(p); 
        generate_nav_points(exp);
    end 
    
    %% Sample Goals and Starts
    repo = what('arbitration');
    filename = strcat(repo.path, '/data/valid_sampled_points.mat');
    load(filename, 'sampled_points');
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
        sa = zeros(sum(valid_ind), 1);
        ev = ones(sum(valid_ind), 1) * 0.01;
        ea = zeros(sum(valid_ind), 1);
        nstarts = [sx(valid_ind)', sy(valid_ind)', st(valid_ind)', sv, sa];
        ngoals = [gx(valid_ind)', gy(valid_ind)', gt(valid_ind)', ev, ea];
        starts = [starts; nstarts];
        goals = [goals; ngoals];
        counts = size(starts, 1);
    end
    
    %% Plot and save sampled starts and goals
    figure(6);
    hold on 
    set(gcf,'Position',[10 10 1000 800])
    contour(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle');
    plot_traj(starts(:, 1), starts(:, 2), starts(:, 3), 'sampled starts'); 
    plot_traj(goals(:, 1), goals(:, 2), goals(:, 3), 'sampled goals'); 
    view(0, 90);
    set(gcf, 'color', 'white');
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    legend('Location', 'NorthWest');
    title('Sampled Goals');
    savefig(strcat(repo.path,'/data/valid_sampled_goals.fig'));
    save(strcat(repo.path,'/data/valid_sampled_goals.mat'), 'starts', 'goals');
end 

function [sampled_points] = generate_nav_points(exp)
    %% sample points
    N = 1000;
    obstacle_threshold = 0.99; % intuitively it's 0 but practically it's 0.99
    counts = 0;
    sampled_x = [];
    sampled_y = [];
    hold on;
    while counts < N
        x = sample_1d(exp.grid_3d.min(1), exp.grid_3d.max(1), N-counts);
        y = sample_1d(exp.grid_3d.min(2), exp.grid_3d.max(2), N-counts);
        pts = [x(:), y(:)];
        u = eval_u(exp.grid_2d, exp.binary_occ_map, pts);
        valid_ind = u > obstacle_threshold;
        invalid_ind = ~valid_ind;
        sampled_x = [sampled_x, x(valid_ind)'];
        sampled_y = [sampled_y, y(valid_ind)'];
        counts = size(sampled_x, 2);
        %scatter(x(valid_ind), y(valid_ind), 10, 'g');
        %scatter(x(invalid_ind), y(invalid_ind), 10, 'b');
        %scatter(sampled_x, sampled_y, 20, 'ro');
    end 
    sampled_t = sample_1d(exp.grid_3d.min(3), exp.grid_3d.max(3), N)';
    sampled_points = [sampled_x; sampled_y; sampled_t];
    
    %% plot and save sampled points
    figure(5); 
    hold on 
    set(gcf,'Position',[10 10 1000 800])
    contour(exp.grid_2d.xs{1}, exp.grid_2d.xs{2}, exp.binary_occ_map, [0 0], 'DisplayName', 'obstacle');
    plot_traj(sampled_x, sampled_y, sampled_t, 'sampled points');
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    legend('Location', 'NorthWest');
    title('Sampled Points'); 
    repo = what('arbitration');
    save(strcat(repo.path,'/data/valid_sampled_points.mat'), 'sampled_points');
    savefig(strcat(repo.path,'/data/valid_sampled_points.fig'));
end 

function d = l2_dist(x1, x2, y1, y2)
    d =  ((x1 - x2) .^ 2 + (y1 - y2) .^ 2) .^ 0.5;
end 

function plot_traj(xs, ys, ths, name)
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
    
function [x] = sample_1d(a, b, N)
    x = a + (b-a) .* rand(N, 1);
end 
