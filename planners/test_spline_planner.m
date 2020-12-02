%% Load up all the info.
params = spline_params();
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
start = [5, -2.25, pi/2, 0.01]; 
params.planner.set_sd_goal(params.goal, params.sd_goal);
params.planner.set_sd_obs(params.sd_obs);
opt_spline = params.planner.plan(start);
            
%% Plot!
xs = opt_spline{1};
ys = opt_spline{2};
ths = opt_spline{3};
u1s = opt_spline{4};
u2s = opt_spline{5};

%% Plot velocity profiles
figure(1)
clf
hold on
plot(0:params.dt:params.horizon, u1s, '-k', 'linewidth', 2);
plot([0,params.horizon], [params.max_linear_vel, params.max_linear_vel], '--b');
plot([0,params.horizon], [-params.max_linear_vel, -params.max_linear_vel], '--b');
title('linear velocity over time');
xlabel('t (sec)');
ylim([-params.max_linear_vel, params.max_linear_vel])
% 
figure(2)
clf
hold on
plot(0:params.dt:params.horizon, u2s, '-k', 'linewidth', 2);
plot([0,params.horizon], [params.max_angular_vel, params.max_angular_vel], '--b');
plot([0,params.horizon], [-params.max_angular_vel, -params.max_angular_vel], '--b');
title('angular velocity over time');
xlabel('t (sec)');
ylim([-params.max_angular_vel, params.max_angular_vel])

%% Plot the trajectory and environment. 
figure(3)
clf
hold on

% Plot the lane boundaries.
plot([params.gmin(1), -4.5], [0,0], '--y', 'linewidth', 4);
plot([4.5, params.gmax(1)], [0,0], '--y', 'linewidth', 4);
plot([0,0], [params.gmin(2), -4.5], '--y', 'linewidth', 4);
plot([0,0], [4.5, params.gmax(2)], '--y', 'linewidth', 4);

% Plot the EGO CAR's trajectory. 
robot_colors = [linspace(0.1, 0.9, length(xs))', zeros([length(xs),1]), zeros([length(xs),1])];
plot_traj(xs, ys, ths, robot_colors);

% Plot obstacles.
for oi = 1:length(params.obstacles)
    obs_info = params.obstacles{oi};
    obs_min = obs_info(1:2);

    x_min = obs_min(1);
    y_min = obs_min(2);
    p_min = 0;
    l = [obs_info(3), ...
        obs_info(4), ...
        0];
    plotcube(l + [params.obs_padding*2, 0], [x_min y_min p_min] - [params.obs_padding, 0], 0.1, [0.8,0.8,0.8]);
    plotcube(l,[x_min y_min p_min], .5, [0.3 0.3 0.3]);
end
contour(params.g2d.xs{1}, params.g2d.xs{2}, params.other_lane_sd, [0,0], ...
    'Color', [0.6,0.6,0.6], 'LineStyle', ':');

% Plot goal + goal region
scatter(params.goal(1), params.goal(2), 'r', 'filled');
quiver(params.goal(1), params.goal(2), cos(params.goal(3)), sin(params.goal(3)), 'r');
pos = [params.goal(1)-params.goal_radius, params.goal(2)-params.goal_radius, ...
       params.goal_radius*2, params.goal_radius*2];
rectangle('position', pos, ...
            'curvature', [1,1], ...
            'EdgeColor','r');
xlim([params.gmin(1),params.gmax(1)]);
ylim([params.gmin(2),params.gmax(2)]);
set(gcf, 'color', 'white')
view(0,90)
set(gcf, 'position', [0,0,800,800])
% grid on

%% Plots the trajectory
function plot_traj(xs, ys, ths, colors)
%     for i=1:length(xs)
%         %plot_car(xs(i), ys(i), ths(i), car_len, car_width, colors(i));
%         pos = [xs(i)-car_rad, ys(i)-car_rad, ...
%                car_rad*2, car_rad*2];
%         rectangle('position', pos, ...
%                     'curvature', [1,1], ...
%                     'EdgeColor', colors(i, :));
%     end
    s = scatter(xs, ys, 'r', 'filled');
    s.CData = colors; %[colors', zeros(length(xs), 1), zeros(length(xs), 1)];
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', colors(end,:));
    q.ShowArrowHead = 'off';
    q.AutoScale = 'off';
    q.AutoScaleFactor = 0.5;
end

%% Plots a car as a rotated rectangle. 
function plot_car(xc, yc, th, car_len, car_width, color)

rot_mat = [cos(th) -sin(th); ... 
           sin(th) cos(th)];

xoff = 0.5*car_len;
yoff = 0.5*car_width;

ur = [xc; yc] + rot_mat * [xoff; yoff];
ul = [xc; yc] + rot_mat * [- xoff; yoff]; 
dl = [xc; yc] + rot_mat * [- xoff; - yoff]; 
dr = [xc; yc] + rot_mat * [xoff; - yoff]; 

xs = [ur(1) ul(1) dl(1) dr(1)];
ys = [ur(2) ul(2) dl(2) dr(2)];

p = patch(xs,ys, color);
p.FaceColor = color;
p.FaceAlpha = 0.1;
p.EdgeColor = color;

end
