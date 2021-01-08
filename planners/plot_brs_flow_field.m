% map_data = yaml.ReadYaml('../maps/bookstore.yaml');
% obs_data_2d = imread('../maps/bookstore.pgm'); 
% obs_data_2d = double(obs_data_2d) / 255;
% exp.map_data = map_data;
% exp.obs_data = obs_data_2d;
% 
% %% Resize the occupancy grid to fit the desired grid size
% % create grid representing the raw image 
% gmin_img = [map_data.origin{1}, map_data.origin{2}];
% gmax_img = gmin_img + map_data.resolution * size(obs_data_2d);
% gnum_img = size(obs_data_2d);
% grid_img = createGrid(gmin_img, gmax_img, gnum_img);
% exp.grid_img = grid_img;
% 
% % Define 3D grid
% gmin_3d = [-10; -10; -pi];
% gmax_3d = [5.4; 5.4; pi];
% gnum_3d = [41; 41; 16];
% pdDim = 3;                  % 3rd dimension is periodic
% grid_3d = createGrid(gmin_3d, gmax_3d, gnum_3d, pdDim);
% 
% %% Avoid BRS
% xstart = [0, 0, 0];
% wMax = exp.wMax; 
% vRange = exp.vRange; 
% dMax = exp.dMax; 
% avoidBrsDynSys = Plane(xstart, wMax, vRange, dMax); 
% exp.avoidBrsDynSys = avoidBrsDynSys; 
% avoidBrsSchemeData.dynSys = exp.avoidBrsDynSys;
% avoidBrsSchemeData.grid = grid_3d;
% avoidBrsSchemeData.uMode = 'max';
% avoidBrsSchemeData.dMode = 'min';
% exp.avoidBrsSchemeData = avoidBrsSchemeData; 
% long_tau = 0:0.5:30
% brs_planner = BRSAvoidPlanner(grid_3d, avoidBrsSchemeData, long_tau); 
load('./data/brs_planner.mat')
thetas = brs_planner.grid.vs{3};
n = length(thetas);
m = ceil(n / 4); 
figure(3);
clf;
set(gcf, 'Position',  [100, 100, 3000, 3000])
for t=1:n
    theta = thetas(t); 
    [g2d, data2d] = proj(brs_planner.grid, brs_planner.valueFun, [0 0 1], [theta]); 
    x = g2d.xs{1};
    y = g2d.xs{2};
    x1 = []; x2 = []; y1 = []; y2 = []; u1 = []; v1 = [];
    dt = 0.3;
    for i = 1:numel(x)
        state = [x(i), y(i), theta]; 
        brs_planner.dynSys.x = state; 
        u = brs_planner.get_avoid_u(state'); 
        brs_planner.dynSys.updateState(u, dt, state); 
        new_state = brs_planner.dynSys.x; 
        x1 = [x1, x(i)]; 
        y1 = [y1, y(i)];
        x2 = [x2, new_state(1)]; 
        y2 = [y2, new_state(2)]; 
        u1 = [u1, new_state(1) - x(i)]; 
        v1 = [v1, new_state(2) - y(i)];
    end
    subplot(m, 4, t);
    hold on;
    title(sprintf('Theta %f', theta));
    xlabel('x(m)');
    ylabel('y(m)');
    quiver(x1, y1, u1, v1, 0, 'DisplayName', 'optimal control');
    contour(x, y, data2d, 'DisplayName', 'value fun', 'color', '#CC1FCB');
    legend('Location', 'NorthWest');
    savefig('./data/brs_planner_avoid_u.fig')
end 

