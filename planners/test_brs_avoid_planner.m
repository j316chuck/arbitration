% Plane_test()
%   Tests the Plane class by computing a reachable set and then computing the
%   optimal trajectory from the reachable set.

%% Plane parameters
xstart = [100; 75; 220*pi/180];
wMax = 1;
vrange = [0.5, 1.0];
dMax = [0.3; 0.3; 0.1];
pl = Plane(xstart, wMax, vrange, dMax);


%% Target and obstacles
g = createGrid([0; 0; 0], [150; 150; 2*pi], [41; 41; 11]);
xi = [75; 50; 0];
obs1 = shapeCylinder(g, 3, xi, 10);
xe = [70; 45; 0];
obs2 = shapeCylinder(g, 3, xe, 10);

%% Compute reachable set
dt = 0.1;
tau = 0:dt:10;

updateMethod = 'local_q';
schemeData.dynSys = pl;
schemeData.grid = g;
schemeData.uMode = 'max';
schemeData.dMode = 'min';
schemeData.accuracy = 'high'; 
if strcmp(updateMethod, 'local_q')
    schemeData.hamFunc = @dubins3Dham_localQ;
    schemeData.partialFunc = @dubins3Dpartial_localQ;
end 

planner = BRSAvoidPlanner(g, schemeData, tau, 0.5, updateMethod);
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
planner.solve_brs_avoid(obs1);
plot_brs_avoid_u(planner, obs1, 10);
planner.solve_brs_avoid(obs2);
plot_brs_avoid_u(planner, obs2, 11); 

function plot_brs_avoid_u(brs, obs, figN)
    figure(figN); 
    clf
    theta = 0; 
    [g2d, data2d] = proj(brs.grid, brs.valueFun, [0 0 1], theta); 
    [g2d, obs2d] = proj(brs.grid, obs, [0 0 1], theta); 
    x = g2d.xs{1};
    y = g2d.xs{2};
    x1 = []; x2 = []; y1 = []; y2 = []; u1 = []; v1 = [];
    for i = 1:numel(x)
        state = [x(i), y(i), theta]; 
        u = brs.get_avoid_u(state'); 
        new_state = brs.use_avoid_control(state); 
        x1 = [x1, x(i)]; 
        y1 = [y1, y(i)];
        x2 = [x2, new_state(1)]; 
        y2 = [y2, new_state(2)]; 
        u1 = [u1, new_state(1) - x(i)]; 
        v1 = [v1, new_state(2) - y(i)];
    end
    hold on;
    title(sprintf('Theta %f', theta));
    xlabel('x(m)');
    ylabel('y(m)');
    quiver(x1, y1, u1, v1, 0, 'DisplayName', 'optimal control');
    contour(x, y, data2d, 'DisplayName', 'value fun');
    contour(x, y, obs2d, [0 0], 'DisplayName', 'obstacle shape', 'color', 'red'); 
    colorbar;
    legend('Location', 'NorthWest', 'Interpreter', 'None');
    hold off;
end 

