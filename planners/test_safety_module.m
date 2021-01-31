% Plane_test()
%   Tests the Plane class by computing a reachable set and then computing the
%   optimal trajectory from the reachable set.

%% Plane parameters
xstart = [100; 75; 220*pi/180];
wMax = 1.2;
vrange = [0.5, 1.0];
dMax = [0.3; 0.3; 0];
pl = Plane(xstart, wMax, vrange, dMax);


%% Target and obstacles
g = createGrid([0; 0; 0], [150; 150; 2*pi], [41; 41; 11]);
xi = [75; 50; 0];
xe = [70; 45; 0]; 
obs1 = shapeCylinder(g, 3, xi, 10);
obs2 = shapeCylinder(g, 3, xe, 10); 

%% Compute reachable set
dt = 0.3;
tau = 0:dt:10;

schemeData.dynSys = pl;
schemeData.grid = g;
schemeData.uMode = 'max';
schemeData.dMode = 'min';
updateMethod = 'localQ'; % localQ 
updateEpsilon = 0.01; 
warmStart = false; 

safety = SafetyModule(g, pl, 'max', 'min', dt, updateEpsilon, ...
                      warmStart, updateMethod, tau);
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
safety.computeAvoidSet(obs1);
plot_safety_avoid_u(safety, obs1, 1); 
safety.computeAvoidSet(obs2); 
plot_safety_avoid_u(safety, obs2, 2); 


function plot_safety_avoid_u(safety, obs, figN)
    figure(figN); 
    clf
    theta = 0; 
    [g2d, data2d] = proj(safety.grid, safety.lastValueFun, [0 0 1], theta); 
    [g2d, obs2d] = proj(safety.grid, obs, [0 0 1], theta); 
    x = g2d.xs{1};
    y = g2d.xs{2};
    x1 = []; x2 = []; y1 = []; y2 = []; u1 = []; v1 = [];
    for i = 1:numel(x)
        state = [x(i), y(i), theta]; 
        u = safety.get_avoid_u(state'); 
        new_state = safety.use_avoid_control(state); 
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


