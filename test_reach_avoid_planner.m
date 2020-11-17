% Plane_test()
%   Tests the Plane class by computing a reachable set and then computing the
%   optimal trajectory from the reachable set.

%% Plane parameters
xstart = [100; 75; 220*pi/180];
wMax = 1.2;
vrange = [1.1, 1.3];
dMax = [0.5; 0.5];
pl = Plane(xstart, wMax, vrange, dMax);


%% Target and obstacles
g = createGrid([0; 0; 0], [150; 150; 2*pi], [41; 41; 11]);
xgoal = [75; 50; 0];
target = shapeCylinder(g, 3, xgoal, 2);
obs1 = shapeRectangleByCorners(g, [300; 250; -inf], [350; 300; inf]);
obs2 = shapeRectangleByCorners(g, [5; 5; -inf], [145; 145; inf]);
obs2 = -obs2;

% modify
obstacle = shapeCylinder(g, 3, [90; 65; 0], 10);

%% Compute reachable set
tau = 0:0.5:500;

schemeData.dynSys = pl;
schemeData.grid = g;
schemeData.uMode = 'min';
schemeData.dMode = 'max';

planner = ReachAvoidPlanner(g, schemeData, tau);
optTrajDt = 0.1;
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
planner.solve_reach_avoid(xstart, xgoal, target, obstacle, optTrajDt);

%% Plot
planner.plot_traj();



