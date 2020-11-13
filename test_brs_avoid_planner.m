% Plane_test()
%   Tests the Plane class by computing a reachable set and then computing the
%   optimal trajectory from the reachable set.

%% Plane parameters
xstart = [100; 75; 220*pi/180];
wMax = 1.2;
vrange = [1.1, 1.3];
dMax = [0.3; 0.3];
pl = Plane(xstart, wMax, vrange, dMax);


%% Target and obstacles
g = createGrid([0; 0; 0], [150; 150; 2*pi], [41; 41; 11]);
xgoal = [75; 50; 0];
obs = shapeCylinder(g, 3, xgoal, 10);


%% Compute reachable set
tau = 0:0.5:10;

schemeData.dynSys = pl;
schemeData.grid = g;
schemeData.uMode = 'max';
schemeData.dMode = 'min';

planner = BRSAvoidPlanner(g, schemeData, tau);
            
%% Plan!
% note: need a non-zero starting velocity to avoid singularities in spline
planner.solve_brs_avoid(obs);
%% Plot
v = planner.get_value(xgoal);

u = planner.get_avoid_u(xgoal);


