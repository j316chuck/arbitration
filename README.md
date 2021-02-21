# Arbitration
Repo for testing different arbitration policies with a 3D Dubins car dynamical system. We experimented with blending a spline planner and a safety planner. The safety planner was obtained from solving the backwards reachable set of the [HJIPDE](https://people.eecs.berkeley.edu/~somil/Papers/Introduction_to_Reachability_to_Share.pdf) equations. 

# Run Experiments 

## Installation Dependencies
- [HelperOC](https://github.com/HJReachability/helperOC)
- [ToolBoxLs](https://www.cs.ubc.ca/~mitchell/ToolboxLS/) 
- [Parallel Computing Toolbox](https://www.mathworks.com/products/parallel-computing.html)
- [Binary Occupancy Maps](https://github.com/aws-robotics/aws-robomaker-bookstore-world)
- [YamlMatlab](https://github.com/ewiger/yamlmatlab) 

## Run pipeline
- Create folders `data` and `outputs` in the main repo directory
- Run `generate_nav_goals.m` to create point navigation start and end goals for robot
- Run `test_planner.m` to run a simple arbitration blending example
- Run `parallel_runner.m` to run all point nav tasks for 100 start and end examples
- Run `output_analysis.m` to generate an analysis of the results
- Run `plot_exp_all.m` to view each algorithm run on a point nav task

## Add new control scheme
- Modify `utils/get_all_control_schemes.m`
- Modify `default_hyperparams.m`
- Modify `Planner.m:get_next_control`

## Add new blending scheme
- Modify `utils/get_all_blend_schemes.m`
- Modify `utils/get_key_blend_schemes.m`
- Modify `default_hyperparams.m`
- Modify `get_hyperparam_string.m`
- Modify `Planner.m:plan_mpc_controls`
- Modify `SplinePlanner.m` add replan function

## Add new map
- Add `map_name.pgm` and `map_name.yaml` to the `maps` directory
- Run `generate_nav_goals.m`
- Modify `get_point_nav_tasks.m`
- Modify `default_hyperparams.m`

# Results 

## Problem Statement
Our task is the point navigation problem in robotics where a robot is given a start position, an end goal, and a set of obstacles in a binary occupancy grid. The robot must plan a goal reaching trajectory from the start location to the end goal which does not collide with any obstacles. We tested the robot in two types of environments: known and unknown. <br> <br>
In the known environment, the robot knows all the obstacles a-priori. In the unknown environment, the robot knows all obstacles within a 2 meter lidar sensing region. We let x denote the 3D position (x, y, theta) of the robot at a specific point in time. We choose the model predictive control framework for our planning algorithms. At each planning iteration we evaluate:  
- V(x) the safety value function for the HJIPDE equations
- x_safety the safety trajectory for the next N horizon time stamps
- x_spline the spline trajectory for the next N horizon time stamps 
- cost(x) the spline planner cost function used to find the optimal spline plan

Our algorithms investigate different arbitration policies that aim to blend these two planner trajectories safely and efficiently. Safety is defined by the robot not crashing into any obstacles. Efficiency is defined by the robot maintaining goal reaching and low jerk controls. 

## Policies
Name | Formulation | Algorithm Description
------------ | ------------- | ----------
[CDC Baseline](https://arxiv.org/abs/1905.00532) | ![cdc formulation](/docs/cdc.png) | Robot goes wherever the spline planner desires until it enters an unsafe state in which the robot switches to a safety controller. 
[Mo and Karen Baseline](http://asl.stanford.edu/wp-content/papercite-data/pdf/Leung.Schmerling.Chen.ea.ISER18.pdf) | ![mo and karen formulation](/docs/mo_karen.png) | Find the best spline trajectory that does not enter an unsafe state within your spline mpc optimization procedure. (Constrained Optimization)
Constant Alpha | ![cpnstant alpha formulation](/docs/constant_alpha.png)  | Blend the two trajectories with a constant alpha (Unconstrained Optimization)
Sampled Alpha | ![sampled alpha formulation](/docs/sample_alpha.png) |Sample alphas from 1 - 0 in a decreasing order until we find an alpha blended trajectory that doesn't go into an unsafe state. 
Time Vary Alpha | ![time vary alpha formulation](/docs/time_vary_alpha.png) | Blend the two trajectories with a time varying alpha that depends on the safety value V(x). 
Reach Avoid | None | Optimal goal reaching trajectory obtained from solving the reach avoid formulation of the HJIPDE equations


## Qualitative Robot Trajectory Plots
![Robot Trajectories Taken By Blending Policies](/docs/example_traj.png)
Map | Environment Type | Plot Link
------------ | ------------- | ----------
[bookstore](/maps/bookstore.png) | known | [Plots](https://drive.google.com/drive/folders/1h-6rxeIYv7fhTN_vJiw0UwcAhmgnrwik?usp=sharing) 
[bookstore](/maps/bookstore.png) | unknown | [Plots](https://drive.google.com/drive/folders/1lL7ceeNvR0gybtQY76HfbGfKflXovyyx?usp=sharing) 
[simple_grid](/maps/simple_env.png) | known | [Plots](https://drive.google.com/drive/folders/1W06TvIMFzIhHwnZ5SYD9mvYPBSXCeV5Y?usp=sharing)
[simple_grid](/maps/simple_env.png) | unknown | [Plots](https://drive.google.com/drive/folders/1lAQ3Ti6wiXc--9-5pKO0O9IvVZnMZCNk?usp=sharing) 

## Quantitative Algorithm Comparisons
Map | Environment Type | Results
------------ | ------------- | ----------
[bookstore](/maps/bookstore.png) | known | [Metrics](https://docs.google.com/spreadsheets/d/1iGt3TIhQT2URcAo91xGabPvCbsk9aFvviDL7tkEz02Q/edit#gid=472574939) 
[bookstore](/maps/bookstore.png) | unknown | [Metrics](https://docs.google.com/spreadsheets/d/1iGt3TIhQT2URcAo91xGabPvCbsk9aFvviDL7tkEz02Q/edit#gid=1791201822) 
[simple_grid](/maps/simple_env.png) | known | [Metrics](https://docs.google.com/spreadsheets/d/1iGt3TIhQT2URcAo91xGabPvCbsk9aFvviDL7tkEz02Q/edit?usp=sharing)

## Main Results
- In a completely known environment, the constrained optimization formulation (e.g. Mo and Karen's paper) performs best empirically. Intuitively this makes sense, as if one knows the obstacles then we should plan around them by treating them as constraints in our optimization procedure. 
- In an unknown environment, the sampled alpha formulation performs best empirically. 

