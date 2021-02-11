# Starting Out
- Create folders `data` and `outputs` 
- Run `generate_nav_goals.m` to create some point nav start and end goals
- Run `test_blender.m` to run a simple arbitration blending example
- Run `runner.m` or `parallel_runner.m` to run key point nav tasks
- Run `output_analysis.m` to generate a table for the results
- Run `plot_trajectories.m` to view each algorithm run on a point nav task

# Adding a new control scheme
- Modify `utils/get_all_control_schemes.m`
- Modify `default_hyperparams.m`
- Modify `Planner.m:get_next_control`


# Adding a new blending scheme
- Modify `utils/get_all_blend_schemes.m`
- Modify `utils/get_key_blend_schemes.m`
- Modify `default_hyperparams.m`
- Modify `get_hyperparam_string.m`
- Modify `Planner.m:plan_mpc_controls`
- Modify `SplinePlanner.m` add replan function

# Adding a new map
- Run `generate_nav_goals.m`
- Edit `get_point_nav_tasks.m`
- Edit `default_hyperparams.m`
- Run `parallel_runner.m`
