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
