function opener(num)
    exp.start = [-9; -9.5; 0];
    exp.goal = [3; 2.75; pi/2];
    exp.map_basename = 'bookstore'; 
    params.blending_scheme = 'probabilistic_blend_value_control_traj';
    params.replan_dt = 0.5;
    params.zero_level_set = 0.0;
    params.alpha = 0.75;
    params.temperature = 1;
    params.blend_function = 'reg_sig';
    %params.hyperparam_str = sprintf("replan_dt_%.3f_zero_levelset_%.3f", params.replan_dt, params.zero_level_set); 
    %params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
    params.hyperparam_str = sprintf("replan_dt_%.3f_%s_temp_%.3f", params.replan_dt, params.blend_function, params.temperature); 
    params.num_alpha_samples = 10;
    params.hyperparam_str = sprintf("replan_dt_%.3f_num_samples_%d_level_set_%.2f", params.replan_dt, params.num_alpha_samples, params.zero_level_set);
    exp.hyperparam_str = params.hyperparam_str; 
    start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
    goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
    obj.exp_name = sprintf("%s_map_%s_%s_blending_scheme_%s_%s", exp.map_basename, start_str, goal_str, params.blending_scheme, exp.hyperparam_str); 
    fig1name = sprintf("%s/planners_%d.fig", obj.exp_name, num); 
    open(fig1name); 
    fig2name = sprintf("%s/replan_traj_timestamp_%d.fig", obj.exp_name, num); 
    open(fig2name); 
end 