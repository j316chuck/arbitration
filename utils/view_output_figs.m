function view_output_figs()
    %close all;
    starts = {[3; -2; -pi; 0.01], [-3; 2; -pi; 0.01], [-8; -9.5; 0; 0.01], [1; 1; 0; 0.01]};
    goals = {[3; -4; -pi/2; 0.01], [-4; -1; -pi/2; 0.01], [-5; -8; 0; 0.01], [3, 2.75, pi/2, 0.01]};
    
    exp.start = starts{2}; %[3; -2; -pi];
    exp.goal = goals{2}; %[4; -1; -pi/2];
    exp.map_basename = 'bookstore'; 
    exp.blending_scheme = 'distance';
    exp.replan_dt = 1.5;
    exp.zero_level_set = 1;
    exp.alpha = 0.75;
    exp.temperature = 0.5;
    exp.blend_function = 'shift_sig';
    %exp.hyperparam_str = sprintf("replan_dt_%.3f_zero_level_set_%.3f", exp.replan_dt, exp.zero_level_set); 
    %exp.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", exp.replan_dt, exp.alpha); 
    exp.hyperparam_str = sprintf("replan_dt_%.3f_%s_temp_%.3f", exp.replan_dt, exp.blend_function, exp.temperature); 
    start_str = sprintf("start_[%.2f %.2f %.2f]", exp.start(1), exp.start(2), exp.start(3)); 
    goal_str = sprintf("goal_[%.2f %.2f %.2f]", exp.goal(1), exp.goal(2), exp.goal(3)); 
    exp_name = sprintf("%s_map_%s_%s_blending_scheme_%s_%s", exp.map_basename, start_str, goal_str, exp.blending_scheme, exp.hyperparam_str); 
    openfig(sprintf('../outputs/%s/planners.fig', exp_name));
    openfig(sprintf('../outputs/%s/metrics.fig', exp_name));
end 