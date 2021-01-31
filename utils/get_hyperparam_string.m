function [hyperparam_str] = get_hyperparam_string(params)
    base_str = sprintf("grid_%s_zls_%.3f_replan_dt_%.3f_obs_weight_%.3f_sensor_%s_radius_%.2f", ... 
        params.grid_size, params.zero_level_set, params.replan_dt, ...
        params.spline_obs_weight, params.sensorArgs.sensor_shape, params.sensorArgs.sensor_radius); 
    if strcmp(params.blend_scheme, 'none')
        hyperparam_str = base_str; 
    elseif strcmp(params.blend_scheme, 'safety_value') 
        hyperparam_str = sprintf("alpha_%.3f_%s", params.alpha, base_str);  
    elseif strcmp(params.blend_scheme, 'safety_control')
        hyperparam_str = sprintf("alpha_%.3f_%s", params.alpha, base_str);  
    elseif strcmp(params.blend_scheme, 'sample_safety_value')
        hyperparam_str = sprintf("num_samples_%.3f_%s", params.num_alpha_samples, base_str);  
    elseif strcmp(params.blend_scheme, 'sample_safety_control')
        hyperparam_str = sprintf("num_samples_%.3f_%s", params.num_alpha_samples, base_str);  
    elseif strcmp(params.blend_scheme, 'time_vary_alpha_closed_loop_safety_control')
        hyperparam_str = sprintf("blend_function_%s_%s", params.blend_function_name, base_str); 
    elseif strcmp(params.blend_scheme, 'time_vary_alpha_open_loop_safety_control')
        hyperparam_str = sprintf("blend_function_%s_%s", params.blend_function_name, base_str); 
    elseif strcmp(params.blend_scheme, 'replan_waypoint')
        hyperparam_str = sprintf("spline_candidates_%.3f_replan_level_set_%.3f_%s", ...
            params.replan_spline_max_num_candidates, params.replan_level_set, base_str); 
    elseif strcmp(params.blend_scheme, 'replan_safe_traj')
        hyperparam_str = sprintf("num_mpc_safety_look_ahead_%.2f_%s", ... 
            params.num_mpc_safety_look_ahead, base_str); 
    else 
        hyperparam_str = base_str; 
        warning("blending scheme not supported");  
    end 
end 
