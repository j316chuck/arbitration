function [hyperparam_str] = get_hyperparam_string(params)
    base_str = sprintf("zls_%.3f_replan_dt_%.3f_obs_weight_%.3f", params.zero_level_set, params.replan_dt, params.spline_obs_weight);  
    if strcmp(params.blend_scheme, 'no_replan')
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
        hyperparam_str = base_str; 
    elseif strcmp(params.blend_scheme, 'replan_waypoint')
        hyperparam_str = sprintf("spline_candidates_%.3f_replan_level_set_%.3f_%s", ...
            params.replan_level_set, params.replan_spline_max_num_candidates, base_str); 
    else 
        hyperparam_str = "";
        warning("blending scheme not supported");  
    end 
end 
