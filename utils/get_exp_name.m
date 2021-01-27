function exp_name = get_exp_name(params)
    point_nav_str = get_point_nav_str(params); 
    exp_name = sprintf("%s_%s_blend_%s_control_%s_%s", params.environment_type, ... 
               point_nav_str, params.blend_scheme, params.control_scheme, params.hyperparam_str); 
end 
