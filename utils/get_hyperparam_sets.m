function params = get_hyperparam_sets(name)
    if strcmp(name, "replan_zls")
        params = get_hyperparams_replan_zls(); 
    elseif strcmp(name, "default")
        params = get_default_hyperparams();
    elseif strcmp(name, "time_vary_alpha_open")
        params = get_hyperparams_time_vary_alpha_open();
    elseif strcmp(name, "HJIPDE_update_methods")
        params = get_hyperparams_HJIPDE_speed_test(); 
    else 
        warning("No hyperparameter set of name %s", name);
        params = {}; 
    end 
end 

%% Default Hyperparameters
function params = get_default_hyperparams()
    params = {default_hyperparams()}; 
end 

%% Apply different HJIPDE solver
function params = get_hyperparams_HJIPDE_speed_test()
    %blend_function_names = get_all_alpha_blend_function_names(); 
    updateMethods = get_all_update_methods(); 
    Ne = length(updateMethods); 
    params = cell(Ne, 1);
    for i = 1:length(updateMethods)
        p = default_hyperparams(); 
        p.updateMethod = updateMethods{i};
        p.hyperparam_str = get_hyperparam_string(p); 
        params{i} = p; 
    end
end 

%% Grid search time vary alpha open blend function
function params = get_hyperparams_time_vary_alpha_open()
    %blend_function_names = get_all_alpha_blend_function_names(); 
    blend_function_names = get_new_alpha_blend_function_names(); 
    Ne = length(blend_function_names); 
    params = cell(Ne, 1);
    for i = 1:length(blend_function_names)
        p = default_hyperparams(); 
        p.blend_scheme = "time_vary_alpha_open_loop";
        p.blend_function_name = blend_function_names{i}; 
        p.blend_function = get_alpha_blend_function(p.blend_function_name);  
        p.hyperparam_str = get_hyperparam_string(p); 
        params{i} = p; 
    end
end 

%% Grid Search Replan Dt and Zls
function params = get_hyperparams_replan_zls()
    zlsets = [0.05, 0.15]; 
    replan_dts = [0.5, 1.5]; 
    Ne = length(zlsets) * length(replan_dts); 
    params = cell(Ne, 1);
    index = 1; 
    for i = 1:length(zlsets)
        for j = 1:length(replan_dts)
            p = default_hyperparams(); 
            p.zero_level_set = zlsets(i); 
            p.replan_dt = replan_dts(j); 
            p.hyperparam_str = get_hyperparam_string(p); 
            params{index} = p; 
            index = index + 1; 
        end
    end
end 
