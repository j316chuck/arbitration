function runner()
    close all;
    params = default_hyperparams();
    sigmoids = {'reg_sig'; 'sub'; 'shift_sig'};
    for temperature = [0.5, 1]
        for replan_dt = [1.5, 5]
            for i = 1:length(sigmoids)
                params.temperature = temperature;
                params.replan_dt = replan_dt;
                params.blend_function = sigmoids{i};
                params.hyperparam_str = sprintf("replan_dt_%.3f_%s_temp_%.3f", params.replan_dt, params.blend_function, params.temperature); 
                exp = load_exp(params);
                pb = PlannerBlender(exp); 
                pb.blend_planners(); 
            end 
        end 
    end 
end 