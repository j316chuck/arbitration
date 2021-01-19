function [params] = get_replan_zls_hyperparams()
    zlsets = [0.05, 0.15]; 
    replan_dts = [0.5, 1, 5]; 
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