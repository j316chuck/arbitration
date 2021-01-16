function exp_types = get_all_experiment_types()
    %% Analyze experiment metrics
    repo = what('arbitration');
    path = strcat(repo.path,'/../outputs/');
    all_blend_schemes = get_all_blend_schemes();    
                    
    % ============ Control  Scheme ============== %
    all_control_schemes = {'follow', 'switch'};  
    exp_types = {}; 
    for i = 1:length(all_blend_schemes)
        for j = 1:length(all_control_schemes)
            params = default_hyperparams();
            bs = all_blend_schemes{i}; 
            cs = all_control_schemes{j}; 
            params.blend_scheme = all_blend_schemes{i}; 
            params.control_scheme = all_control_schemes{j}; 
            hs = get_hyperparam_string(params); 
            exp_types{end+1} = sprintf("blend_%s_control_%s_%s", bs, cs, hs);            
        end 
    end 
end 