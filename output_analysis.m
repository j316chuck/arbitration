function output_analysis()
    dirs = dir(fullfile('./outputs'));
    n = length(dirs);
    experiments = {'blending_scheme_blend_safety_value_traj', 'blending_scheme_probabilistic_blend_safety_control', 'blending_scheme_probabilistic_blend_safety_value', 'blending_scheme_switch_replan'};
    scores = zeros(length(experiments), 4);
    for exp_index = 1:length(experiments)
        exp_name = experiments{exp_index};
        for j = 1:n
            s = dirs(j, :);
            if contains(s.name, exp_name)
                f = fullfile(s.folder, s.name, 'final_state.mat'); 
                if isfile(f)
                    load(f); 
                    %1-success 2-crashed 3-ran out of time 4-total
                    result = obj.termination_state + 1; 
                    total_index = 4;
                    scores(exp_index, result) =  scores(exp_index, result) + 1;
                    scores(exp_index, total_index) = scores(exp_index, total_index) + 1;
                end 
            end 
        end
    end
    for i = 1:length(scores)
        exp = experiments{i};
        fprintf("Exp: %s %d/%d reached goal\n", exp, scores(i, 1), scores(i, 4));
        fprintf("Exp: %s %d/%d crashed\n", exp, scores(i, 2), scores(i, 4));
        fprintf("Exp: %s %d/%d ran out of time\n", exp, scores(i, 3), scores(i, 4));
    end 
end