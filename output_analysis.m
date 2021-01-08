function output_analysis()
    dirs = dir(fullfile('./old_outputs/1_3/'));
    n = length(dirs);
    experiments = {'blending_scheme_switch', 'blending_scheme_probabilistic_blend_safety_control'};
    scores = zeros(length(experiments), 4);
    results = zeros(length(experiments), 2);
    crashed_exps = {};
    tle_exps  = {};
    for exp_index = 1:length(experiments)
        exp_name = experiments{exp_index};
        for j = 1:n
            s = dirs(j, :);
            if contains(s.name, exp_name) && contains(s.name, 'level_set_0.2')
                f = fullfile(s.folder, s.name, 'final_state.mat'); 
                if isfile(f)
                    load(f); 
                    %1-success 2-crashed 3-ran out of time 4-total
                    result = obj.termination_state + 1; 
                    total_index = 4;
                    scores(exp_index, result) =  scores(exp_index, result) + 1;
                    scores(exp_index, total_index) = scores(exp_index, total_index) + 1;
                    results(exp_index, 1) = results(exp_index, 1) + obj.scores.avg_jerk; 
                    results(exp_index, 2) = results(exp_index, 2) + obj.scores.avg_safety_score; 
                    if obj.termination_state == 1
                        crashed_exps{length(crashed_exps) + 1} = f;
                        fprintf("Crashed exp: %s\n", f);  
                    end 
                    if obj.termination_state == 2
                        tle_exps{length(tle_exps) + 1} = f;
                        %fprintf("Tle exp: %s\n", f);  
                    end 
                end 
            end 
        end
    end
    for i = 1:length(scores)
        exp = experiments{i};
        fprintf("Exp: %s %d/%d reached goal\n", exp, scores(i, 1), scores(i, 4));
        fprintf("Exp: %s %d/%d crashed\n", exp, scores(i, 2), scores(i, 4));
        fprintf("Exp: %s %d/%d ran out of time\n", exp, scores(i, 3), scores(i, 4));
        fprintf("Exp: %s avg jerk: %f \n", exp, results(i, 1) / 14);
        fprintf("Exp: %s avg safety score: %f\n", exp, scores(i, 2) / 14);
    end 
    save('results.mat')
end