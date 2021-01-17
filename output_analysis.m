function output_analysis()
    verbose = false;
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs'); 
    results_folder = strcat(repo.path, '/outputs/results');
    output_path = sprintf("%s/%s", results_folder, 'output_analysis.mat');

    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end
    print_smoke_test_results();
     [metrics, lin_jerk_matrix, ang_jerk_matrix, safety_score_matrix, ...
    dist_to_opt_traj_matrix, timestamp_matrix, ...
    num_safety_control_matrix, termination_matrix, exp_name_matrix ...
    crashed_matrix, reached_matrix, tle_matrix, errored_matrix] = get_metrics(output_folder, verbose);
    load(output_path, 'metrics'); 
    print_exp_results(metrics); 
    save(output_path);
end

function [metrics, lin_jerk_matrix, ang_jerk_matrix, safety_score_matrix, ...
        dist_to_opt_traj_matrix, timestamp_matrix, ...
        num_safety_control_matrix, termination_matrix, exp_name_matrix ...
        crashed_matrix, reached_matrix, tle_matrix, errored_matrix] = ... 
        get_metrics(output_folder, verbose)
    [starts, goals] = get_all_smoke_test_cases();
    control_schemes = get_all_control_schemes();
    blend_schemes = get_all_blend_schemes(); 
    Ns = length(starts); % start and goal pairs
    Nc = length(control_schemes); % control scheme
    Nb = length(blend_schemes); % blend scheme
    Nm = 11; % num metrics
    
    % default value -1
    metrics = -ones(Ns, Nc, Nb, Nm); 
    lin_jerk_matrix = -ones(Ns, Nc, Nb);
    ang_jerk_matrix = -ones(Ns, Nc, Nb);
    safety_score_matrix = -ones(Ns, Nc, Nb);
    dist_to_opt_traj_matrix = -ones(Ns, Nc, Nb);
    timestamp_matrix = -ones(Ns, Nc, Nb);
    num_safety_control_matrix = -ones(Ns, Nc, Nb);
    termination_matrix = -ones(Ns, Nc, Nb);
    exp_name_matrix = cell(Ns, Nc, Nb); 
    reached_matrix = zeros(Ns, Nc, Nb); 
    crashed_matrix = zeros(Ns, Nc, Nb); 
    tle_matrix = zeros(Ns, Nc, Nb); 
    errored_matrix = zeros(Ns, Nc, Nb); 

    dirs = dir(fullfile(output_folder));
    for i = 1:Ns
        for j = 1:Nc
            for k = 1:Nb
                start = starts{i}; 
                goal = goals{i}; 
                st = sprintf("start_[%.2f %.2f %.2f]", start(1), start(2), start(3)); 
                go = sprintf("goal_[%.2f %.2f %.2f]", goal(1), goal(2), goal(3));
                cs = control_schemes{j};
                bs = blend_schemes{k}; 
                exp_name = sprintf("%s_%s_blend_%s_control_%s", st, go, bs, cs); 
                exp_ran = false; 
                for it = 1:length(dirs)
                    exp_folder = dirs(it, :); 
                    if ~contains(exp_folder.name, exp_name)
                       continue;
                    end 
                    f = fullfile(exp_folder.folder, exp_folder.name, 'final_state.mat');
                    if ~isfile(f)
                        continue;
                    end 
                    load(f, 'obj'); 
                    timestamp =  size(obj.blend_traj, 2) * obj.dt; 
                    num_safety = sum(obj.blend_traj(6, :) == 0); 
                    metrics(i, j, k, :) = [obj.scores.avg_lin_jerk, ...
                                        obj.scores.avg_ang_jerk, ...
                                        obj.scores.avg_safety_score, ...
                                        obj.scores.avg_dist_to_opt_traj, ...
                                        timestamp, ...
                                        num_safety, ...
                                        obj.termination_state, ...
                                        obj.termination_state == 0, ...
                                        obj.termination_state == 1, ...
                                        obj.termination_state == 2, ...
                                        obj.termination_state == -1];
                    lin_jerk_matrix(i, j, k) = obj.scores.avg_lin_jerk;
                    ang_jerk_matrix(i, j, k) = obj.scores.avg_ang_jerk;
                    safety_score_matrix(i, j, k) = obj.scores.avg_safety_score; 
                    dist_to_opt_traj_matrix(i, j, k) = obj.scores.avg_dist_to_opt_traj;
                    timestamp_matrix(i, j, k) = timestamp; 
                    num_safety_control_matrix(i, j, k) = num_safety; 
                    termination_matrix(i, j, k) = obj.termination_state;
                    reached_matrix(i, j, k) = obj.termination_state == 0; 
                    crashed_matrix(i, j, k) = obj.termination_state == 1; 
                    tle_matrix(i, j, k) = obj.termination_state == 2; 
                    errored_matrix(i, j, k) = obj.termination_state == -1; 
                    exp_name_matrix{i, j, k} = exp_name; 
                    if verbose && obj.termination_state == 1
                       fprintf("Crashed exp: %s\n", exp_name);  
                    end 
                    if verbose && obj.termination_state == 2
                        fprintf("Tle exp: %s\n", exp_name); 
                    end
                    exp_ran = true;
                end 
                if ~exp_ran
                    fprintf("Skipped %s_%s_%s_%s\n", st, go, cs, bs); 
                end
            end
        end
    end
end 

function print_exp_results(metrics)
    %% Print summary statistics
    blend_schemes = get_all_blend_schemes(); 
    control_schemes = get_all_control_schemes();
    for k = 1:length(blend_schemes)
        for j = 1:length(control_schemes)
            bs = blend_schemes{k}; 
            cs = control_schemes{j}; 
            exp_type = sprintf("blend_%s_control_%s", bs, cs); 
            alj = mean(metrics(:, j, k, 1));
            slj = std(metrics(:, j, k, 1)); 
            fprintf("Exp: %s avg lin jerk: %f std: %f\n", exp_type, alj, slj);
            aaj = mean(metrics(:, j, k, 2));
            saj = std(metrics(:, j, k, 2)); 
            fprintf("Exp: %s avg ang jerk: %f std: %f\n", exp_type, aaj, saj);
            assh = mean(metrics(:, j, k, 3));
            sssh = std(metrics(:, j, k, 3)); 
            fprintf("Exp: %s avg safety_score: %f std: %f\n", exp_type, assh, sssh); 
            adot = mean(metrics(:, j, k, 4));
            sdot = std(metrics(:, j, k, 4)); 
            fprintf("Exp: %s avg dist to opt traj: %f std: %f\n", exp_type, adot, sdot); 
            ati = mean(metrics(:, j, k, 5));
            sti = std(metrics(:, j, k, 5)); 
            fprintf("Exp: %s avg time taken: %f std: %f\n", exp_type, ati, sti); 
            asst = mean(metrics(:, j, k, 6));
            sns = std(metrics(:, j, k, 6)); 
            fprintf("Exp: %s avg num safety controls taken: %f std: %f\n", exp_type, asst, sns); 
            termination = metrics(:, j, k, 7);
            num_success = sum(termination == 0); 
            num_crashed = sum(termination == 1); 
            num_tle = sum(termination == 2); 
            num_error = sum(termination == -1); 
            total_exp = num_success + num_crashed + num_tle + num_error;
            fprintf("Exp: %s %d/%d reached goal\n", exp_type, num_success, total_exp);
            fprintf("Exp: %s %d/%d crashed\n", exp_type, num_crashed, total_exp);
            fprintf("Exp: %s %d/%d ran out of time\n", exp_type, num_tle, total_exp);
            fprintf("Exp: %s %d/%d errored\n", exp_type, num_error, total_exp);
        end 
    end 
end 

function print_smoke_test_results()
    %% Analyze smoke_test.mat
    repo = what("arbitration"); 
    path = strcat(repo.path, '/outputs/smoke_test.mat'); 
    load(path, 'failed_cases', 'exp_names', 'results', 'elapsed_time'); 
    for i = 1:length(failed_cases)
        fprintf("Failed %s\n", failed_cases{i}); 
    end 
    for i = 1:length(results)
        fprintf("Exp: %s, result: %d, time: %f\n", exp_names{i}, results(i), elapsed_time(i));  
    end 
end 

  