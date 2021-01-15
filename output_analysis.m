function output_analysis()
    % This function stores data in the format 
    % {exp_type -> [[avg_lin_jerk, avg_ang_jerk, avg_safety score, avg_dist_to_opt_traj, time, num_safety_control, exp_result], ...]}
    % {exp_type -> [(exp_name_1), (exp_name_2), ...]}
    % prints the experiments that failed to run for debugging
    % plots each experiment and metric for each exp_run
    % prints out a high level summary of the metrics of each experiment
    smoke_test_analysis();

    %% Option 2
    results_folder = strcat(repo.path, '/outputs/results');
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end 

    %% Get All Experiment Results Data
    Nt = length(exp_types); 
    all_exp_metrics = {}; % maps exp_types -> list of metrics
    all_exp_names = {};   % maps exp_types -> list of exp names
    for i = 1:Nt
        all_exp_metrics{i} = zeros(0, 7); 
        all_exp_names{i} = {}; 
    end  
    dirs = dir(fullfile(path));
    N = length(dirs);
    for eti = 1:Nt % experiment_type_index
        exp_type = exp_types{eti};
        for j = 1:N
            s = dirs(j, :);
            if contains(s.name, exp_type)
                exp_name = fullfile(s.folder, s.name); 
                f = fullfile(exp_name, 'final_state.mat'); 
                names = all_exp_names{eti};
                names{end+1} = exp_name; 
                all_exp_names{eti} = names;
                metrics = all_exp_metrics{eti}; 
                if isfile(f)
                    load(f); 
                    total_time = size(obj.blend_traj, 2) * obj.dt;
                    if contains(exp_name, 'switch') 
                        num_safety_control = sum(obj.blend_traj(6, :) == 0); 
                    end
                    % 0-success 1-crashed 2-ran out of time
                    metric = [  obj.scores.avg_lin_jerk; ...
                                obj.scores.avg_ang_jerk; .... 
                                obj.scores.avg_safety_score; ...
                                obj.scores.avg_dist_to_opt_traj; ...
                                total_time; ... 
                                num_safety_control; ...
                                obj.termination_state
                              ];
                    metrics(end+1, :) = metric; 
                    % debugging failed cases 
                    if obj.termination_state == 1
                        fprintf("Crashed exp: %s\n", exp_name);  
                    end 
                    if obj.termination_state == 2
                        fprintf("Tle exp: %s\n", exp_name);  
                    end 
                else 
                    metrics(end+1, :) = -ones(7, 1)' * 1; 
                    fprintf("Failed exp: %s\n", exp_name); 
                end 
                all_exp_metrics{eti} = metrics;
            end 
        end
    end
    
    %% Check if all experiment types have same number of experiments run
    total_exp_num = zeros(Nt); % maps exp type -> total exp num
    for  i=1:Nt 
         exp_metrics = all_exp_metrics{eti};
         total_exp_num(i) = size(exp_metrics, 1); 
    end 
    if ~all(total_exp_num == total_exp_num(1))
        warning("Unequal amount of experiments run for each experiment type");
    end 
    
    %% Plot experiment scores
    % TODO 
    
    
    %% Print summary statistics
    for i = 1:Nt
        exp_type = exp_types{i};
        exp_metrics = all_exp_metrics{i};  
        alj = mean(exp_metrics(:, 1));
        slj = std(exp_metrics(:, 1)); 
        fprintf("Exp: %s avg lin jerk: %f std: %f\n", exp_type, alj, slj);
        aaj = mean(exp_metrics(:, 2));
        saj = std(exp_metrics(:, 2)); 
        fprintf("Exp: %s avg ang jerk: %f std: %f\n", exp_type, aaj, saj);
        assh = mean(exp_metrics(:, 3));
        sssh = std(exp_metrics(:, 3)); 
        fprintf("Exp: %s avg safety_score: %f std: %f\n", exp_type, assh, sssh); 
        adot = mean(exp_metrics(:, 4));
        sdot = std(exp_metrics(:, 4)); 
        fprintf("Exp: %s avg dist to opt traj: %f std: %f\n", exp_type, adot, sdot); 
        ati = mean(exp_metrics(:, 5));
        sti = std(exp_metrics(:, 5)); 
        fprintf("Exp: %s avg time taken: %f std: %f\n", exp_type, ati, sti); 
        asst = mean(exp_metrics(:, 6));
        sns = std(exp_metrics(:, 6)); 
        fprintf("Exp: %s avg num safety controls taken: %f std: %f\n", exp_type, asst, sns); 
        termination = exp_metrics(:, 7);
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
    
    %% Save statistics 
    save(sprintf("%s/%s", results_folder, 'output_analysis.mat'));
end

function smoke_test_analysis()
    %% Analyze smoke_test.mat
    repo = what("~arbitration"); 
    path = strcat(repo.path, 'smoke_test.mat'); 
    load(path); 
    for i = 1:length(failed_cases)
        fprintf("Failed %s\n", failed_cases{i}); 
    end 
    for i = 1:length(results)
        fprintf("Exp: %s, result: %d, time: %f\n", exp_names{i}, results(i), elapsed_time(i));  
    end 
    
    %% Analyze experiment metrics
    repo = what('arbitration');
    path = strcat(repo.path,'/outputs/');
    all_blend_schemes = {'time_vary_alpha_open_loop_safety_control', ...
                        'time_vary_alpha_closed_loop_safety_control', ...
                        'safety_value', ...
                        'safety_control', ...
                        'sample_safety_value', ...
                        'sample_safety_control', ...
                        'replan_waypoint', ....
                        'replan_safe_traj', ...
                        'none'};    
                    
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
            hs = get_hyperparam_string(params)
            exp_types{end+1} = sprintf("blend_%s_control_%s_%s", bs, cs, hs);            
        end 
    end 
    
    results_folder = strcat(repo.path, '/outputs/results');
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end 

    %% Get All Experiment Results Data
    Nt = length(exp_types); 
    all_exp_metrics = {}; % maps exp_types -> list of metrics
    all_exp_names = {};   % maps exp_types -> list of exp names
    for i = 1:Nt
        all_exp_metrics{i} = zeros(0, 7); 
        all_exp_names{i} = {}; 
    end  
    dirs = dir(fullfile(path));
    N = length(dirs);
    for eti = 1:Nt % experiment_type_index
        exp_type = exp_types{eti};
        for j = 1:N
            s = dirs(j, :);
            if contains(s.name, exp_type)
                exp_name = fullfile(s.folder, s.name); 
                f = fullfile(exp_name, 'final_state.mat'); 
                names = all_exp_names{eti};
                names{end+1} = exp_name; 
                all_exp_names{eti} = names;
                metrics = all_exp_metrics{eti}; 
                if isfile(f)
                    load(f); 
                    total_time = size(obj.blend_traj, 2) * obj.dt;
                    num_safety_control = sum(obj.blend_traj(6, :) == 0); 
                     
                    
                    % 0-success 1-crashed 2-ran out of time
                    metric = [  obj.scores.avg_lin_jerk; ...
                                obj.scores.avg_ang_jerk; .... 
                                obj.scores.avg_safety_score; ...
                                obj.scores.avg_dist_to_opt_traj; ...
                                total_time; ... 
                                num_safety_control; ...
                                obj.termination_state
                              ];
                    metrics(end+1, :) = metric; 
                    % debugging failed cases 
                    if obj.termination_state == 1
                        fprintf("Crashed exp: %s\n", exp_name);  
                    end 
                    if obj.termination_state == 2
                        fprintf("Tle exp: %s\n", exp_name);  
                    end 
                else 
                    metrics(end+1, :) = -ones(7, 1)' * 1; 
                    fprintf("Failed exp: %s\n", exp_name); 
                end 
                all_exp_metrics{eti} = metrics;
            end 
        end
    end
    
    %% Check if all experiment types have same number of experiments run
    total_exp_num = zeros(Nt); % maps exp type -> total exp num
    for  i=1:Nt 
         exp_metrics = all_exp_metrics{eti};
         total_exp_num(i) = size(exp_metrics, 1); 
    end 
    if ~all(total_exp_num == total_exp_num(1))
        warning("Unequal amount of experiments run for each experiment type");
    end 
    
    %% Plot experiment scores
    % TODO 
    
    
    %% Print summary statistics
    for i = 1:Nt
        exp_type = exp_types{i};
        exp_metrics = all_exp_metrics{i};  
        alj = mean(exp_metrics(:, 1));
        slj = std(exp_metrics(:, 1)); 
        fprintf("Exp: %s avg lin jerk: %f std: %f\n", exp_type, alj, slj);
        aaj = mean(exp_metrics(:, 2));
        saj = std(exp_metrics(:, 2)); 
        fprintf("Exp: %s avg ang jerk: %f std: %f\n", exp_type, aaj, saj);
        assh = mean(exp_metrics(:, 3));
        sssh = std(exp_metrics(:, 3)); 
        fprintf("Exp: %s avg safety_score: %f std: %f\n", exp_type, assh, sssh); 
        adot = mean(exp_metrics(:, 4));
        sdot = std(exp_metrics(:, 4)); 
        fprintf("Exp: %s avg dist to opt traj: %f std: %f\n", exp_type, adot, sdot); 
        ati = mean(exp_metrics(:, 5));
        sti = std(exp_metrics(:, 5)); 
        fprintf("Exp: %s avg time taken: %f std: %f\n", exp_type, ati, sti); 
        asst = mean(exp_metrics(:, 6));
        sns = std(exp_metrics(:, 6)); 
        fprintf("Exp: %s avg num safety controls taken: %f std: %f\n", exp_type, asst, sns); 
        termination = exp_metrics(:, 7);
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
    
    %% Save statistics 
    save(sprintf("%s/%s", results_folder, 'output_analysis.mat'));

    
end 
