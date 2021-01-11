function output_analysis()
    % This function stores data in the format 
    % {exp_type -> [[avg_lin_jerk, avg_ang_jerk, avg_safety score, avg_dist_to_opt_traj, time, num_safety_control, exp_result], ...]}
    % {exp_type -> [(exp_name_1), (exp_name_2), ...]}
    % prints the experiments that failed to run for debugging
    % plots each experiment and metric for each exp_run
    % prints out a high level summary of the metrics of each experiment
    
    %% Edit these variables for different experiment types (separated by folder name)
    path = './outputs/';
    exp_types = {
        %'probabilistic_blend_safety_control_traj_replan_dt_1.500_num_samples_10_level_set_0.20', ... 
        %'switch_replan_dt_1.500_zero_level_set_0.10'
        'switch_replan_dt_1.500_zero_level_set_0.200_spline_obs_weight_1.000000', ...
    };
    results_folder = './outputs/initial_results/';
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
                ni = size(names, 1); 
                names{ni} = exp_name; 
                all_exp_names{eti} = names;
                metrics = all_exp_metrics{eti}; 
                if isfile(f)
                    load(f); 
                    total_time = size(obj.blend_traj, 2) * obj.dt;
                    if contains(exp_name, 'switch') 
                        num_safety_control = sum(obj.blend_traj(6, :) == 0); 
                    else
                        num_safety_control = sum(obj.blend_traj(6, :) == -0.2); 
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
%                     debugging failed cases 
%                     if obj.termination_state == 1
%                         fprintf("Crashed exp: %s\n", exp_name);  
%                     end 
%                     if obj.termination_state == 2
%                         fprintf("Tle exp: %s\n", exp_name);  
%                     end 
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
        exp_type = exp_types{eti};
        exp_metrics = all_exp_metrics{eti};  
        alj = sum(exp_metrics(:, 1));
        slj = std(exp_metrics(:, 1)); 
        fprintf("Exp: %s avg lin jerk: %f std: %f\n", exp_type, alj, slj);
        aaj = sum(exp_metrics(:, 2));
        saj = std(exp_metrics(:, 2)); 
        fprintf("Exp: %s avg ang jerk: %f std: %f\n", exp_type, aaj, saj);
        assh = sum(exp_metrics(:, 3));
        sssh = std(exp_metrics(:, 3)); 
        fprintf("Exp: %s avg safety_score: %f std: %f\n", exp_type, assh, sssh); 
        adot = sum(exp_metrics(:, 4));
        sdot = std(exp_metrics(:, 4)); 
        fprintf("Exp: %s avg dist to opt traj: %f std: %f\n", exp_type, adot, sdot); 
        ati = sum(exp_metrics(:, 5));
        sti = std(exp_metrics(:, 5)); 
        fprintf("Exp: %s avg time taken: %f std: %f\n", exp_type, ati, sti); 
        ans = sum(exp_metrics(:, 6));
        sns = std(exp_metrics(:, 6)); 
        fprintf("Exp: %s avg num safety controls taken: % std: %f\n", exp_type, ans, sns); 
        termination = exp_metrics(:, 7);
        num_success = sum(termination == 0); 
        num_crashed = sum(termination == 1); 
        num_tle = sum(termination == 2); 
        num_error = sum(termination == -1); 
        total_exp = total_exp_num(i);
        fprintf("Exp: %s %d/%d reached goal\n", exp_type, num_success, total_exp);
        fprintf("Exp: %s %d/%d crashed\n", exp_type, num_crashed, total_exp);
        fprintf("Exp: %s %d/%d ran out of time\n", exp_type, num_tle, total_exp);
        fprintf("Exp: %s %d/%d errored\n", exp_type, num_error, total_exp);
    end 
    
    %% Save statistics 
    save(sprintf("%s/%s", results_folder, 'output_analysis.mat'));
end
