function output_analysis()  
    %% Change these parameters
    verbose = true;
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs/');
    results_folder = strcat(output_folder, '/results');
    output_mat_path = sprintf("%s/%s", results_folder, 'output_analysis.mat');
    output_csv_path = sprintf("%s/%s", results_folder, 'output_analysis.csv'); 
    foutput_csv_path = sprintf("%s/%s", results_folder, 'filtered_output_analysis.csv'); 
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end
    control_schemes = {'switch'};
    blend_schemes = {'none', 'replan_safe_traj', 'time_vary_alpha_open_loop_safety_control', 'sample_safety_control'};
    [starts, goals] = get_point_nav_tasks("sampled"); %"sampled" %"smoke_test"
    params = get_hyperparam_sets("default"); %"replan_zls"

    %% Get metrics
%      [metrics, exp_names] = get_metrics(output_folder, ... 
%          starts, goals, control_schemes, blend_schemes, params, verbose);
%     save(output_mat_path, 'metrics', 'exp_names'); 
    
    %% Write table
    load(output_mat_path, 'metrics'); 
    write_metrics_table(metrics, control_schemes, blend_schemes, params, output_csv_path, false, verbose); 
    write_metrics_table(metrics, control_schemes, blend_schemes, params, foutput_csv_path, true, verbose); 
end

function [metrics, exp_names] = get_metrics(output_folder, starts, goals, ...
        control_schemes, blend_schemes, params, verbose)   
    Ns = 50; % start and goal pairs
    Nc = length(control_schemes); % control scheme
    Nb = length(blend_schemes); % blend scheme
    Nh = length(params); % hyperparameters
    Nm = 11; % num metrics 
    
    %% Set Default Values
    metrics = -ones(Ns, Nc, Nb, Nh, Nm);
    exp_names = cell(Ns * Nc * Nb * Nh, 1); 
    
    %% Extract Data
    dirs = dir(fullfile(output_folder));
    index = 0; 
    for h = 1:Nh
        for j = 1:Nc
            for k = 1:Nb
                for i = 1:Ns
                    index = index + 1; 
                    p = params{h};  
                    p.start = starts{i}; 
                    p.goal = goals{i}; 
                    p.control_scheme = control_schemes{j};
                    p.blend_scheme = blend_schemes{k}; 
                    p.hyperparam_str = get_hyperparam_string(p); 
                    p.run_planner = false; 
                    p.run_brs = false; 
                    exp = load_exp(p); 
                    p = Planner(exp);
                    exp_name = p.exp_name; 
                    exp_ran = false; 
                    exp_names{index} = exp_name; 
                    for it = 1:length(dirs)
                        exp_folder = dirs(it, :); 
                        if ~strcmp(exp_folder.name, exp_name)
                           continue;
                        end 
                        f = fullfile(exp_folder.folder, exp_folder.name, 'final_state.mat');
                        if ~isfile(f)
                            continue;
                        end 
                        load(f, 'obj'); 
                        timestamp =  size(obj.blend_traj, 2) * obj.dt; 
                        if timestamp == 0
                            fprintf("Crashed at beginning %s\n", exp_name); 
                            continue
                        end 
                        num_safety = sum(obj.blend_traj(6, :) == 0); 
                        metrics(i, j, k, h, :) = [obj.scores.avg_lin_jerk, ...
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
                        if verbose && obj.termination_state == -1
                           fprintf("Errored exp: %s\n", exp_name);  
                        elseif verbose && obj.termination_state == 1
                           fprintf("Crashed exp: %s\n", exp_name);  
                        elseif verbose && obj.termination_state == 2
                           fprintf("Tle exp: %s\n", exp_name); 
                        end
                        exp_ran = true;
                    end 
                    if ~exp_ran
                        fprintf("Skipped %s index %d\n", exp_name, i); 
                    end
                end
            end
        end 
    end
end 

function write_metrics_table(metrics, control_schemes, blend_schemes, params, csv_path, filtered, verbose)
    %% Initialize metric cell columns
    Nc = length(control_schemes); 
    Nb = length(blend_schemes); 
    Nh = length(params); 
    Ne = Nc * Nb * Nh; 
    alg_names = cell(Ne, 1); 
    reached_goal = cell(Ne, 1); 
    crashed = cell(Ne, 1);  
    time_limit_exceeded = cell(Ne, 1);  
    errored = cell(Ne, 1);  
    total_experiments = cell(Ne, 1);  
    avg_lin_jerk = cell(Ne, 1);  
    std_lin_jerk = cell(Ne, 1); 
    avg_ang_jerk = cell(Ne, 1);  
    std_ang_jerk = cell(Ne, 1);      
    avg_safety_score = cell(Ne, 1);  
    std_safety_score = cell(Ne, 1);      
    avg_dist_to_opt_traj = cell(Ne, 1);  
    std_dist_to_opt_traj = cell(Ne, 1);      
    avg_time_taken = cell(Ne, 1);  
    std_time_taken = cell(Ne, 1);  
    avg_num_safety_ctrls = cell(Ne, 1);  
    std_num_safety_ctrls = cell(Ne, 1);  
    table_column_names = {
        'Algorithm', ...
        'Num Reached Goal', ...
        'Num Crashed', ...
        'Num Time Exceeded', ...
        'Num Errored', ...
        'Num Total Experiments', ...
        'Avg Lin Jerk', ...
        'Std Lin Jerk', ...
        'Avg Ang Jerk', ...
        'Std Ang Jerk', ...
        'Avg Safety Score', ...
        'Std Safety Score', ...       
        'Avg Dist to Opt Traj', ...
        'Std Dist to Opt Traj', ...
        'Avg Time Taken', ...
        'Std Time Taken', ...
        'Avg Num Safety Ctrls', ...
        'Std Num Safety Ctrls'
    };
    
    %% Aggregate and log experiment run statistics
    index = 0; 
    for j = 1:length(control_schemes)
        for k = 1:length(blend_schemes)
             for h = 1:length(params)
                index = index + 1; 
                hs = params{h}.hyperparam_str; 
                bs = blend_schemes{k}; 
                cs = control_schemes{j}; 
                alg_name = sprintf("blend_%s_control_%s_%s", bs, cs, hs);
                if ~filtered
                    alj = mean(metrics(:, j, k, h, 1));
                    slj = std(metrics(:, j, k, h, 1)); 
                    aaj = mean(metrics(:, j, k, h, 2));
                    saj = std(metrics(:, j, k, h, 2)); 
                    assh = mean(metrics(:, j, k, h, 3));
                    sssh = std(metrics(:, j, k, h, 3)); 
                    adot = mean(metrics(:, j, k, h, 4));
                    sdot = std(metrics(:, j, k, h, 4)); 
                    ati = mean(metrics(:, j, k, h, 5));
                    sti = std(metrics(:, j, k, h, 5)); 
                    ansc = mean(metrics(:, j, k, h, 6));
                    snsc = std(metrics(:, j, k, h, 6)); 
                    termination = metrics(:, j, k, h, 7);
                    num_reached_goal = sum(termination == 0); 
                    num_crashed = sum(termination == 1); 
                    num_tle = sum(termination == 2); 
                    num_error = sum(termination == -1); 
                    total_exp = num_reached_goal + num_crashed + num_tle + num_error;
                else 
                    termination = metrics(:, j, k, h, 7);
                    valid_ind = termination == 0;
                    num_reached_goal = sum(termination == 0); 
                    num_crashed = sum(termination == 1); 
                    num_tle = sum(termination == 2); 
                    num_error = sum(termination == -1); 
                    total_exp = num_reached_goal + num_crashed + num_tle + num_error;
                    alj = mean(valid_ind .* metrics(:, j, k, h, 1));
                    slj = std(valid_ind .* metrics(:, j, k, h, 1)); 
                    aaj = mean(valid_ind .* metrics(:, j, k, h, 2));
                    saj = std(valid_ind .* metrics(:, j, k, h, 2)); 
                    assh = mean(valid_ind .* metrics(:, j, k, h, 3));
                    sssh = std(valid_ind .* metrics(:, j, k, h, 3)); 
                    adot = mean(valid_ind .* metrics(:, j, k, h, 4));
                    sdot = std(valid_ind .* metrics(:, j, k, h, 4)); 
                    ati = mean(valid_ind .* metrics(:, j, k, h, 5));
                    sti = std(valid_ind .* metrics(:, j, k, h, 5)); 
                    ansc = mean(valid_ind .* metrics(:, j, k, h, 6));
                    snsc = std(valid_ind .* metrics(:, j, k, h, 6)); 
                end
                
                alg_names{index} = alg_name; 
                reached_goal{index}  = num_reached_goal; 
                crashed{index}  = num_crashed; 
                time_limit_exceeded{index}  = num_tle; 
                errored{index}  = num_error; 
                total_experiments{index}  = total_exp; 
                avg_lin_jerk{index}  = alj; 
                std_lin_jerk{index}  = slj; 
                avg_ang_jerk{index}  = aaj; 
                std_ang_jerk{index}  = saj;     
                avg_safety_score{index}  = assh; 
                std_safety_score{index}  = sssh;     
                avg_dist_to_opt_traj{index}  = adot; 
                std_dist_to_opt_traj{index}  = sdot;     
                avg_time_taken{index}  = ati; 
                std_time_taken{index}  = sti; 
                avg_num_safety_ctrls{index}  = ansc; 
                std_num_safety_ctrls{index}  = snsc; 

                if verbose
                    fprintf("%s %d/%d reached goal\n", alg_name, num_reached_goal, total_exp);
                    fprintf("%s %d/%d crashed\n", alg_name, num_crashed, total_exp);
                    fprintf("%s %d/%d ran out of time\n", alg_name, num_tle, total_exp);
                    fprintf("%s %d/%d errored\n", alg_name, num_error, total_exp);
                    fprintf("%s avg lin jerk: %f std: %f\n", alg_name, alj, slj);
                    fprintf("%s avg ang jerk: %f std: %f\n", alg_name, aaj, saj);
                    fprintf("%s avg safety_score: %f std: %f\n", alg_name, assh, sssh); 
                    fprintf("%s avg dist to opt traj: %f std: %f\n", alg_name, adot, sdot); 
                    fprintf("%s avg time taken: %f std: %f\n", alg_name, ati, sti); 
                    fprintf("%s avg num safety controls taken: %f std: %f\n", alg_name, ansc, snsc);
                end 
            end 
        end 
    end
    
    %% Write table
    t = table(alg_names, ... 
            reached_goal, ...
            crashed, ...
            time_limit_exceeded, ... 
            errored, ... 
            total_experiments, ...
            avg_lin_jerk, ...
            std_lin_jerk, ...
            avg_ang_jerk, ...
            std_ang_jerk, ...    
            avg_safety_score, ...; 
            std_safety_score, ...     
            avg_dist_to_opt_traj, ...
            std_dist_to_opt_traj, ...     
            avg_time_taken, ...
            std_time_taken, ...; 
            avg_num_safety_ctrls, ...
            std_num_safety_ctrls);
    t.Properties.VariableNames = table_column_names; 
    writetable(t, csv_path); 
end 
  