function output_analysis()  
    %% Change these parameters
    verbose = true;
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs/');
    results_folder = strcat(output_folder, '/results/');
    output_mat_path = sprintf("%s/%s", results_folder, 'output_analysis.mat');
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end
    control_schemes = {'switch'};
    blend_schemes = {'time_vary_alpha_open_loop_safety_control', 'replan_safe_traj', 'none', 'sample_safety_control'};
    [starts, goals] = get_point_nav_tasks("sampled"); %"sampled" %"smoke_test"
    params = get_hyperparam_sets("default"); %"time_vary_open" %"default" %"replan_zls"

    %% Get metrics
     [metrics, exp_names] = get_metrics(output_folder, ... 
         starts, goals, control_schemes, blend_schemes, params, verbose);
    save(output_mat_path, 'metrics', 'exp_names'); 
    
    %% Write table
    load(output_mat_path, 'metrics'); 
    ar_output_csv_path = sprintf("%s/%s", results_folder, 'all_reached_goal_output_analysis.csv'); 
    rg_output_csv_path = sprintf("%s/%s", results_folder, 'reached_goal_output_analysis.csv'); 
    output_csv_path = sprintf("%s/%s", results_folder, 'output_analysis.csv'); 
    write_metrics_table(metrics, control_schemes, blend_schemes, params, ...
        ar_output_csv_path, true, true, verbose); 
    write_metrics_table(metrics, control_schemes, blend_schemes, params, ...
        rg_output_csv_path, false, true, verbose); 
    write_metrics_table(metrics, control_schemes, blend_schemes, params, ...
        output_csv_path, false, false, verbose); 
end

function [metrics, exp_names] = get_metrics(output_folder, starts, goals, ...
        control_schemes, blend_schemes, params, verbose)   
    Ns = 100; % start and goal pairs
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

function write_metrics_table(metrics, control_schemes, blend_schemes, ...
    params, csv_path, use_same, use_reached_goal, verbose)
    %% Initialize metric cell columns
    Ns = size(metrics, 1); 
    Nc = length(control_schemes); 
    Nb = length(blend_schemes); 
    Nh = length(params); 
    Ne = Nc * Nb * Nh; 
    alg_names = cell(Ne, 1); 
    success = cell(Ne, 1); 
    crashed = cell(Ne, 1);  
    time_limit_exceeded = cell(Ne, 1);  
    errored = cell(Ne, 1);  
    total_experiments = cell(Ne, 1); 
    num_experiments = cell(Ne, 1); 
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
        'Num Experiments in metric', ...
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
        'Std Num Safety Ctrls', ...
     };
    
    %% Get all valid indices for fair comparison of blending and/or hyperparameter schemes
    valid_ind = ones(Ns, 1); 
    if use_same 
        for j = 1:Nc
            for h = 1:Nh
                for k = 1:Nb
                     terminate_success = (metrics(:, j, k, h, 7) == 0); 
                     valid_ind = valid_ind(:) .* terminate_success; 
                end
            end
        end 
    end
    
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
                if use_reached_goal
                    cur_valid = valid_ind(:) .* (metrics(:, j, k, h, 7) == 0); 
                else
                    cur_valid = valid_ind(:);
                end 
                alj = mean(metrics(:, j, k, h, 1) .* cur_valid);
                slj = std(metrics(:, j, k, h, 1) .* cur_valid); 
                aaj = mean(metrics(:, j, k, h, 2) .* cur_valid);
                saj = std(metrics(:, j, k, h, 2) .* cur_valid); 
                assh = mean(metrics(:, j, k, h, 3) .* cur_valid);
                sssh = std(metrics(:, j, k, h, 3) .* cur_valid); 
                adot = mean(metrics(:, j, k, h, 4) .* cur_valid);
                sdot = std(metrics(:, j, k, h, 4) .* cur_valid); 
                ati = mean(metrics(:, j, k, h, 5) .* cur_valid);
                sti = std(metrics(:, j, k, h, 5) .* cur_valid); 
                ansc = mean(metrics(:, j, k, h, 6) .* cur_valid);
                snsc = std(metrics(:, j, k, h, 6) .* cur_valid); 
                termination = metrics(:, j, k, h, 7);
                num_reached_goal = sum(termination == 0); 
                num_crashed = sum(termination == 1); 
                num_tle = sum(termination == 2); 
                num_error = sum(termination == -1); 
                total_exp = num_reached_goal + num_crashed + num_tle + num_error;
                num_exp = sum(cur_valid); 
               
                alg_names{index} = alg_name; 
                success{index}  = num_reached_goal; 
                crashed{index}  = num_crashed; 
                time_limit_exceeded{index}  = num_tle; 
                errored{index}  = num_error; 
                total_experiments{index}  = total_exp; 
                num_experiments{index} = num_exp; 
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
            success, ...
            crashed, ...
            time_limit_exceeded, ... 
            errored, ... 
            total_experiments, ...
            num_experiments, ...
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
  