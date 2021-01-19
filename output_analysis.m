function hyperparam_analysis()  
    %% Change these parameters
    verbose = true;
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs');
    results_folder = strcat(output_folder, '/results');
    output_mat_path = sprintf("%s/%s", results_folder, 'output_analysis.mat');
    output_csv_path = sprintf("%s/%s", results_folder, 'output_analysis.csv'); 
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end
    [starts, goals] = get_point_nav_tasks("sampled"); %"smoke_test"
    control_schemes = get_all_control_schemes();
    blend_schemes = get_key_blend_schemes(); 
    
    %% Get metrics
     [metrics, lin_jerk_matrix, ang_jerk_matrix, safety_score_matrix, ...
    dist_to_opt_traj_matrix, timestamp_matrix, ...
    num_safety_control_matrix, termination_matrix, exp_name_matrix ...
    crashed_matrix, reached_matrix, tle_matrix, errored_matrix] = ...
    get_metrics(output_folder, starts, goals, control_schemes, blend_schemes, verbose);
    save(output_mat_path); 
    
    %% Write table
    load(output_mat_path, 'metrics'); 
    write_metrics_table(metrics, control_schemes, blend_schemes, output_csv_path, verbose); 
end

function [metrics, lin_jerk_matrix, ang_jerk_matrix, safety_score_matrix, ...
        dist_to_opt_traj_matrix, timestamp_matrix, ...
        num_safety_control_matrix, termination_matrix, exp_name_matrix ...
        crashed_matrix, reached_matrix, tle_matrix, errored_matrix] = ... 
        get_metrics(output_folder, starts, goals, control_schemes, blend_schemes, verbose)   
    Ns = 30; % start and goal pairs
    Nc = length(control_schemes); % control scheme
    Nb = length(blend_schemes); % blend scheme
    Nh = 4; 
    Nm = 11; % num metrics
    
    %% Set Default Values
    metrics = -ones(Ns, Nc, Nb, Nh, Nm); 
    lin_jerk_matrix = -ones(Ns, Nc, Nb, Nh);
    ang_jerk_matrix = -ones(Ns, Nc, Nb, Nh);
    safety_score_matrix = -ones(Ns, Nc, Nb, Nh);
    dist_to_opt_traj_matrix = -ones(Ns, Nc, Nb, Nh);
    timestamp_matrix = -ones(Ns, Nc, Nb, Nh);
    num_safety_control_matrix = -ones(Ns, Nc, Nb, Nh);
    termination_matrix = -ones(Ns, Nc, Nb, Nh);
    exp_name_matrix = cell(Ns, Nc, Nb, Nh); 
    reached_matrix = zeros(Ns, Nc, Nb, Nh); 
    crashed_matrix = zeros(Ns, Nc, Nb, Nh); 
    tle_matrix = zeros(Ns, Nc, Nb, Nh); 
    errored_matrix = zeros(Ns, Nc, Nb, Nh); 
    
    %% Extract Data
    dirs = dir(fullfile(output_folder));
    for j = 1:Nc
        for k = 1:Nb
            for i = 1:Ns
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
                    fprintf("Skipped %s_%s_%s_%s\n", st, go, cs, bs); 
                end
            end
        end
    end
end 

function write_metrics_table(metrics, control_schemes, blend_schemes, csv_path, verbose)
    %% Initialize metric cell columns
    Nc = length(control_schemes); 
    Nb = length(blend_schemes); 
    Ne = Nc * Nb; 
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
        'Num TLE', ...
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
            index = index + 1; 
            bs = blend_schemes{k}; 
            cs = control_schemes{j}; 
            alg_name = sprintf("blend_%s_control_%s", bs, cs);
            termination = metrics(:, j, k, 7);
            num_reached_goal = sum(termination == 0); 
            num_crashed = sum(termination == 1); 
            num_tle = sum(termination == 2); 
            num_error = sum(termination == -1); 
            total_exp = num_reached_goal + num_crashed + num_tle + num_error;
            alj = mean(metrics(:, j, k, 1));
            slj = std(metrics(:, j, k, 1)); 
            aaj = mean(metrics(:, j, k, 2));
            saj = std(metrics(:, j, k, 2)); 
            assh = mean(metrics(:, j, k, 3));
            sssh = std(metrics(:, j, k, 3)); 
            adot = mean(metrics(:, j, k, 4));
            sdot = std(metrics(:, j, k, 4)); 
            ati = mean(metrics(:, j, k, 5));
            sti = std(metrics(:, j, k, 5)); 
            ansc = mean(metrics(:, j, k, 6));
            snsc = std(metrics(:, j, k, 6)); 
            
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
  