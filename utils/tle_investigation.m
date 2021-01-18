function tle_investigate()  
    %% Change these parameters
    verbose = true;
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs');
    results_folder = strcat(output_folder, '/results');
    output_path = strcat(results_folder, "/tle_investigate.mat"); 
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end
    [starts, goals] = get_point_nav_tasks("sampled"); %"smoke_test"
    control_schemes = get_all_control_schemes();
    blend_schemes = get_key_blend_schemes(); 
    
    %% Get metrics
    %investigate(output_folder, output_path, starts, goals, control_schemes, blend_schemes);  
    load(output_path); 
    for i = 1:length(reached_time)
        x = reached_time{i}; 
        s = exp_schemes{i}; 
        fprintf("%s avg time: %f std time: %f max time: %f min time: %f\n", ...
            s, mean(x), std(x), max(x), min(x)); 
    end 
end

function investigate(output_folder, output_path, starts, goals, control_schemes, blend_schemes)   
    Ns = 40; % start and goal pairs
    Nc = length(control_schemes); % control scheme
    Nb = length(blend_schemes); % blend scheme
    reached_time = cell(Nc * Nb, 1); 
    crashed_time = cell(Nc * Nb, 1); 
    exp_schemes = cell(Nc * Nb, 1); 

    %% Extract Data
    dirs = dir(fullfile(output_folder));
    index = 0; 
    for j = 1:Nc
        for k = 1:Nb
            index = index + 1; 
            reached_time_taken = []; 
            crashed_time_taken = []; 
            cs = control_schemes{j};
            bs = blend_schemes{k}; 
            exp_type = sprintf("blend_%s_control_%s", bs, cs);
            for i = 1:Ns
                start = starts{i}; 
                goal = goals{i}; 
                st = sprintf("start_[%.2f %.2f %.2f]", start(1), start(2), start(3)); 
                go = sprintf("goal_[%.2f %.2f %.2f]", goal(1), goal(2), goal(3));
                exp_name = sprintf("%s_%s_%s", st, go, exp_type); 
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
                    timestamp =  size(obj.blend_traj, 2); 
                    if obj.termination_state == 0
                        reached_time_taken(end+1) = timestamp;  
                    elseif obj.termination_state == 1
                        crashed_time_taken(end+1) = timestamp; 
                    end
                end 
            end
            reached_time{index} = reached_time_taken;
            crashed_time{index} = crashed_time_taken;
            exp_schemes{index} = exp_type; 
        end
    end
    save(output_path, 'reached_time', 'crashed_time', 'exp_schemes'); 
end 
