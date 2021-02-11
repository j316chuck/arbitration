function plot_exp_with_aggregation()
    %% Change these parameters
    output_name = "/results/1_25_all_exps_backup/"; 
    control_schemes = {'switch'}; 
    blend_schemes = {'time_vary_alpha_open_loop', 'sample_safety_control', 'replan_safe_traj', 'none'}; %get_key_blend_schemes()
    hyperparam_str = "default"; 
    hyperparam_sets = get_hyperparam_sets(hyperparam_str); %replan_zls %time_vary_alpha_open"
    Nb = length(blend_schemes); 

    %% Extract and Make Data Paths
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, output_name); 
    results_folder = strcat(repo.path, output_name, '/results');
    if ~exist(results_folder, 'dir')
        mkdir(results_folder); 
    end 
    dirs = dir(fullfile(output_folder));
    exp_states = {'success', 'crashed', 'tle'};
    for i=1:length(exp_states)
        for k=1:Nb
            folder_name = sprintf("%s_%s", exp_states{i}, blend_schemes{k});
            path = strcat(results_folder, "/", folder_name); 
            if ~exist(path, 'dir')
                mkdir(path); 
            end 
        end 
    end 
    
    %% Plot Trajectories
    for it = 1:length(dirs)
        exp_folder = dirs(it, :); 
        f = fullfile(exp_folder.folder, exp_folder.name, 'final_state.mat');
        if ~isfile(f)
            continue;
        end 
        load(f, 'obj');
        [blend_scheme, bs_index] = get_blend_scheme(obj, blend_schemes); 
        % figure set up
        figure(1); 
        clf
        hold on; 
        % plot env
        contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
        contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'occ_fmm_map', 'LineWidth', 1, 'color', 'blue');
        contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal_shape', 'color', 'red');
        scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
        scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
        % plot traj
        traj_name = sprintf("%s_robot_trajectory", blend_scheme); 
        plot_traj_probs(obj.blend_traj(1, :), obj.blend_traj(2, :), obj.blend_traj(3, :), obj.blend_traj(6, :), traj_name); 
        % plot zls
        zls = obj.blending.zero_level_set; 
        theta = calc_zls_theta(obj.start, obj.goal); 
        brs_name = sprintf("BRS (theta=%.2f, levelset=%.2f)", theta, zls);
        [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], theta);
        contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', brs_name, 'color', 'green');
        colorbar;
        % set fig params
        view(0, 90)
        set(gcf, 'color', 'white')
        set(gcf, 'position', [0, 0, 800, 800])
        xlabel('x (meters)');
        ylabel('y (meters)');
        l = legend('Location', 'SouthWest');
        set(l, 'Interpreter', 'none', 'fontsize', 5);
        set(l,'position', [0.25 0.75 0.02 0.02]);
        nav_str = sprintf("start_[%.2f %.2f %.2f]_goal_[%.2f %.2f %.2f]", ...
            obj.start(1), obj.start(2), obj.start(3), obj.goal(1), obj.goal(2), obj.goal(3)); 
        unknown_str = get_unknown_str(obj); 
        title_name = sprintf("%s_map_%s_%s_%s", unknown_str, blend_scheme, obj.control_scheme, nav_str); 
        title(title_name, 'Interpreter', 'None');
        % save fig
        exp_state = exp_states{obj.termination_state+1}; 
        png_path = sprintf("%s/%s_%s/%s.png", results_folder, ...
            exp_state, blend_scheme, nav_str); 
        saveas(gcf, png_path);
        hold off; 
    end         
end

function str = get_unknown_str(obj)
    if obj.is_unknown_environment
        str = "unknown";
    else
        str = "known";
    end 
end 

function theta = calc_zls_theta(start, goal)
    dy = goal(2) - start(2);
    dx = goal(1) - start(1);
    theta = atan(dy/dx); 
end 

function [blend_scheme, bs_index] = get_blend_scheme(obj, blend_schemes)
    for k = 1:length(blend_schemes)
        if strcmp(obj.blend_scheme, blend_schemes{k})
            bs_index = k; 
        end 
    end
    blend_scheme = blend_schemes{bs_index}; 
end 

