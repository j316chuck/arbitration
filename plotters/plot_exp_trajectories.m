function plot_exp_trajectories(index)
    if nargin < 1
        index = 1;
    end
    %% Change these parameters
    output_name = "/results/1_31"; 
    nav_task_type = "sampled"; %"smoke";  
    [starts, goals] = get_point_nav_tasks(nav_task_type); 
    control_schemes = {'switch'}; 
    blend_schemes = {'time_vary_alpha_open_loop_safety_control', 'sample_safety_control', 'replan_safe_traj', 'none'}; %get_key_blend_schemes()
    labels = {'value alpha (open)', 'sample alpha (static)', 'mo and karen', 'cdc'}; %get_new_alpha_blend_function_names(); 
    colors = get_all_blend_scheme_colors();
    hyperparam_str = "default"; 
    hyperparam_sets = get_hyperparam_sets(hyperparam_str); %replan_zls %time_vary_alpha_open"
    Nh = length(hyperparam_sets); 
    Nc = length(control_schemes); 
    Nb = length(blend_schemes); 
    start = starts{index};  
    goal = goals{index};
    title_str = "unknown_key_alpha_blending_schemes_switch_control"; 
    plot_hyperparam = false; 
    plot_all_zls = true;

    %% Extract Data Paths
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, output_name); 
    results_folder = strcat(repo.path, output_name, '/results');
    dirs = dir(fullfile(output_folder));
    st = sprintf("start_[%.2f %.2f %.2f]", start(1), start(2), start(3)); 
    go = sprintf("goal_[%.2f %.2f %.2f]", goal(1), goal(2), goal(3));
    plot_name = sprintf("%s %s %s", title_str, st, go); 
    
    %% Plot Trajectories
    clf
    figure(1); 
    hold on; 
    first_iteration = true;
    for h = 1:Nh
        for j = 1:Nc
            for k = 1:Nb
                params = hyperparam_sets{h};  
                params.start = start;
                params.goal = goal;
                bs = blend_schemes{k}; 
                cs = control_schemes{j}; 
                if plot_hyperparam
                    color = colors(h, :);
                    legend_name = labels{h};
                else
                    color = colors(k, :);
                    legend_name = labels{k};
                end 
                params.blend_scheme = bs;
                params.control_scheme = cs;
                params.hyperparam_str = get_hyperparam_string(params); 
                params.run_planner = false;
                params.run_brs = false; 
                exp = load_exp(params); 
                pb = Planner(exp);
                alg_name = sprintf("blend_%s_control_%s", bs, cs); 
                exp_name = pb.exp_name; 
                exp_ran = false; 
                zls_theta = calc_zls_theta(start, goal); 
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
                    log_failed_experiments(obj, exp_name); 
                    if first_iteration
                        first_iteration = false;
                        plot_env(obj);
                        plot_traj(obj.reach_avoid_planner.opt_traj, 'reach avoid', 'green'); 
                    end
                    if plot_all_zls
                        plot_zls(obj, legend_name, zls_theta, color); 
                    else 
                        plot_zls(obj, 'bookstore', zls_theta, '#CC1FCB'); 
                    end
                    plot_traj(obj.blend_traj, legend_name, color); 
                    set_plot_params(plot_name);
                    exp_ran = true; 
                end 
                if ~exp_ran
                    fprintf("Missing exp: %s\n", exp_name); 
                end 
            end 
        end 
    end 
    set_plot_params(plot_name);
    
    %% Saving to png and fig folder
    results_png_folder = strcat(results_folder, "/all_traj_png"); 
    if ~exist(results_png_folder, 'dir')
        mkdir(results_png_folder); 
    end 
    results_fig_folder = strcat(results_folder, "/all_traj_fig"); 
    if ~exist(results_fig_folder, 'dir')
        mkdir(results_fig_folder); 
    end 
    fig_path = strcat(results_fig_folder, "/", plot_name, ".fig"); 
    savefig(fig_path); 
    png_path = strcat(results_png_folder, "/", plot_name, ".png");
    saveas(gcf, png_path); 
    hold off;
    pause(1);
end

function theta = calc_zls_theta(start, goal)
    dy = goal(2) - start(2);
    dx = goal(1) - start(1);
    theta = atan(dy/dx); 
end 

function log_failed_experiments(obj, exp_name)
    if obj.termination_state == -1
        fprintf("Exp %s errored\n", exp_name); 
    elseif obj.termination_state == 1
        fprintf("Exp %s crashed\n", exp_name); 
    elseif obj.termination_state == 2
        fprintf("Exp %s tle\n", exp_name); 
    end 
end

function plot_traj(blend_traj, name, color)
    xs = blend_traj(1, :); 
    ys = blend_traj(2, :); 
    ths = blend_traj(3, :);  
    s = scatter(xs, ys, 5, 'black', 'filled', 'DisplayName', name);
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.1;
end 

function set_plot_params(name)
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    l = legend('Location', 'SouthWest');
    set(l, 'Interpreter', 'none', 'fontsize', 5);
    set(l,'position', [0.25 0.75 0.02 0.02]);
    title(name, 'Interpreter', 'None');
end 

function plot_env(obj)
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'occ_fmm_map', 'LineWidth', 1, 'color', 'blue');
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal_shape', 'color', 'red');
    scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
    scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
end 

function plot_zls(obj, name, theta, color)
    zls = obj.blending.zero_level_set; 
    name = sprintf("BRS %s (theta=%.2f, levelset=%.2f)", name, theta, zls);
    [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], theta);
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', name, 'color', color);
end 

