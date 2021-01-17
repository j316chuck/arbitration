function plot_trajectories(index)
    %% Change these parameters
    repo = what("arbitration"); 
    output_folder = strcat(repo.path, '/outputs'); 
    results_folder = strcat(repo.path, '/outputs/results');
    [starts, goals] = get_all_smoke_test_cases(); 
    control_schemes = {'switch'}; 
    blend_schemes = get_key_blend_schemes(); 
    colors = get_all_blend_scheme_colors(); 
    Nc = length(control_schemes); 
    Nb = length(blend_schemes); 
    start = starts{index};  
    goal = goals{index};
    title_str = "key_blend_switch"; 


    %% Extract data 
    dirs = dir(fullfile(output_folder));
    st = sprintf("start_[%.2f %.2f %.2f]", start(1), start(2), start(3)); 
    go = sprintf("goal_[%.2f %.2f %.2f]", goal(1), goal(2), goal(3));
    plot_name = sprintf("%s %s %s", title_str, st, go); 
    clf
    figure(1); 
    hold on; 
    first_iteration = true;
    for j = 1:Nc
        for k = 1:Nb
            color = colors(k, :); 
            cs = control_schemes{j};
            bs = blend_schemes{k}; 
            alg_name = sprintf("blend_%s_control_%s", bs, cs); 
            legend_name = sprintf("%s_%s", bs, cs); 
            exp_name = sprintf("%s_%s_%s", st, go, alg_name); 
            exp_ran = false; 
            zls_theta = 0; 
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
                if obj.termination_state == -1
                    fprintf("Exp %s errored", exp_name); 
                elseif obj.termination_state == 1
                    fprintf("Exp %s crashed", exp_name); 
                elseif obj.termination_state == 2
                    fprintf("Exp %s tle", exp_name); 
                end 
                exp_ran = true; 
                if first_iteration
                    first_iteration = false;
                    plot_env(obj);
                    plot_zls(obj, zls_theta); 
                    plot_traj(obj.reach_avoid_planner.opt_traj, 'reach avoid', 'green'); 
                end 
                plot_traj(obj.blend_traj, legend_name, color); 
                set_plot_params(plot_name);
            end 
            if ~exp_ran
                fprintf("Missing exp: %s\n", exp_name); 
            end 
        end 
    end 
    figpath = strcat(results_folder, "/", plot_name, ".fig"); 
    savefig(figpath); 
end 

function plot_traj(blend_traj, name, color)
    xs = blend_traj(1, :); 
    ys = blend_traj(2, :); 
    ths = blend_traj(3, :);  
    s = scatter(xs, ys, 15, 'black', 'filled', 'DisplayName', name);
    s.HandleVisibility = 'off';
    q = quiver(xs, ys, cos(ths), sin(ths), 'Color', color);
    q.DisplayName = name;
    q.HandleVisibility = 'on';
    q.ShowArrowHead = 'on';
    q.AutoScale = 'on';
    q.AutoScaleFactor = 0.3;
end 

function set_plot_params(name)
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    l = legend('Location', 'SouthWest');
    set(l, 'Interpreter', 'none')
    title(name, 'Interpreter', 'None');
end 

function plot_env(obj)
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.binary_occ_map, [0 0], 'DisplayName', 'binary_obs_map', 'color', 'black');
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, obj.exp.obs_map, [0 0], 'DisplayName', 'occ_fmm_map', 'LineWidth', 1, 'color', 'blue');
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, -obj.exp.goal_map_3d(:, :, 1), [0 0], 'DisplayName', 'goal_shape', 'color', 'red');
    scatter(obj.goal(1), obj.goal(2), 100, 'k', 'x', 'DisplayName', 'goal'); 
    scatter(obj.start(1), obj.start(2), 75, 'b', 'o', 'filled', 'DisplayName', 'start'); 
end 

function plot_zls(obj, theta)
    zls = obj.blending.zero_level_set; 
    name = sprintf("BRS (theta=%.2f, levelset=%.2f)", theta, zls);
    [~, vf_slice] = proj(obj.exp.grid_3d, obj.brs_planner.valueFun, [0 0 1], theta);
    contour(obj.exp.grid_2d.xs{1}, obj.exp.grid_2d.xs{2}, vf_slice, [zls, zls], 'DisplayName', name, 'color', '#CC1FCB');
end 

