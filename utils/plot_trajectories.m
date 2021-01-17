%% Change these parameters
verbose = false;
repo = what("arbitration"); 
output_folder = strcat(repo.path, '/outputs'); 
results_folder = strcat(repo.path, '/outputs/results');
[starts, goals] = get_all_smoke_test_cases();
smoke_testcase_index = 1;
start = starts{smoke_testcase_index};  
goal = goals{smoke_testcase_index}; 
control_schemes = {'switch'}; 
blend_schemes = get_key_blend_schemes(); 
colors = get_all_blend_scheme_colors(); 
Nc = length(control_schemes); 
Nb = length(blend_schemes); 


%% Extract data 
dirs = dir(fullfile(output_folder));
st = sprintf("start_[%.2f %.2f %.2f]", start(1), start(2), start(3)); 
go = sprintf("goal_[%.2f %.2f %.2f]", goal(1), goal(2), goal(3));
clf
figure(1); 
hold on; 
iteration = 1;
for j = 1:Nc
    for k = 1:Nb
        color = colors(k, :); 
        cs = control_schemes{j};
        bs = blend_schemes{k}; 
        alg_name = sprintf("blend_%s_control_%s", bs, cs); 
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
            exp_ran = true; 
            if iteration == 1
                plot_env(obj);
                plot_zls(obj, zls_theta); 
                plot_traj(obj.reach_avoid_planner.opt_traj, 'reach avoid', 'magenta'); 
            end 
            plot_traj(obj.blend_traj, alg_name, color); 
            set_plot_params(st, go);
            iteration = iteration + 1; 
        end 
        if ~exp_ran
            fprintf("Missing exp: %s\n", exp_name); 
        end 
    end 
end 
%savefig()

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

function set_plot_params(st, go)
    view(0, 90)
    set(gcf, 'color', 'white')
    set(gcf, 'position', [0, 0, 800, 800])
    xlabel('x (meters)');
    ylabel('y (meters)');
    l = legend('Location', 'NorthWest');
    set(l, 'Interpreter', 'none')
    name = sprintf("%s %s traj", st, go);
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

