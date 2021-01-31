repo = what("arbitration");
path = strcat(repo.path, "/outputs/", ...
"unknown_bookstore_map_start_[-1.38 -3.22 1.57]_goal_[0.78 1.55 1.57]_blend_none_control_switch_grid_med_large_zls_0.150_replan_dt_1.000_obs_weight_1.000");

f = dir(path);
for i = 1:length(f)
    figFile = fullfile(strcat(f(i).folder, '/', f(i).name)); 
    if contains(figFile, "fig")
        fig = openfig(figFile, 'invisible');
        [pathstr, name, ext] = fileparts(figFile);
        s = strcat(pathstr, "/", name, ".png");
        saveas(fig, s);
        close all;
    end 
end 
