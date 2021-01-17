function plot_scatter_metrics()
    repo = what("arbitration");
    folder = strcat(repo.path, "/outputs/results/"); 
    path = strcat(folder, "output_analysis.mat"); 
    load(path, 'metrics');
    metric_names = get_all_metric_names();
    colors = get_all_blend_scheme_colors(); 
    xi = 1; % change this
    yi = 3; % change this
    algs = [8, 9]; % change this
    
    % Scatter plot
    figure(5); 
    clf
    hold on
    blend_schemes = get_all_blend_schemes(); 
    control_schemes = get_all_control_schemes(); 
    for k = 1:length(blend_schemes)
        for j = 1:length(control_schemes)
            if ~any(find(algs == k))
                continue; 
            end 
            c = colors(k, :); 
            if mod(j, 2) 
                c = desaturate_color(c, 0.1); 
            end 
            xscore = metrics(:, j, k, xi); 
            yscore = metrics(:, j, k, yi); 
            label = sprintf("%s-%s", blend_schemes{k}, control_schemes{j}); 
            scatter(xscore, yscore, 50, c, 'DisplayName', label); 
        end
    end
    xticks(-1:0.5:3);
    yticks(-1:0.5:3);
    legend("Location", "northwest", "Interpreter", "None");
    t = sprintf("%s vs %s metrics", metric_names{yi}, metric_names{xi}); 
    title(t); 
    xlabel(metric_names{xi}); 
    ylabel(metric_names{yi});
end 
