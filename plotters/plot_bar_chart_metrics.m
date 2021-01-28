function plot_bar_chart_metrics()
    clear all
    clf

    %% Load the data.
    % The data is stored as an N x M x K matrix where
    %   N = initial/goal condition pair 
    %   M = control switching scheme (e.g. on or off at runtime)
    %   K = blending method type (e.g. fixed alpha blending)
    % This will be average over some of these dims
    load('output_analysis.mat');

    % Choose the metric to plot.
    % Types of supported metrics:
    %   dist_to_opt_traj
    %   lin_jerk
    %   ang_jerk
    %   safety_score
    %   termination 
    metric = 'dist_to_opt_traj';

    abs_on = false; % should we take absolute value of the data?

    % Grab the data to plot.
    if strcmp(metric , 'dist_to_opt_traj')
        metric_matrix = dist_to_opt_traj_matrix;
        metric_name = 'Dist to opt traj';
        title_name = 'Dist to opt traj (avg over initial cond)'; 
    elseif strcmp(metric, 'lin_jerk')
        metric_matrix = lin_jerk_matrix;
        metric_name = 'Linear Jerk (m/s^3)';
        title_name = 'Linear Jerk (avg over traj. and initial cond)'; 
        abs_on = true;
    elseif strcmp(metric, 'ang_jerk')
        metric_matrix = ang_jerk_matrix;
        metric_name = 'Angular Jerk (rad/s^3)';
        title_name = 'Angular Jerk (avg over traj. and initial cond)'; 
        abs_on = true;
    elseif strcmp(metric, 'safety_score')
        metric_matrix = safety_score_matrix;
        metric_name = 'Safety Score';
        title_name = 'Safety Score (avg over traj. and initial cond)';   
    elseif strcmp(metric, 'termination')
        % -1 is the program crashed
        % 0 is reached goal
        % 1 is robot crashed 
        % 2 is max num timestamps reached
        metric_matrix = termination_matrix;
        metric_name = 'Goal-Reaching';
        title_name = 'Goal-Reaching (avg over initial cond)';   
    else
        error('unsupported metric type!');
    end

    %% Compute the mean and standard deviation of the data.

    % Choose which dimension to average over:
    %   dim = 1 --> average over initial conds
    %   dim = 2 --> average over if safety controller is active at runtime
    %   dim = 3 --> average over method types.
    avg_dim = 1;

    % Plot two separate plots, one for condition without safety ctrl, other
    % with safety control. 
    no_safety_ctrl_idx = 1;
    safety_ctrl_idx = 2; 

    % Average over specified dimension.
    if abs_on
        method_and_ctrller_mean = mean(abs(metric_matrix), avg_dim);
    else
        method_and_ctrller_mean = mean(metric_matrix, avg_dim);
    end
    method_and_ctrller_mean = squeeze(method_and_ctrller_mean); % get rid of dimensions of length 1.

    % Get the standard deviation.
    if abs_on
        method_and_ctrller_stdev = std(abs(metric_matrix), 0, avg_dim);
    else
        method_and_ctrller_stdev = std(metric_matrix, 0, avg_dim);
    end
    method_and_ctrller_stdev = squeeze(method_and_ctrller_stdev); % get rid of dimensions of length 1.

    %% Setup labels for each of the plots
    blend_scheme_names = {'value alpha (open)',...
                    'value alpha (closed)', ...
                    'safety value', ...
                    'static alpha', ...  
                    'sample alpha (static)', ...
                    'min inv. alpha (static)',	...
                    'triangle replan', 			...
                    'Mo and Karen', 		...
                    'CDC' 						};
    num_schemes = 1:length(blend_scheme_names);            
    blend_scheme_labels = categorical(blend_scheme_names);

    %% Compute standard deviation info.

    % Number of standard deviations from the mean to plot.
    num_std_devs = 1; 

    % Compute the lower and upper bound of the standard deviation.
    errlow = -num_std_devs * method_and_ctrller_stdev;
    errhigh = num_std_devs * method_and_ctrller_stdev;

    method_colors = [255, 166, 0; ...
                     255, 124, 67; ...
                     249, 93, 106; ...
                     212, 80, 135; ...
                     160, 81, 149; ...
                     102, 81, 145; ...
                     47, 75, 124; ...
                     0, 63, 92;
                     127,127,127] ./ 255.;

    %% Plot the bar chart!
    figure(1)
    for ctrl_idx=1:2
        % Index == 1 --> Safety controller OFF during execution
        % Index == 2 --> Safety control ON during execution
        subplot(1,2,ctrl_idx)
        hold on
        for method_idx=1:length(method_and_ctrller_mean(1,:))
            curr_color = method_colors(method_idx,:);

            % Plot the bar chart.
            br = bar(num_schemes(method_idx), method_and_ctrller_mean(ctrl_idx,method_idx));
            br.FaceColor = curr_color;

            % Plot the standard deviation from the mean.
            er = errorbar(num_schemes(method_idx),method_and_ctrller_mean(ctrl_idx,method_idx),errlow(ctrl_idx,method_idx),errhigh(ctrl_idx,method_idx), 's');    
            er.Color = [0 0 0];                            
            er.LineStyle = 'none';
        end
        % Make the background of plot white.
        set(gcf, 'color', 'w');

        % Setup the labels
        ylim([min(method_and_ctrller_mean+errlow, [], 'all'), max(method_and_ctrller_mean+errhigh, [], 'all')]);
        ylabel(metric_name);
        xticks(num_schemes)
        xticklabels(blend_scheme_names)
        xtickangle(45)

        % Add subplot title
        if ctrl_idx == 1
            ctrl_name = 'No Safety Controller';
        else
            ctrl_name = 'With Safety Controller';
        end
        title(ctrl_name)
    end

    % Add title over whole thing:
    sgtitle(title_name)
end


%% Desaturates color by 70 percent. 
function curr_color = desaturate_color(color)
    f = 0.7; % desaturate by 70%
    L = 0.3*color(1) + 0.6*color(2) + 0.1*color(3);
    curr_color(1) = color(1) + f * (L - color(1));
    curr_color(2) = color(2) + f * (L - color(2));
    curr_color(3) = color(3) + f * (L - color(3));
end
