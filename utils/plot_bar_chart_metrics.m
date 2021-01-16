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
metric = 'termination';

% Grab the data to plot.
if strcmp(metric , 'dist_to_opt_traj')
    metric_matrix = dist_to_opt_traj_matrix;
    metric_name = 'Dist to opt traj';
    title_name = 'Dist to opt traj (avg over initial cond)'; 
elseif strcmp(metric, 'lin_jerk')
    metric_matrix = lin_jerk_matrix;
    metric_name = 'Linear Jerk';
    title_name = 'Linear Jerk (avg over traj. and initial cond)'; 
elseif strcmp(metric, 'ang_jerk')
    metric_matrix = ang_jerk_matrix;
    metric_name = 'Angular Jerk';
    title_name = 'Angular Jerk (avg over traj. and initial cond)'; 
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

% Average over specified dimension.
method_and_ctrller_mean = mean(abs(metric_matrix), avg_dim);
method_and_ctrller_mean = squeeze(method_and_ctrller_mean); % get rid of dimensions of length 1.

% Get the standard deviation.
method_and_ctrller_stdev = std(abs(metric_matrix), 0, avg_dim);
method_and_ctrller_stdev = squeeze(method_and_ctrller_stdev); % get rid of dimensions of length 1.

%% Setup labels for each of the plots
num_conds = 1:18;
blend_scheme_names = {'value alpha (open)',...
                'value alpha (closed)', ...
                'safety value', ...
                'Static alpha', ...  
                'Sample alpha (static)', ...
                'Min inv. alpha (static)',	...
                'Triangle replan', 			...
                'Mo and Karen', 		...
                'CDC' 						};
blend_scheme_labels = categorical(blend_scheme_names);

%% Compute standard deviation info.

% Number of standard deviations from the mean to plot.
num_std_devs = 1; 

% Compute the lower and upper bound of the standard deviation.
errlow = -num_std_devs * method_and_ctrller_stdev;
errhigh = num_std_devs * method_and_ctrller_stdev;

method_colors = [1,0,0; ...
                 0.9,0,0.5; ...
                 0,0,1; ...
                 0.2,0.1,0.9; ...
                 0.2,0.9,0.1; ...
                 0.6,0.1,0.5; ...
                 0.7,0.5, 0.1; ...
                 0.1,0.8,0.3;
                 0.5,0.5,0.5];

%% Plot the bar chart!
j=1;
hold on
for method_idx=1:length(method_and_ctrller_mean(1,:))
    color = method_colors(method_idx,:);
    for ctrl_idx=1:2
        % Index == 1 --> Safety controller OFF during execution
        % Index == 2 --> Safety control ON during execution
        curr_color = color;
        
        % If safety control is off during execution, desaturate color.
        if mod(ctrl_idx, 2) ~= 0
            curr_color = desaturate_color(curr_color);
        end
        offset = 0;
        if ctrl_idx == 2
            offset = -0.2;
        end
        
        % Plot the bar chart.
        br = bar(num_conds(j)+offset, method_and_ctrller_mean(ctrl_idx,method_idx));
        br.FaceColor = curr_color;
        
        % Plot the standard deviation from the mean.
        er = errorbar(num_conds(j)+offset,method_and_ctrller_mean(ctrl_idx,method_idx),errlow(ctrl_idx,method_idx),errhigh(ctrl_idx,method_idx), 's');    
        er.Color = [0 0 0];                            
        er.LineStyle = 'none';
        
        j=j+1;
    end
end

% Make the background of plot white.
set(gcf, 'color', 'w');

% Setup the labels
ylabel(metric_name);
xticks(1.5:2:18.5)
xticklabels(blend_scheme_names)
xtickangle(45)
title(title_name)

%% Desaturates color by 70 percent. 
function curr_color = desaturate_color(color)
    f = 0.7; % desaturate by 70%
    L = 0.3*color(1) + 0.6*color(2) + 0.1*color(3);
    curr_color(1) = color(1) + f * (L - color(1));
    curr_color(2) = color(2) + f * (L - color(2));
    curr_color(3) = color(3) + f * (L - color(3));
end