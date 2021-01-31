function plot_exp_trajectories_all(startN, endN)
    if nargin < 2
        startN = 1;
        endN = 100;
    end 
    for i = startN:endN
        plot_exp_trajectories(i);
    end 
end 