function plot_exp_all()
    verbose = false;
    if ~verbose 
        f = {@plot_exp_trajectories_all, @plot_exp_with_aggregation, @output_analysis}; 
        num_workers = 3; 
        parfor (i = 1:length(f), num_workers)
            plot_function = f{i};
            plot_function();
        end 
    else
        plot_exp_with_aggregation(); 
        plot_exp_trajectories_all();
    end 
end
