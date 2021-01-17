function run_and_cache_planners(N, starts, goals)
    for i = 1:length(starts)
        if i > N; break; end
        params = default_hyperparams(); 
        params.start = starts{i};
        params.goal = goals{i};
        params.run_brs = (i == 1); 
        params.run_planner = true;
        exp = load_exp(params); 
        Planner(exp);
        close all;
    end
end 