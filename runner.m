function runner()
    close all;
    sigmoids = {'reg_sig'; 'sub'; 'shift_sig'};
    starts = {[3; -2; -pi; 0.01], [-3; 2; -pi; 0.01], [-8; -9.5; 0; 0.01], [1; 1; 0; 0.01]};
    goals = {[3; -4; -pi/2; 0.01], [-4; -1; -pi/2; 0.01], [-5; -8; 0; 0.01], [3, 2.75, pi/2, 0.01]};
    % bad starts: [-4; -3.5; pi/2; 0.01], 
    % bad goals: [-4; -1; -pi/2; 0.01], 
    names = {};
    results = {};
    times = [];
    errors = [];
    exp_cnt = 0;
    for temperature = [1, 0.9, 0.75, 0.5]
        for replan_dt = [0.5, 1.0, 1.5, 2.5, 5]
            for i = 1:length(sigmoids)
                for li = 1:4
                    try 
                        tStart = tic; 
                        params = default_hyperparams();
                        params.plot_every_iter = false;
                        params.start = starts{li};
                        params.goal = goals{li};
                        params.temperature = temperature;
                        params.replan_dt = replan_dt;
                        params.blend_function = sigmoids{i};
                        params.hyperparam_str = sprintf("replan_dt_%.3f_%s_temp_%.3f", params.replan_dt, params.blend_function, params.temperature); 
                        exp = load_exp(params);
                        pb = PlannerBlender(exp); 
                        pb.blend_planners(); 
                        tEnd = toc(tStart);
                        exp_cnt = exp_cnt + 1;
                        times(exp_cnt) = tEnd;
                        names{exp_cnt} = pb.exp_name; 
                        results{exp_cnt} = pb.scores;
                        errors(exp_cnt) = 0;
                    catch 
                        exp_cnt = exp_cnt + 1;
                        times(exp_cnt) = -1;
                        names{exp_cnt} = [];
                        results{exp_cnt} = [];
                        errors(exp_cnt) = 1;
                    end 
                    save('outputs/runner_summary.mat'); 
                end 
            end 
        end 
    end 
    
    for alpha = [1, 0.95, 0.9, 0.75, 0.5]
        for replan_dt = [0.5, 1.0, 1.5, 2.5, 5]
            for li = 1:4
                try 
                    tStart = tic;            
                    params = default_hyperparams();
                    params.plot_every_iter = false;
                    params.blending_scheme = 'constant';
                    params.plot_every_iter = false;
                    params.start = starts{li};
                    params.goal = goals{li};
                    params.alpha = alpha;
                    params.replan_dt = replan_dt;
                    params.hyperparam_str = sprintf("replan_dt_%.3f_alpha_value_%.3f", params.replan_dt, params.alpha); 
                    exp = load_exp(params);
                    pb = PlannerBlender(exp); 
                    pb.blend_planners(); 
                    tEnd = toc(tStart);
                    exp_cnt = exp_cnt + 1;
                    times(exp_cnt) = tEnd;
                    names{exp_cnt} = pb.exp_name; 
                    results{exp_cnt} = pb.scores;
                    errors(exp_cnt) = 0;
                catch 
                    exp_cnt = exp_cnt + 1;
                    times(exp_cnt) = -1;
                    names{exp_cnt} = [];
                    results{exp_cnt} = [];
                    errors(exp_cnt) = 1;
                end 
                save('outputs/runner_summary.mat'); 
            end 
        end 
    end
    
    
    for zero_level_set = [0, 0.1, 0.25, 0.5, 1, 2]
        for replan_dt = [0.5, 1.0, 1.5, 2.5, 5]
            for li = 2:4
                try 
                    tStart = tic;
                    params = default_hyperparams();
                    params.plot_every_iter = false;
                    params.blending_scheme = 'switch';
                    params.start = starts{li};
                    params.goal = goals{li};
                    params.zero_level_set = zero_level_set;
                    params.replan_dt = replan_dt;
                    params.hyperparam_str = sprintf("replan_dt_%.3f_zero_level_set_%.3f", params.replan_dt, params.zero_level_set); 
                    exp = load_exp(params);
                    pb = PlannerBlender(exp); 
                    pb.blend_planners(); 
                    tEnd = toc(tStart);
                    exp_cnt = exp_cnt + 1;
                    times(exp_cnt) = tEnd;
                    names{exp_cnt} = pb.exp_name; 
                    results{exp_cnt} = pb.scores;
                    errors(exp_cnt) = 0;
                catch 
                    exp_cnt = exp_cnt + 1;
                    times(exp_cnt) = -1;
                    names{exp_cnt} = [];
                    results{exp_cnt} = [];
                    errors(exp_cnt) = 1;
                end 
                save('outputs/runner_summary.mat'); 
            end 
        end 
    end
end 