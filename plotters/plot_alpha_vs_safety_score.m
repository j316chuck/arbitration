function plot_alpha_vs_safety_score()
    exp_types = {
        'blending_scheme_probabilistic_blend_safety_control_traj_replan_dt_1.500_num_samples_20_use_safe_1_level_set_0.20_spline_obs_weight_1.000000'
    };
    %% Get All Experiment Results Data
    Nt = length(exp_types); 
    all_exp_metrics = {}; % maps exp_types -> list of metrics
    all_exp_names = {};   % maps exp_types -> list of exp names
    for i = 1:Nt
        all_exp_metrics{i} = zeros(0, 7); 
        all_exp_names{i} = {}; 
    end  
    path = '../old_outputs/option_1_5_prob_control';
    dirs = dir(fullfile(path));
    N = length(dirs);
    orig_traj_avg_safety_score = [];
    safety_traj_avg_safety_score = [];
    mpc_alphas = [];
    difference_safety = []; 
    for eti = 1:Nt % experiment_type_index
        exp_type = exp_types{eti};
        for j = 1:N
            s = dirs(j, :);
            if contains(s.name, exp_type)
                exp_name = fullfile(s.folder, s.name); 
                f = fullfile(exp_name, 'final_state.mat'); 
                names = all_exp_names{eti};
                names{end+1} = exp_name; 
                all_exp_names{eti} = names;
                metrics = all_exp_metrics{eti}; 
                if isfile(f)
                    load(f); 
                    N = size(obj.orig_traj, 2);
                    for it=1:obj.num_mpc_steps:N
                        st = it;
                        et = min(it+obj.num_mpc_steps-1, N);
                        safety_score = 0; 
                        for jt=st:et
                            state = [obj.orig_traj(1, jt); obj.orig_traj(2, jt); obj.orig_traj(3, jt)];
                            safety_score = safety_score + obj.brs_planner.get_value(state); 
                        end 
                        mean_ss = safety_score / (et - st + 1); 
                        orig_traj_avg_safety_score(end+1) = mean_ss; 
                                                safety_score = 0; 
                        for jt=st:et
                            state = [obj.safety_traj(1, jt); obj.safety_traj(2, jt); obj.safety_traj(3, jt)];
                            safety_score = safety_score + obj.brs_planner.get_value(state); 
                        end 
                        mean_ss = safety_score / (et - st + 1); 
                        safety_traj_avg_safety_score(end+1) = mean_ss; 
                        mpc_alphas(end+1) = max(obj.blend_traj(6, it), 0);
                        difference_safety(end+1) = safety_traj_avg_safety_score(end) - orig_traj_avg_safety_score(end); 
                    end 
                else 
                    metrics(end+1, :) = -ones(7, 1)' * 1; 
                    fprintf("Failed exp: %s\n", exp_name); 
                end 
                figure(1);
                clf;
                hold on;
                xlabel("safety score");
                ylabel("alpha");
                title("Alpha vs Safety Score");
                scatter(difference_safety, mpc_alphas, 30, 'gx', 'DisplayName', 'difference'); 
                scatter(orig_traj_avg_safety_score, mpc_alphas, 30, 'rx', 'DisplayName', 'orig_traj');
                scatter(safety_traj_avg_safety_score, mpc_alphas, 30, 'bo', 'DisplayName', 'safety_traj');
                legend('Interpreter', 'None');
                all_exp_metrics{eti} = metrics;
            end 
        end
    end
    
    X = difference_safety;
    y = mpc_alphas;
    b = glmfit(X, y, 'binomial','link','logit'); 
    fprintf("%f", b); 
end 