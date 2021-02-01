function test_reset_planner_follow_unknown_map_termination()
    %% Initial Run
    repo = what("arbitration"); 
    pb_path = strcat(repo.path, '/data/reset_state/planner.mat');
    close all;
%     params = default_hyperparams();
%     params.start = [-2; 0; pi/2; 0.01]; %[-0.375; -1.915; pi/2; 0.01];  %s{1}; 
%     params.goal = [-4; 0; pi/2; 0.01]; 
%     params.blend_scheme = 'none';
%     params.control_scheme = 'switch'; 
%     params.environment_type = 'unknown'; % 'known' 
%     params.run_planner = false; 
%     params.run_brs = false;
%     exp = load_exp(params);
%     pb = Planner(exp);
%     pb.blend_plans();
%     save(pb_path, 'pb'); 
    %% Reset Test
    reset_positions = [0, 3, 9]; 
    failed = 0;
    for i = 1:length(reset_positions) 
        load(pb_path, 'pb'); 
        rp = reset_positions(i);
        output_folder = strcat(repo.path, '/data/reset_state/'); 
        output_mat_path = strcat(output_folder, '/final_state.mat'); 
        pb.output_folder = output_folder; 
        pb.reset_state(rp); 
        pb.blend_plans(); 
        load(pb_path, 'pb'); 
        load(output_mat_path, 'obj'); 
        if ~is_arr_equal(obj.blend_traj, pb.blend_traj)
            failed = failed + 1;
            fprintf("Two trajectories not equivalent\n for reset %d\n", rp); 
        else
            fprintf("Successful reset at index: %d\n", rp); 
        end 
    end 
    fprintf("Failed %d/%d cases\n", failed, length(reset_positions)); 
end 

function equal = is_arr_equal(arr1, arr2, eps)
    if nargin < 3
        eps = 1e-3;
    end 
    if length(arr1(:)) ~= length(arr2(:))
        equal = false; 
    else
        equal = ~any(arr1(:) - arr2(:) > eps, 'all'); 
    end
end 