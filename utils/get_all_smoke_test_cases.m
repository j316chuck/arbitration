function [starts, goals] = get_all_smoke_test_cases()
    starts = {
             [-0.375; -1.915; pi; 0.01; 0], ... % open - open
             [-0.375; -1.915; pi; 0.01; 0], ... % open - tight
             [0.395; 3.86; pi/2; 0.01; 0], ... % tight - tight
             [-0.375; -1.915; pi; 0.01; 0], ... % open - obstacle
             [0.395; 3.86; -pi/2; 0.01; 0], ... % tight - tight
    };     
    goals = {
             [0.78; 1.55; pi/2; 0.01; 0], ... % open - open
             [-0.375; 2.705; pi; 0.01; 0],... % open - tight
             [0.395; 2.705; pi/2; 0.01; 0], ... % tight - tight
             [-4.61; -0.76; pi; 0.01; 0], ... % open - obstacle
             [0.395; 2.705; pi/2; 0.01; 0], ... % hard angle
    }; 
end 