function [starts, goals] = get_point_nav_tasks(type)
    repo = what("arbitration"); 
    if strcmp(type, "smoke")
        [starts, goals] = get_all_smoke_test_cases(); 
    elseif strcmp(type, "sampled")
        path = strcat(repo.path, '/data/sampled_goals.mat'); 
        load(path, 'starts', 'goals'); 
        starts = num2cell(starts', 1); 
        goals = num2cell(goals', 1); 
    else
        warning("No nav task of type %s found", type); 
        return; 
    end
end
