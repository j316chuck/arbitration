function point_nav_str = get_point_nav_str(params)
    start_str = sprintf("start_[%.2f %.2f %.2f]", params.start(1), params.start(2), params.start(3)); 
    goal_str = sprintf("goal_[%.2f %.2f %.2f]", params.goal(1), params.goal(2), params.goal(3)); 
    point_nav_str = sprintf("%s_map_%s_%s", params.map_basename, start_str, goal_str); 
end 