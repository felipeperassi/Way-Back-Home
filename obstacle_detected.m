function obstacle = obstacle_detected(min_dist)
    if min_dist <= 0.4
        obstacle = true;
    else
        obstacle = false;
    end
end