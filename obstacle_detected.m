function obstacle = obstacle_detected(min_dist)
    if min_dist <= 0.3
        obstacle = true;
    else
        obstacle = false;
    end
end