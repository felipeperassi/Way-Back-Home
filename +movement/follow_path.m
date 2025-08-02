function [robot_state, v_cmd, w_cmd, next_wp_idx] = follow_path(pose, path_points, next_wp_idx)
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    goal_point = path_points(1, :);  % Objetivo final
    dist_to_goal = norm([x, y] - goal_point);

    if dist_to_goal < 0.5
        v_cmd = 0;
        w_cmd = 0;
        robot_state = 'stationary';

    else
        goalReached = false;
        while ~goalReached && next_wp_idx > 1
            wp = path_points(next_wp_idx, :);
            dx = wp(1) - x;
            dy = wp(2) - y;
            dist_to_wp = norm([dx dy]);
            if dist_to_wp < 0.4
                next_wp_idx = next_wp_idx - 1;
            else
                goalReached = true;
            end
        end

        wp = path_points(next_wp_idx, :);
        dx = wp(1) - x;
        dy = wp(2) - y;
        angle_to_wp = atan2(dy, dx);
        angle_diff = wrapToPi(angle_to_wp - theta);
        
        v_cmd = 0.5; 
        w_cmd = 1.5 * angle_diff;
        robot_state = 'navigation';
    end
end