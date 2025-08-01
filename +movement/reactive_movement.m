function [v_cmd, w_cmd, count_react, robot_state] = reactive_movement(v_cmd, w_cmd, max_angle, obstacle, robot_state, count_react, is_localizing)
    if is_localizing
        robot_state = "localization";
        v_cmd = 0.5;
        w_cmd = 0.5;
    end

    if obstacle && count_react < 50
        disp('Obstaculo detectado, deteniendo robot');
        v_cmd = 0;
        w_cmd = min(0.5 * max_angle, 0.5);
        count_react = count_react + 1;
            
    elseif obstacle && count_react >= 50 && count_react < 75
        disp('Obstaculo detectado, girando para evitar');
        v_cmd = 0.05;
        w_cmd = min(0.5 * max_angle, 0.5);
        count_react = count_react + 1;
    
    elseif obstacle && count_react >= 75 && ~is_localizing
        count_react = 0;
        robot_state = 'calculate_destiny';

    elseif ~obstacle && ~is_localizing
        count_react = 0;
        robot_state = 'calculate_destiny'; 
    end
end