function [v_cmd, w_cmd, count_react, robot_state] = reactive_movement(v_cmd, w_cmd, max_angle, obstacle, robot_state, count_react, is_localizing)
    if obstacle 

        if count_react < 10
            disp('Obstaculo detectado, deteniendo robot');
            v_cmd = 0;
            w_cmd = min(0.5 * max_angle, 0.5);
            count_react = count_react + 1;
        
        elseif count_react >= 10 && count_react < 15
            disp('Obstaculo detectado, girando para evitar');
            v_cmd = 0.05;
            w_cmd = min(0.5 * max_angle, 0.5);
            count_react = count_react + 1;
        
        else
            count_react = 0;

            if is_localizing
                robot_state = 'localization';
            else
                v_cmd = 0;
                w_cmd = 0;
                robot_state = 'calculate_destiny';
            end
        end

    else
        count_react = 0;

        if is_localizing
            v_cmd = 0.5;  % Con 0.5 el simulador se ve mal, pero en la práctica anda bien. Además, 0.1 es muy chico para localizarlo.
            w_cmd = min(0.5 * max_angle, 0.5);
            robot_state = 'localization';
        else
            v_cmd = 0;
            w_cmd = 0;
            robot_state = 'calculate_destiny';
        end
    end
end