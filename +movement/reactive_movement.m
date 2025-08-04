function [v_cmd, w_cmd, count_react, robot_state] = reactive_movement(v_cmd, w_cmd, max_angle, obstacle, robot_state, count_react, is_localizing)
    % REACTIVE_MOVEMENT - Control reactivo para evitar obstáculos durante el movimiento del robot.
    %
    % Entradas:
    %   v_cmd         - Velocidad lineal actual (m/s)
    %   w_cmd         - Velocidad angular actual (rad/s)
    %   max_angle     - Ángulo de desvío máximo (rad)
    %   obstacle      - Booleano que indica si se detectó un obstáculo
    %   robot_state   - Estado actual del robot (string)
    %   count_react   - Contador de pasos en estado reactivo 
    %   is_localizing - Booleano que indica si el robot está en fase de localización
    %
    % Salidas:
    %   v_cmd         - Nueva velocidad lineal (m/s)
    %   w_cmd         - Nueva velocidad angular (rad/s)
    %   count_react   - Contador actualizado de reacciones frente a obstáculos
    %   robot_state   - Nuevo estado del robot
    
    % Reacción ante detección de obstáculo, en distintas fases según un contador
    if obstacle 
        
        % El robot reacciona al obstáculo deteniéndose y girando hacia el ángulo máximo permitido
        if count_react < 10
            disp('Obstaculo detectado, deteniendo robot');
            v_cmd = 0;
            w_cmd = 1.5 * max_angle;    %min(0.5 * max_angle, 0.5);
            count_react = count_react + 1;
        
        % Avanzar lentamente mientras se sigue girando para no quedar atascado en ángulos repetitivos
        elseif count_react >= 10 && count_react < 15
            disp('Obstaculo detectado, girando para evitar');
            v_cmd = 0.05;
            w_cmd = 1.5 * max_angle;    %min(0.5 * max_angle, 0.5);
            count_react = count_react + 1;
        
        % Se resetea el contador luego de un tiempo y se verifica el estado del robot
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

    % Si no hay obstáculo, se reinicia el contador y se ajustan las velocidades
    else
        count_react = 0;

        % El robot se está localizando, entonces se mueve lentamente
        if is_localizing
            v_cmd = 0.5; 
            w_cmd = 1.5 * max_angle;    %min(0.5 * max_angle, 0.5);
            robot_state = 'localization';
        
        % El robot está por calcular destino, entonces se detiene
        else
            v_cmd = 0;
            w_cmd = 0;
            robot_state = 'calculate_destiny';
        end
    end
end