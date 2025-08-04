function [robot_state, v_cmd, w_cmd, next_wp_idx] = follow_path(pose, path_points, next_wp_idx)
    % FOLLOW_PATH - Controlador de navegación para seguir una trayectoria.
    %
    % Entradas:
    %   pose          - Vector con la pose actual del robot [x, y, theta]
    %   path_points   - Matriz Nx2 con los puntos del camino a seguir (desde el objetivo hasta el inicio)
    %   next_wp_idx   - Índice del siguiente waypoint a alcanzar
    %
    % Salidas:
    %   robot_state   - Estado del robot: 'navigation' o 'stationary'
    %   v_cmd         - Velocidad lineal (en m/s)
    %   w_cmd         - Velocidad angular (en rad/s)
    %   next_wp_idx   - Índice actualizado del próximo waypoint a alcanzar

    % Declarar parámetros de la pose del robot
    x = pose(1);
    y = pose(2);
    theta = pose(3);

    % Calcular la distancia al objetivo
    goal_point = path_points(1, :);  
    dist_to_goal = norm([x, y] - goal_point);

    % Si el robot está muy cerca del objetivo, se detiene
    if dist_to_goal < 0.5
        v_cmd = 0;
        w_cmd = 0;
        robot_state = 'stationary';
    
    else
        goalReached = false;

        % Se busca el waypoint más cercano aún no alcanzado
        while ~goalReached && next_wp_idx > 1

            % Calcular la distancia al waypoint actual
            wp = path_points(next_wp_idx, :);
            dx = wp(1) - x;
            dy = wp(2) - y;
            dist_to_wp = norm([dx dy]);

            % Si el waypoint actual está muy cerca, se pasa al siguiente
            if dist_to_wp < 0.4
                next_wp_idx = next_wp_idx - 1;

            % Si no, detener búsqueda
            else
                goalReached = true;
            end
        end

        % Se recalculan los comandos hacia el waypoint objetivo
        wp = path_points(next_wp_idx, :);
        dx = wp(1) - x;
        dy = wp(2) - y;

        % Ángulo deseado hacia el waypoint
        angle_to_wp = atan2(dy, dx);
        angle_diff = wrapToPi(angle_to_wp - theta);
        
        % Se ajustan las velocidades lineal y angular
        v_cmd = 0.3; 
        w_cmd = 1.5 * angle_diff;
        robot_state = 'navigation';
    end
end